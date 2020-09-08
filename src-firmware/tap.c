
/*
 * Based on tapuino.c
 * 
 * TODO only avr and sd
 */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "bitbanging.h"
#include "utils.h"

#include "ff.h"
#include "timer.h"

typedef enum
{
  C64,
  VIC,
  C16,
} MACHINE_TYPE;

typedef enum
{
  PAL,
  NSTC,
} VIDEO_MODE;


// magic strings found in the TAP header, represented as uint32_t values in little endian format (reversed string)
#define TAP_MAGIC_C64      0x2D343643   // "C64-"  as "-46C"
#define TAP_MAGIC_C16      0x2D363143   // "C16-"  as "-61C"
#define TAP_MAGIC_POSTFIX1 0x45504154   // "TAPE"  as "EPAT"
#define TAP_MAGIC_POSTFIX2 0x5741522D   // "-RAW"  as "WAR-"

struct TAP_INFO {
  uint8_t version;              // TAP file format version:
                                // Version  0: 8-bit data, 0x00 indicates overflow
                                //          1: 8-bit, 0x00 indicates 24-bit overflow to follow
                                //          2: same as 1 but with 2 half-wave values
  uint8_t platform;             // Platform 0: C64
                                //          1: VIC
                                //          2: C16
  uint8_t video;                // Video    0: PAL 
                                //          1: NTSC
  uint8_t reserved;
  volatile uint32_t length;     // total length of the TAP data excluding header in bytes
  volatile uint32_t cycles;     // 
};

static struct TAP_INFO g_tap_info;

// the maximum TAP delay is a 24-bit value i.e. 0xFFFFFF cycles
// we need this constant to determine if the loader has switched off the motor before the tap has completed
// which would cause the code to enter an endless loop (when the motor is off the buffers do not progress)
// so we check during the buffer wait loop to see if we have exceeded the maximum delay time and exit if so.
#define MAX_SIGNAL_CYCLES     (g_cycle_mult_raw * 0xFFFFFF * 2)

// helper variables for the ISR and loader code

#define TAP_BUF_SIZE 64
#define TAP_BUF_HALF_SIZE 32
static volatile uint8_t g_tap_buffer[TAP_BUF_SIZE];
static FIL g_fil;

static volatile uint8_t g_read_index;           // read index in the g_tap_buffer buffer
static volatile uint8_t g_write_index;          // write index in the g_tap_buffer buffer
static volatile uint32_t g_total_timer_count;   // number of (AVR) cycles that the timer has been running for
static volatile uint8_t g_tap_file_complete;    // flag to indicate that all bytes have been read from the TAP
static volatile uint32_t g_tap_file_pos;        // current read position in the TAP (bytes)
static bool g_first_half = false;
static uint32_t g_pulse_length = 0;             // length of pulse in uS
static uint32_t g_pulse_length_save;            // save length for read

static double g_cycle_mult_raw = 1.0;
static double g_cycle_mult_8 = 8.0;

static void setup_cycle_timing(void) {
  double ntsc_cycles_per_second = 1022272;
  double pal_cycles_per_second = 985248;

  switch (g_tap_info.platform)
  {
    case C64:
      ntsc_cycles_per_second = 1022272;
      pal_cycles_per_second  = 985248;
    break;
    case VIC:
      ntsc_cycles_per_second = 1022727;
      pal_cycles_per_second  = 1108404;
    break;
    case C16:
      ntsc_cycles_per_second = 894886;
      pal_cycles_per_second  = 886724;
    break;
  }
  
  switch(g_tap_info.video)
  {
    case PAL:
      g_cycle_mult_raw = (1000000.0 / pal_cycles_per_second);
    break;
    case NSTC:
      g_cycle_mult_raw = (1000000.0 / ntsc_cycles_per_second);
    break;
  }
  g_cycle_mult_8   = (g_cycle_mult_raw * 8.0);
}

static inline uint8_t next_tap_byte(void) {
  uint8_t b = g_tap_buffer[g_read_index];
  g_read_index++;
  g_read_index &= (TAP_BUF_SIZE-1);
  g_tap_file_pos++;
  return b;
}

static uint32_t next_tap_data(void) {
  uint32_t tap_data = (uint32_t) next_tap_byte();
  // code for format 0 handling
  if (g_tap_info.version == 0 && tap_data == 0) {
    tap_data = 256;
  }        
  if (tap_data != 0) {
    tap_data *= g_cycle_mult_8;
  } else {
    tap_data =  (uint32_t) next_tap_byte();
    tap_data |= ((uint32_t) next_tap_byte()) << 8;
    tap_data |= ((uint32_t) next_tap_byte()) << 16;
    tap_data *= g_cycle_mult_raw;
  }
  if (g_tap_info.version == 2) {
    // format 2 is half-wave and timer is running at 2Mhz so double
    tap_data <<= 1;
  }
  return tap_data;
}

// timer1 is running at 2MHz or 0.5 uS per tick.
// signal values are measured in uS, so OCR1A is set to the value from the TAP file (converted into uS) for each signal half
// i.e. TAP value converted to uS * 2 == full signal length
ISR(TIMER1_COMPA_vect) {
  // keep track of the number of cycles in case we get to a MOTOR stop situation before the TAP has completed
  g_total_timer_count += OCR1A;
  
  // don't process if the MOTOR is off!
  if (!get_motor()) {
    return;
  }

  if (!g_pulse_length) {
    // previous pulse is complete
    set_read(g_first_half);
    if (g_first_half) {
      // end of first half
      if (g_tap_info.version != 2) {
        g_pulse_length = g_pulse_length_save;
      } else {
        g_pulse_length = next_tap_data();
      }
    } else {
      // end of second half
      if (g_tap_file_pos >= g_tap_info.length) {
        // reached the end of the TAP file so don't process any more!
        g_tap_file_complete = 1;
        return;
      }

      g_pulse_length = next_tap_data();   
      //if (g_tap_info.version != 2) {
      // save this for the 2nd half of the wave
      g_pulse_length_save = g_pulse_length;
      //}
    }
    g_first_half = !g_first_half;
  }

  // setup timer for the next interrupt
  // check to see if its bigger than 16 bits
  if (g_pulse_length > 0xFFFF) {
    OCR1A = 0xFFFF;
    g_pulse_length -= 0xFFFF;
  } else {
    OCR1A = (uint16_t) g_pulse_length;
    // clear this to get next pulse
    g_pulse_length = 0;
  } 
}

static void signal_timer_start(void) {
  TCCR1A = 0x00;   // clear timer registers
  TCCR1B = 0x00;
  TIMSK1 = 0x00;
  TCNT1  = 0x00;

  g_total_timer_count = 0;
  // pre-scaler 16 MHZ / 8 = 2 MHZ, CTC Mode
  TCCR1B |=  _BV(CS11) | _BV(WGM12);
  OCR1A = 0xFFFF;
  OCR1B = 0xFFFF;
  // output compare interrupt
  TIMSK1 |=  _BV(OCIE1A);
}

static void signal_timer_stop(void) {
  // stop all timer1 interrupts
  TIMSK1 = 0;
}

static int verify_tap(void) {
  FRESULT res;
  UINT br;

  memset(&g_tap_info, 0, sizeof(g_tap_info));

  res = f_read(&g_fil, (void*) g_tap_buffer, 12, &br);
  if (res != FR_OK) {
    LOGNL("S_READ_FAILED");
    return 0;
  }
  res = f_read(&g_fil, (void*) &g_tap_info, 8, &br);
  if (res != FR_OK) {
    LOGNL("S_READ_FAILED");
    return 0;
  }
  // check size first
  if (g_tap_info.length != (f_size(&g_fil) - 20)) {
    LOGNL("S_INVALID_SIZE");
    g_tap_info.length = f_size(&g_fil) - 20;
  }
  
  uint32_t* tap_magic = (uint32_t*) g_tap_buffer;
  // check the post fix for "TAPE-RAW", use a 4-byte magic trick
  if (tap_magic[1] != TAP_MAGIC_POSTFIX1 || tap_magic[2] != TAP_MAGIC_POSTFIX2) {
    LOGNL("S_INVALID_TAP");
    return 0;
  }
  // now check type: C16 or C64, use a 4-byte magic trick
  if (tap_magic[0] != TAP_MAGIC_C64 && tap_magic[0] != TAP_MAGIC_C16 ) {
    LOGNL("S_INVALID_TAP");
    return 0;
  }

  return 1;
}

int tap_play_file(const char *filename)
{
  UINT br;
  FRESULT res;
  uint32_t bytes_read = 0;
  //int perc = 0;
  
  LOG("PLAY ");
  LOGNL((char*)filename);

  res = f_open(&g_fil, filename, FA_READ);
  if (res != FR_OK) {
    LOGNL("S_OPEN_FAILED");
    return 0;
  }

  if (!verify_tap()) {
    return 0;
  }

  setup_cycle_timing();

  // setup all start conditions
  g_write_index = TAP_BUF_HALF_SIZE;
  g_read_index = 0;
  g_tap_file_pos = 0;
  g_pulse_length = g_pulse_length_save = 0;
  g_first_half = false;
  g_tap_file_complete = 0;

  LOGNL("LOADING");
  // get the first buffer ready
  res = f_read(&g_fil, (void*) g_tap_buffer, TAP_BUF_SIZE, &br);
  if (res != FR_OK) {
    LOGNL("S_READ_FAILED");
    return 0;
  }
  bytes_read += br;

  set_read(true);
  set_sense(false);
  // start send-ISR
  signal_timer_start();

  while (br > 0) {
    set_led((g_read_index & TAP_BUF_HALF_SIZE) ? true : false);

    // Wait until ISR is in the new half of the buffer
    while ((g_read_index & TAP_BUF_HALF_SIZE) == (g_write_index & TAP_BUF_HALF_SIZE)
        && !g_tap_file_complete);

    if (g_tap_file_complete) {
      break;
    }
    
    res = f_read(&g_fil, (void*) g_tap_buffer + g_write_index, TAP_BUF_HALF_SIZE, &br);
    bytes_read += br;
    if (res != FR_OK && bytes_read < g_tap_info.length) {
      LOGNL("READ FAILED");
      // TODO maybe end of file
      //break;
    }
    g_write_index += TAP_BUF_HALF_SIZE;
    g_write_index &= (TAP_BUF_SIZE - 1);
    //perc = (g_tap_file_pos * 100) / g_tap_info.length;
  }
  LOGNL("FILE END");
  // wait for the remaining buffer to be read.
  while (!g_tap_file_complete) {
    // we need to do the same trick as above, BC's Quest for Tires stops the motor right near the
    // end of the tape, then restarts for the last bit of data, so we can't rely on the motor signal
    // a better approach might be to see if we have read all the data and then break. //
    if (g_total_timer_count > MAX_SIGNAL_CYCLES) {
      break;
    }
  }

  signal_timer_stop();
  f_close(&g_fil);
  
  set_sense(true);
  set_read(true);
  set_led(false);

  LOGNL("TAP COMPLETE");
  return res == FR_OK;
}
