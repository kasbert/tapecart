/* tapecart - a tape port storage pod for the C64

   Copyright (C) 2013-2017  Ingo Korb <ingo@akana.de>
   All rights reserved.
   Idea by enthusi

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
   OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
   SUCH DAMAGE.


   timer.h: Wrapper functions to abstract timer accesses

*/

#ifndef TIMER_H
#define TIMER_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

#define delay_ms(x) _delay_ms(x)
#define delay_us(x) _delay_us(x)

#if CONFIG_HARDWARE_VARIANT != 5 && CONFIG_HARDWARE_VARIANT != 6
#define PULSETIMER_HANDLER ISR(TIM0_COMPA_vect)
#else
#define PULSETIMER_HANDLER ISR(TIMER0_COMPA_vect)
#endif

#ifdef HAVE_SD
#define SD_TIMER_HANDLER ISR(TIMER2_COMPA_vect)

static inline void sd_timer_init(void) {
  /* Start 100Hz system timer */
  OCR2A = F_CPU / 1024 / 100 - 1;
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  TIMSK2 = _BV(OCIE2A);
}
#endif

static inline void pulsetimer_init(void) {
  OCR0A  = 0xff; // start with a long(ish) pause
  TCCR0A = _BV(WGM01);
  TIMSK0 = _BV(OCIE0A);
}

static inline void pulsetimer_set_pulselen(uint8_t len) {
#if CONFIG_HARDWARE_VARIANT != 5 && CONFIG_HARDWARE_VARIANT != 6
  OCR0A = len;
#else
  /* adjust for higher clock frequency */
  OCR0A = len * 2;
#endif
}

/* enable/disable pulse ISR */
static void inline pulsetimer_enable(bool state) {
  if (state)
    TCCR0B = _BV(CS01) | _BV(CS00); // starts timer
  else
    TCCR0B = 0;
}

#endif
