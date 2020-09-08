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


   utils.c: Misc utility function

*/

#include "config.h"
#include "bitbanging.h"
#include "timer.h"
#include "utils.h"
#include "uart.h"

#define TIME_BASE 200 // milliseconds

void blink_value(uint16_t value, uint8_t bits) {
  set_led(false);
  delay_ms(TIME_BASE);

  while (bits-- > 0) {
    set_led(true);

    if (value & (1 << bits)) {
      delay_ms(TIME_BASE * 3);
    } else {
      delay_ms(TIME_BASE);
    }

    set_led(false);
    delay_ms(TIME_BASE);
  }
}

#ifdef UART_LOG
void LOG(const char *p) {
  uart_puts(p);
}

void LOGNL(const char *p) {
  uart_puts(p);
  uart_putcrlf();
  uart_flush();
}

void LOGFLUSH(void) {
  uart_flush();
}
#endif

