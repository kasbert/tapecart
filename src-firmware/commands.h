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


   commands.h: command numbers shared between the tapecart and its flashtool

*/

#ifndef COMMAND_H
#define COMMAND_H

typedef enum {
  CMD_EXIT = 0,
  CMD_READ_DEVICEINFO,
  CMD_READ_DEVICESIZES,
  CMD_READ_CAPABILITIES,

  CMD_READ_FLASH  = 0x10,
  CMD_READ_FLASH_FAST,
  CMD_WRITE_FLASH,
  CMD_WRITE_FLASH_FAST, // FIXME: Not Yet Implemented
  CMD_ERASE_FLASH_64K,
  CMD_ERASE_FLASH_BLOCK,
  CMD_CRC32_FLASH,

  CMD_READ_LOADER = 0x20,
  CMD_READ_LOADINFO,
  CMD_WRITE_LOADER,
  CMD_WRITE_LOADINFO,

  CMD_LED_OFF = 0x30,
  CMD_LED_ON,
  CMD_READ_DEBUGFLAGS,
  CMD_WRITE_DEBUGFLAGS,

  CMD_DIR_SETPARAMS = 0x40,
  CMD_DIR_LOOKUP,
  
  /* SD commands */
  CMD_SD_OPEN_DIR = 0x80,
  CMD_SD_READ_DIR_FAST,
  CMD_SD_SELECT_FILE,

  /* internal use only */
  CMD_RAMEXEC = 0xf0,
} command_t;

#define FILENAME_LENGTH 20
typedef struct direlement {
    char name[FILENAME_LENGTH+1];
    uint32_t size;
    uint8_t type;
} DirElement;


#endif
