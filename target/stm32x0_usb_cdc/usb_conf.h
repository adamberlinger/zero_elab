/*
 * BSD 3-Clause License
 * 
 * Copyright (c) 2016-2022, Adam Berlinger
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _USB_CONF_H
#define _USB_CONF_H

#include "stm32_common.h"

#ifdef __cplusplus
    extern "C" {
#endif

#ifdef STM32C0XX
#define USB_SRAM_START_ADDRESS (0x40009800)
#define USB_SRAM_32ACCESS
#else
#define USB_SRAM_START_ADDRESS (0x40006000)
#endif

#define usb_ms_delay           wait_ms
#define usb_interrupt          usb_handler

#ifdef STM32F1XX
  #define USB_SRAM_32BIT
#endif

#ifdef STM32C0XX
  #define USB USB_DRD_FS
  #define EP0R CHEP0R
  #define EP1R CHEP1R
  #define EP2R CHEP2R
  #define EP3R CHEP3R
  #define EP4R CHEP4R
  #define EP5R CHEP5R
  #define EP6R CHEP6R
  #define EP7R CHEP7R
  #define USB_CNTR_FRES USB_CNTR_USBRST
#endif

#ifdef __cplusplus
    }
#endif

#endif
