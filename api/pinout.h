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
/**
 * \file target.h
 * \brief This file declares macros for pinout reporting
 */
#ifndef _API_PINOUT_H_
#define _API_PINOUT_H_

#ifdef __cplusplus
    extern "C" {
#endif

typedef enum {
    PINOUT_NONE = 0,
    PINOUT_SO8 = 1,
    PINOUT_TSSOP20 = 2,
    PINOUT_LQFP32 = 3,
    PINOUT_ARDUINO = 4,
    PINOUT_PKG_GENERIC_FLAG = 0x80,
} pinout_package_t;

#define PINOUT_GENERIC_PACKAGE(pins) (PINOUT_PKG_GENERIC_FLAG | (pins/2))

typedef enum {
    PINOUT_OSC = 1,
    PINOUT_PWM = 2,
    PINOUT_VOLT = 3,
    PINOUT_PWM_IN = 4,
    PINOUT_GEN = 5,
    PINOUT_PULSE = 6,
    PINOUT_LOG = 7,
} pinout_function_t;

typedef enum {
    PINOUT_GND = 1,
    PINOUT_VDD = 2,
    PINOUT_VDDA = 3,
    PINOUT_VDDX = 4,
    PINOUT_NRST = 5,
    PINOUT_SWDIO = 6,
    PINOUT_SWCLK = 7,
    PINOUT_UART_RX = 8,
    PINOUT_UART_TX = 9,
    PINOUT_BOOT0 = 10,
    PINOUT_USB_DP = 11,
    PINOUT_USB_DM = 12,
    PINOUT_START_BLINK = 13,
    PINOUT_5V = 14,
} pinout_system_t;

#define PINOUT_OUTSIDE_PIN  (0xFF)

#define PINOUT_PINDEF_SIZE  (3)
#define PINOUT_SIZE(x)  (1 + (PINOUT_PINDEF_SIZE*(x)))
#define PINOUT_PIN(port, number)        (((port - 'A') << 4) | (number & 0xf))
#define PINOUT_FUNC(function, channel)  (((function & 0xf) << 4) | (channel & 0xf))

#define PINOUT_SPEC(pkg_number, port, gpio_number, function, channel) \
    pkg_number, PINOUT_PIN(port, gpio_number), PINOUT_FUNC(function, channel)
#define PINOUT_SYS(pkg_number, function) pkg_number, 0xff, (function & 0xf)
#define PINOUT_CORE(pkg_number, port, gpio_number, function) \
    pkg_number, PINOUT_PIN(port, gpio_number), (function & 0xf)

#define PINOUT_ADD_SPEC(array, index, pkg_number, port, \
    gpio_number, function, channel) \
    do { \
      array[index] = pkg_number; \
      array[index+1] = PINOUT_PIN(port,gpio_number); \
      array[index+2] = PINOUT_FUNC(function, channel); \
      index += PINOUT_PINDEF_SIZE; \
    } while(0)
#define PINOUT_ADD_SYS(array, index, pkg_number, function) \
    do { \
      array[index] = pkg_number; \
      array[index+1] = 0xff; \
      array[index+2] = (function & 0xf); \
      index += PINOUT_PINDEF_SIZE; \
    } while(0)
#define PINOUT_ADD_CORE(array, index, pkg_number, port, gpio_number, function) \
    do { \
      array[index] = pkg_number; \
      array[index+1] = PINOUT_PIN(port,gpio_number); \
      array[index+2] = (function & 0xf); \
      index += PINOUT_PINDEF_SIZE; \
    } while(0)
    

#ifdef __cplusplus
  }
#endif

#endif
