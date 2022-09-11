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
#ifndef _STM32_COMMON_H_
#define _STM32_COMMON_H_

#ifdef STM32F4XX
    #include "stm32f4xx.h"
#elif STM32F3XX
    #include "stm32f3xx.h"
#elif STM32F0XX
    #include "stm32f0xx.h"

    #define DMA1_Channel4_7_IRQn DMA1_Channel4_5_IRQn

    #undef USB_EP_TYPE_MASK
    #undef USB_EP_BULK
    #undef USB_EP_CONTROL
    #undef USB_EP_ISOCHRONOUS
    #undef USB_EP_INTERRUPT
    #undef USB_EP_T_MASK
#elif STM32L0XX
    #include "stm32l0xx.h"

    #define DMA1_Channel4_7_IRQn DMA1_Channel4_5_6_7_IRQn
    #define DMA_TARGET_SOURCE_SELECT
#elif STM32F1XX
    #include "stm32f1xx.h"

    #define USB_IRQn USB_LP_CAN1_RX0_IRQn
#elif STM32G0XX
    #include "stm32g0xx.h"
#else
    #error "STM32 target not recognized. Missing define"
#endif

#include "core.h"

#define GPIO_PORT_MASK  (uint16_t)(0xFFF0)
#define GPIO_PIN_MASK   (uint16_t)(0xF)

#define DEFINE_PIN(port,pin) (gpio_pin_t)(((uint32_t)(port) & GPIO_PORT_MASK) | ((uint16_t)(pin) & GPIO_PIN_MASK))

#ifdef STM_NUCLEO32
    #include "nucleo32_pindef.h"
#endif
#ifdef STM_NUCLEO
    #include "nucleo_pindef.h"
#endif

#ifdef __cplusplus
    extern "C" {
#endif

typedef uint16_t gpio_pin_t;

typedef enum {
    MODE_OUT_PP,
    MODE_OUT_OD,
    MODE_IN,
    MODE_AN
} gpio_mode_t;

void stm32_gpio_init(gpio_pin_t pin,gpio_mode_t mode);
void stm32_gpio_af(gpio_pin_t pin,gpio_mode_t mode,int af_number);
void stm32_gpio_pullup(gpio_pin_t pin);
void stm32_gpio_pulldown(gpio_pin_t pin);
void stm32_target_clken(GPIO_TypeDef* port);
void stm32_common_init(void);
int stm32_gpio_read(gpio_pin_t pin);

GPIO_TypeDef* stm32_get_port(gpio_pin_t pin);
uint8_t stm32_get_pindef(gpio_pin_t pin);

#ifdef __cplusplus
    }
#endif

#endif
