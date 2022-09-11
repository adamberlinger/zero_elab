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
#ifndef _TIMER_TARGET_DB_H_
#define _TIMER_TARGET_DB_H_

#include <stdint.h>
#include "timer.h"
#include "stm32_dma.h"

#ifdef __cplusplus
    extern "C" {
#endif

#define TIM_SLAVE_DB_SIZE (28)

#ifdef GPIOE
    #define TIM1_PIN_SIZE      12
#else
    #define TIM1_PIN_SIZE      8
#endif

#define TIM2_PIN_SIZE          15

#ifdef GPIOE
    #define TIM3_PIN_SIZE      16
#else
    #define TIM3_PIN_SIZE      12
#endif

#ifdef TIM4
    #define TIM4_CONFIG_SIZE   1
    #define TIM4_PIN_SIZE      12
#else
    #define TIM4_CONFIG_SIZE   0
    #define TIM4_PIN_SIZE      0
#endif

#ifdef TIM8
    #define TIM8_CONFIG_SIZE   1
    #define TIM8_PIN_SIZE      10
#else
    #define TIM8_CONFIG_SIZE   0
    #define TIM8_PIN_SIZE      0
#endif

#define TIM15_PIN_SIZE          4

#ifdef GPIOE
    #define TIM16_PIN_SIZE      5
#else
    #define TIM16_PIN_SIZE      4
#endif

#ifdef GPIOE
    #define TIM17_PIN_SIZE      4
#else
    #define TIM17_PIN_SIZE      3
#endif

#ifdef TIM20
    #define TIM20_CONFIG_SIZE   1
    #define TIM20_PIN_SIZE      0
#else
    #define TIM20_CONFIG_SIZE   0
    #define TIM20_PIN_SIZE      0
#endif

#define TIM_PIN_DB_SIZE    (TIM1_PIN_SIZE + TIM2_PIN_SIZE + TIM3_PIN_SIZE + \
        TIM4_PIN_SIZE + TIM8_PIN_SIZE + TIM15_PIN_SIZE + TIM16_PIN_SIZE + \
        TIM17_PIN_SIZE + TIM20_PIN_SIZE)

#define TIM_CONFIG_DB_SIZE (8 + TIM4_CONFIG_SIZE + TIM8_CONFIG_SIZE + TIM20_CONFIG_SIZE)

#define TIM_DMA_DB_SIZE (8 + TIM4_CONFIG_SIZE)

#define TIM_TARGET_APB_PRESCALERS

#define TIM_ETR_DB_SIZE (10)

extern const timer_pin_db_t timer_pin_db[TIM_PIN_DB_SIZE];
extern const timer_config_db_t timer_config_db[TIM_CONFIG_DB_SIZE];
extern const timer_slave_db_t timer_slave_db[TIM_SLAVE_DB_SIZE];
extern const timer_dma_db_t timer_dma_db[TIM_DMA_DB_SIZE];
extern const timer_pin_db_t timer_etr_db[TIM_ETR_DB_SIZE];

#ifdef __cplusplus
    }
#endif

#endif
