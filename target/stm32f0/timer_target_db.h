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
#include "stm32_common.h"
#include "stm32_dma.h"
#include "timer.h"

#ifdef __cplusplus
    extern "C" {
#endif

#define TIM1_PIN_SIZE          4
#define TIM2_PIN_SIZE          6
#define TIM3_PIN_SIZE          4

#define TIM14_PIN_SIZE         0
#define TIM15_PIN_SIZE         0
#define TIM16_PIN_SIZE         0
#define TIM17_PIN_SIZE         0

#ifdef TIM6
    #define TIM6_CONFIG_SIZE   1
#else
    #define TIM6_CONFIG_SIZE   0
#endif

#ifdef TIM7
    #define TIM7_CONFIG_SIZE   1
#else
    #define TIM7_CONFIG_SIZE   0
#endif

#ifdef TIM15
    #define TIM15_CONFIG_SIZE   1
#else
    #define TIM15_CONFIG_SIZE   0
#endif


#define TIM_PIN_DB_SIZE    (TIM1_PIN_SIZE + TIM2_PIN_SIZE + TIM3_PIN_SIZE + \
        TIM14_PIN_SIZE + TIM15_PIN_SIZE + TIM16_PIN_SIZE + \
        TIM17_PIN_SIZE)

#define TIM_CONFIG_DB_SIZE (7 + TIM6_CONFIG_SIZE + TIM7_CONFIG_SIZE + TIM15_CONFIG_SIZE)
#define TIM_DMA_DB_SIZE (7)

#define TIM_SLAVE_DB_SIZE (10 + 4 * TIM15_CONFIG_SIZE)

#define TIM_ETR_DB_SIZE     (0)

extern const timer_pin_db_t timer_pin_db[TIM_PIN_DB_SIZE];
extern const timer_config_db_t timer_config_db[TIM_CONFIG_DB_SIZE];
extern const timer_slave_db_t timer_slave_db[TIM_SLAVE_DB_SIZE];
extern const timer_dma_db_t timer_dma_db[TIM_DMA_DB_SIZE];
extern const timer_pin_db_t timer_etr_db[TIM_ETR_DB_SIZE];

#ifdef __cplusplus
    }
#endif

#endif
