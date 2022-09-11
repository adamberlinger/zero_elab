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
#ifndef _DAC_TARGET_DB_H_
#define _DAC_TARGET_DB_H_

#include <stdint.h>
#include "stm32_common.h"
#include "stm32_dma.h"

#ifdef __cplusplus
    extern "C" {
#endif

#ifdef STM32F303x8
    #define DAC_PIN_DB_SIZE 3
    #define DAC_DMA_DB_SIZE 3
    #define DAC_TIM_DB_SIZE 9
#else
    #define DAC_PIN_DB_SIZE 2
    #define DAC_DMA_DB_SIZE 2
    #define DAC_TIM_DB_SIZE 5
#endif

#define DEFINE_DAC_CHANNEL(dac,channel) (uint8_t)(((dac & 0xF) << 4) | (channel & 0xF))

typedef struct {
    uint8_t dac_channel;
    gpio_pin_t pin;
}dac_pin_db_t;

typedef struct {
    uint8_t dac_channel;
    dma_handle_t dma;
}dac_dma_db_t;

typedef struct {
    uint8_t dac_id;
    uint8_t timer_id;
    uint8_t trigger_source;
}dac_tim_db_t;

extern const dac_pin_db_t dac_pin_db[DAC_PIN_DB_SIZE];
extern const dac_dma_db_t dac_dma_db[DAC_DMA_DB_SIZE];
extern const dac_tim_db_t dac_tim_db[DAC_TIM_DB_SIZE];

#ifdef __cplusplus
    }
#endif

#endif
