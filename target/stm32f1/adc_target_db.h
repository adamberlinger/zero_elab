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
#ifndef _ADC_TARGET_DB_H_
#define _ADC_TARGET_DB_H_

#include <stdint.h>
#include "stm32_common.h"
#include "stm32_dma.h"

#ifdef __cplusplus
    extern "C" {
#endif

#define ADC_PIN_DB_SIZE 16
#define ADC_TIM_DB_SIZE 1
#define ADC_DMA_DB_SIZE 1

#define DEFINE_ADC_CHANNEL(adc,channel) (uint16_t)(((adc & 0xFF) << 8) | (channel & 0xFF))
#define EXPORT_ADC_ID(adc_channel)      (int)((adc_channel >> 8) & 0xFF)
#define EXPORT_ADC_CHANNEL(adc_channel)      (int)(adc_channel & 0xFF)

typedef struct {
    uint16_t adc_channel;
    gpio_pin_t pin;
}adc_pin_db_t;

typedef struct {
    uint8_t adc_id;
    dma_handle_t dma;
}adc_dma_db_t;

typedef struct {
    uint8_t adc_id;
    uint8_t timer_id;
    uint8_t trigger_source;
}adc_tim_db_t;

extern const adc_pin_db_t adc_pin_db[ADC_PIN_DB_SIZE];
extern const adc_dma_db_t adc_dma_db[ADC_DMA_DB_SIZE];
extern const adc_tim_db_t adc_tim_db[ADC_TIM_DB_SIZE];

#ifdef __cplusplus
    }
#endif

#endif
