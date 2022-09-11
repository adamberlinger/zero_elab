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
#include "adc_target_db.h"

#define ADC_PIN_ENTRY(adc_id,adc_channel,port,pin) {DEFINE_ADC_CHANNEL(adc_id,adc_channel),DEFINE_PIN(port,pin)}

/* TODO: check ADC1&2 for xE vs x8 */
const adc_pin_db_t adc_pin_db[ADC_PIN_DB_SIZE] = {
    /* ADC1 pin definitions (10/13) */
    ADC_PIN_ENTRY(1, 0, GPIOA_BASE, 0),
    ADC_PIN_ENTRY(1, 1, GPIOA_BASE, 1),
    ADC_PIN_ENTRY(1, 2, GPIOA_BASE, 2),
    ADC_PIN_ENTRY(1, 3, GPIOA_BASE, 3),
    ADC_PIN_ENTRY(1, 4, GPIOA_BASE, 4),
    ADC_PIN_ENTRY(1, 5, GPIOA_BASE, 5),
    ADC_PIN_ENTRY(1, 6, GPIOA_BASE, 6),
    ADC_PIN_ENTRY(1, 7, GPIOA_BASE, 7),
    ADC_PIN_ENTRY(1, 8, GPIOB_BASE, 0),
    ADC_PIN_ENTRY(1, 9, GPIOB_BASE, 1),

    ADC_PIN_ENTRY(1, 10, GPIOC_BASE, 0),
    ADC_PIN_ENTRY(1, 11, GPIOC_BASE, 1),
    ADC_PIN_ENTRY(1, 12, GPIOC_BASE, 2),
    ADC_PIN_ENTRY(1, 13, GPIOC_BASE, 3),
    ADC_PIN_ENTRY(1, 14, GPIOC_BASE, 4),
    ADC_PIN_ENTRY(1, 15, GPIOC_BASE, 5),
};

const adc_dma_db_t adc_dma_db[ADC_DMA_DB_SIZE] = {
    {1, DMA1_Channel1},
};
const adc_tim_db_t adc_tim_db[ADC_TIM_DB_SIZE] = {
    /* ADC1 & 2 Timers (6*2) */
    {1, 3, 4},
};
