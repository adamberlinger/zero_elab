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
    ADC_PIN_ENTRY(1, 1, GPIOA_BASE, 0),
    ADC_PIN_ENTRY(1, 2, GPIOA_BASE, 1),
    ADC_PIN_ENTRY(1, 3, GPIOA_BASE, 2),
    ADC_PIN_ENTRY(1, 4, GPIOA_BASE, 3),
    ADC_PIN_ENTRY(1, 5, GPIOF_BASE, 4),
    ADC_PIN_ENTRY(1, 6, GPIOC_BASE, 0),
    ADC_PIN_ENTRY(1, 7, GPIOC_BASE, 1),
    ADC_PIN_ENTRY(1, 8, GPIOC_BASE, 2),
    ADC_PIN_ENTRY(1, 9, GPIOC_BASE, 3),
    ADC_PIN_ENTRY(1, 10, GPIOF_BASE, 2),
#ifndef ADC3
    ADC_PIN_ENTRY(1, 11, GPIOB_BASE, 0),
    ADC_PIN_ENTRY(1, 12, GPIOB_BASE, 1),
    ADC_PIN_ENTRY(1, 13, GPIOB_BASE, 13),
#endif
    /* ADC2 pin definitions (14/17) */
    ADC_PIN_ENTRY(2, 1, GPIOA_BASE, 4),
    ADC_PIN_ENTRY(2, 2, GPIOA_BASE, 5),
    ADC_PIN_ENTRY(2, 3, GPIOA_BASE, 6),
    ADC_PIN_ENTRY(2, 4, GPIOA_BASE, 7),
    ADC_PIN_ENTRY(2, 5, GPIOC_BASE, 4),
    ADC_PIN_ENTRY(2, 6, GPIOC_BASE, 0),
    ADC_PIN_ENTRY(2, 7, GPIOC_BASE, 1),
    ADC_PIN_ENTRY(2, 8, GPIOC_BASE, 2),
    ADC_PIN_ENTRY(2, 9, GPIOC_BASE, 3), /* Unused channel 10 */
    ADC_PIN_ENTRY(2, 11, GPIOC_BASE, 5),
    ADC_PIN_ENTRY(2, 12, GPIOB_BASE, 2),
#ifndef ADC3
    ADC_PIN_ENTRY(2, 13, GPIOB_BASE, 12), /* ONLY for F303x8 */
    ADC_PIN_ENTRY(2, 14, GPIOB_BASE, 14), /* ONLY for F303x8 */
    ADC_PIN_ENTRY(2, 15, GPIOB_BASE, 15), /* ONLY for F303x8 */
#endif
#ifdef ADC3
    /* ADC3 pin definitions (15) */
    ADC_PIN_ENTRY(3, 1, GPIOB_BASE, 1),
    ADC_PIN_ENTRY(3, 2, GPIOE_BASE, 9),
    ADC_PIN_ENTRY(3, 3, GPIOE_BASE, 13), /* Unused channel 4 */
    ADC_PIN_ENTRY(3, 5, GPIOB_BASE, 13),
    ADC_PIN_ENTRY(3, 6, GPIOE_BASE, 8),
    ADC_PIN_ENTRY(3, 7, GPIOD_BASE, 10),
    ADC_PIN_ENTRY(3, 8, GPIOD_BASE, 11),
    ADC_PIN_ENTRY(3, 9, GPIOD_BASE, 12),
    ADC_PIN_ENTRY(3, 10, GPIOD_BASE, 13),
    ADC_PIN_ENTRY(3, 11, GPIOD_BASE, 14),
    ADC_PIN_ENTRY(3, 12, GPIOB_BASE, 0),
    ADC_PIN_ENTRY(3, 13, GPIOE_BASE, 7),
    ADC_PIN_ENTRY(3, 14, GPIOE_BASE, 10),
    ADC_PIN_ENTRY(3, 15, GPIOE_BASE, 11),
    ADC_PIN_ENTRY(3, 16, GPIOE_BASE, 12),
    /* ADC4 pin definitions (13) */
    ADC_PIN_ENTRY(4, 1, GPIOE_BASE, 14),
    ADC_PIN_ENTRY(4, 2, GPIOE_BASE, 15),
    ADC_PIN_ENTRY(4, 3, GPIOB_BASE, 12),
    ADC_PIN_ENTRY(4, 4, GPIOB_BASE, 14),
    ADC_PIN_ENTRY(4, 5, GPIOB_BASE, 15),
    ADC_PIN_ENTRY(4, 6, GPIOE_BASE, 8),
    ADC_PIN_ENTRY(4, 7, GPIOD_BASE, 10),
    ADC_PIN_ENTRY(4, 8, GPIOD_BASE, 11),
    ADC_PIN_ENTRY(4, 9, GPIOD_BASE, 12),
    ADC_PIN_ENTRY(4, 10, GPIOD_BASE, 13),
    ADC_PIN_ENTRY(4, 11, GPIOD_BASE, 14),
    ADC_PIN_ENTRY(4, 12, GPIOD_BASE, 8),
    ADC_PIN_ENTRY(4, 13, GPIOD_BASE, 9),
#endif
};

const adc_dma_db_t adc_dma_db[ADC_DMA_DB_SIZE] = {
    {1, DMA1_Channel1},
#ifndef DMA2
    {2, DMA1_Channel4},
#else
    {2, DMA2_Channel1},
#endif
#ifdef DMA2
    {3, DMA2_Channel5},
    {4, DMA2_Channel2},
#endif
};
const adc_tim_db_t adc_tim_db[ADC_TIM_DB_SIZE] = {
    /* ADC1 & 2 Timers (6*2) */
    {1, 15, 14},
    {1, 6, 13},
    {1, 2, 11},
    {1, 4, 12},
    {1, 3, 4},
    {1, 1, 9},

    {2, 15, 14},
    {2, 6, 13},
    {2, 2, 11},
    {2, 4, 12},
    {2, 3, 4},
    {2, 1, 9},
#ifdef ADC3
    /* ADC3 & 4 Timers (6*2) */
    {3, 7, 13},
    {3, 3, 11},
    {3, 15, 14},
    {3, 4, 12},
    {3, 2, 7},
    {3, 1, 9},

    {4, 7, 13},
    {4, 15, 14},
    {4, 3, 11},
    {4, 4, 12},
    {4, 2, 7},
    {4, 1, 9},
#endif
};
