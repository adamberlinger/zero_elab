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

const adc_pin_db_t adc_pin_db[ADC_PIN_DB_SIZE] = {
    /* ADC1 pin definitions (10) */
    ADC_PIN_ENTRY(1, 0, GPIOA_BASE, 0),
    ADC_PIN_ENTRY(1, 1, GPIOA_BASE, 1),
    ADC_PIN_ENTRY(1, 2, GPIOA_BASE, 2),
    ADC_PIN_ENTRY(1, 3, GPIOA_BASE, 3),
    ADC_PIN_ENTRY(1, 4, GPIOA_BASE, 4),
    ADC_PIN_ENTRY(1, 5, GPIOA_BASE, 5),
    ADC_PIN_ENTRY(1, 6, GPIOA_BASE, 6),
    ADC_PIN_ENTRY(1, 7, GPIOA_BASE, 7),
    ADC_PIN_ENTRY(1, 8, GPIOA_BASE, 8),

    ADC_PIN_ENTRY(1, 11, GPIOA_BASE, 11),
    ADC_PIN_ENTRY(1, 12, GPIOA_BASE, 12),
    ADC_PIN_ENTRY(1, 13, GPIOA_BASE, 13),
    ADC_PIN_ENTRY(1, 14, GPIOA_BASE, 14),

    ADC_PIN_ENTRY(1, 17, GPIOB_BASE, 0),
    ADC_PIN_ENTRY(1, 18, GPIOB_BASE, 1),
    ADC_PIN_ENTRY(1, 19, GPIOB_BASE, 2),

    ADC_PIN_ENTRY(1, 20, GPIOB_BASE, 10), 
    ADC_PIN_ENTRY(1, 21, GPIOB_BASE, 11), 
    ADC_PIN_ENTRY(1, 22, GPIOB_BASE, 12), 
};

const adc_dma_db_t adc_dma_db[ADC_DMA_DB_SIZE] = {
    {1, 5},
};
const adc_tim_db_t adc_tim_db[ADC_TIM_DB_SIZE] = {
    /* ADC1 Timers (2) */
    {1, 1, 0},
    {1, 3, 3},
};

#define SAMPLETIME_ADC_FREQUENCY    (14000000)
#define SAMPLETIME_ADC_CAPACITANCE  (0.000000000008)  //8pF
#define SAMPLETIME_ADC_RESISTANCE   (1000)

/* Value of ln(2^14) */
#define SAMPLETIME_LN_14            (9.704060528)

#define SAMPLETIME_MAX_FREQUENCY(t) (uint32_t)((SAMPLETIME_ADC_FREQUENCY / (t + 15.5)) + 1.0)
#define SAMPLETIME_MAX_IMPEDANCE(t) (uint32_t)(((t / (SAMPLETIME_ADC_FREQUENCY * SAMPLETIME_ADC_CAPACITANCE * SAMPLETIME_LN_14)) - SAMPLETIME_ADC_RESISTANCE) - 1.0)

#define SAMPLETIME_ROW(config,t)    {config, SAMPLETIME_MAX_FREQUENCY(t), SAMPLETIME_MAX_IMPEDANCE(t)}


/* This table is valid only for 12-bit */
const adc_sampletime_db_t adc_sampletime_db[ADC_SAMPLETIME_DB_SIZE] = {
    SAMPLETIME_ROW(0,   1.5),
    SAMPLETIME_ROW(1,   3.5),
    SAMPLETIME_ROW(2,   7.5),
    SAMPLETIME_ROW(3,  12.5),
    SAMPLETIME_ROW(4,  19.5),
    SAMPLETIME_ROW(5,  39.5),
    SAMPLETIME_ROW(6,  79.5),
    SAMPLETIME_ROW(7, 160.5),
};
