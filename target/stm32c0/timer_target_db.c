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
#include "timer_target_db.h"

#define TIM_CHANNEL_ENTRY(tim,channel,port,pin,af) {DEFINE_TIM_CHANNEL(tim, channel), DEFINE_PIN(port,pin), af}


const timer_pin_db_t timer_pin_db[TIM_PIN_DB_SIZE] = {
    /* TIM, Channel, Port, Pin, AF number */
    TIM_CHANNEL_ENTRY(14,1,GPIOA_BASE,4,4),
    TIM_CHANNEL_ENTRY(14,1,GPIOA_BASE,7,4),
    TIM_CHANNEL_ENTRY(16,1,GPIOB_BASE,8,2),
    TIM_CHANNEL_ENTRY(16,1,GPIOB_BASE,6,2), /* Complementary */
    TIM_CHANNEL_ENTRY(16,1,GPIOA_BASE,2,2),
    TIM_CHANNEL_ENTRY(16,1,GPIOA_BASE,6,5),

    TIM_CHANNEL_ENTRY(17,1,GPIOB_BASE,9,2),
    TIM_CHANNEL_ENTRY(17,1,GPIOA_BASE,7,5),
    TIM_CHANNEL_ENTRY(17,1,GPIOB_BASE,7,2), /* Complementary */
    TIM_CHANNEL_ENTRY(17,1,GPIOA_BASE,4,5), /* Complementary */


    /* TIM1 without complementary channels (12) */
    TIM_CHANNEL_ENTRY(1,1,GPIOA_BASE,8,2),
    TIM_CHANNEL_ENTRY(1,2,GPIOA_BASE,9,2),
    TIM_CHANNEL_ENTRY(1,3,GPIOA_BASE,10,2),
    TIM_CHANNEL_ENTRY(1,4,GPIOA_BASE,11,2),

    /* TIM3 */
    TIM_CHANNEL_ENTRY(3,1,GPIOA_BASE,6,1),
    TIM_CHANNEL_ENTRY(3,2,GPIOA_BASE,7,1),
    TIM_CHANNEL_ENTRY(3,3,GPIOB_BASE,0,1),
    TIM_CHANNEL_ENTRY(3,4,GPIOB_BASE,1,1),

#ifdef STM32C071xx
    /* TIM2 */
    TIM_CHANNEL_ENTRY(2,1,GPIOA_BASE, 5, 3),
#endif
};

const timer_config_db_t timer_config_db[TIM_CONFIG_DB_SIZE] = {
    {1, TIM1, TIMER_TYPE_ADVANCED, 4, &RCC->APBENR2, &RCC->APBRSTR2, RCC_APBENR2_TIM1EN},
    {3, TIM3, TIMER_TYPE_GENERAL, 4, &RCC->APBENR1, &RCC->APBRSTR1, RCC_APBENR1_TIM3EN},

    {14, TIM14, TIMER_TYPE_GENERAL, 1, &RCC->APBENR2, &RCC->APBRSTR2, RCC_APBENR2_TIM14EN},

    {16, TIM16, TIMER_TYPE_GENERAL | TIMER_TYPE_HAS_COMPLEMENTARY, 1, &RCC->APBENR2, &RCC->APBRSTR2, RCC_APBENR2_TIM16EN},
    {17, TIM17, TIMER_TYPE_GENERAL | TIMER_TYPE_HAS_COMPLEMENTARY, 1, &RCC->APBENR2, &RCC->APBRSTR2, RCC_APBENR2_TIM17EN},
#ifdef STM32C071xx
    {2, TIM2, TIMER_TYPE_GENERAL, 4, &RCC->APBENR1, &RCC->APBRSTR1, RCC_APBENR1_TIM2EN},
#endif
};

const timer_slave_db_t timer_slave_db[TIM_SLAVE_DB_SIZE] = {
    /* Master - Slave - ITR - reverse_index */
    {1, 3, 0, 1},
    {3, 1, 2, 0},

    /* Other connections are not trough TRGO */
};

const timer_dma_db_t timer_dma_db[TIM_DMA_DB_SIZE] = {
    {1, 25},
#ifdef STM32C071xx
    {2, 31},
#endif
    {3, 37},

    {16, 46},
    {17, 49},
};

const timer_pin_db_t timer_etr_db[TIM_ETR_DB_SIZE] = {
    /* TODO: fill the DB */
};
