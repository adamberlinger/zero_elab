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
    /* TIM1 without complementary channels (12) */
    TIM_CHANNEL_ENTRY(1,1,GPIOA_BASE,8,2),
    TIM_CHANNEL_ENTRY(1,2,GPIOA_BASE,9,2),
    TIM_CHANNEL_ENTRY(1,3,GPIOA_BASE,10,2),
    TIM_CHANNEL_ENTRY(1,4,GPIOA_BASE,11,2),

    /* TIM2 */
    TIM_CHANNEL_ENTRY(2,1,GPIOA_BASE,0,2),
    TIM_CHANNEL_ENTRY(2,2,GPIOA_BASE,1,2),
    TIM_CHANNEL_ENTRY(2,3,GPIOA_BASE,2,2),
    TIM_CHANNEL_ENTRY(2,4,GPIOA_BASE,3,2),

    TIM_CHANNEL_ENTRY(2,1,GPIOA_BASE,15,2),
    TIM_CHANNEL_ENTRY(2,2,GPIOB_BASE,3,2),

    /* TIM3 */
    TIM_CHANNEL_ENTRY(3,1,GPIOA_BASE,6,1),
    TIM_CHANNEL_ENTRY(3,2,GPIOA_BASE,7,1),
    TIM_CHANNEL_ENTRY(3,3,GPIOB_BASE,0,1),
    TIM_CHANNEL_ENTRY(3,4,GPIOB_BASE,1,1),
};

const timer_config_db_t timer_config_db[TIM_CONFIG_DB_SIZE] = {
    {1, TIM1, TIMER_TYPE_ADVANCED, 4, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM1EN},
    {2, TIM2, TIMER_TYPE_GENERAL, 4, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM2EN},
    {3, TIM3, TIMER_TYPE_GENERAL, 4, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM3EN},
#ifdef TIM6
    {6, TIM6, TIMER_TYPE_BASIC, 0, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM6EN},
#endif
#ifdef TIM7
    {7, TIM7, TIMER_TYPE_BASIC, 0, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM7EN},
#endif
    {14, TIM14, TIMER_TYPE_GENERAL, 2, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM14EN},
#ifdef TIM15
    {15, TIM15, TIMER_TYPE_GENERAL, 2, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM15EN},
#endif
    {16, TIM16, TIMER_TYPE_GENERAL, 1, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM16EN},
    {17, TIM17, TIMER_TYPE_GENERAL, 1, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM17EN},
};

const timer_slave_db_t timer_slave_db[TIM_SLAVE_DB_SIZE] = {
    /* Master - Slave - ITR - reverse_index */
    {1, 2, 0, 2},
    {1, 3, 0, 5},

    {2, 1, 1, 0},
    {2, 3, 1, 6},
    {2, 15, 1, 11},

    /* index 5 */
    {3, 1, 2, 1},
    {3, 2, 2, 3},
    {3, 15, 1, 12},

    {14, 2, 3, 0},
    {14, 3, 3, 0},

#ifdef TIM15
    /* index 10 */
    {15, 1, 0, 0},
    {15, 2, 1, 4},
    {15, 3, 2, 7},

    {17, 1, 3, 0},
#endif
};

const timer_dma_db_t timer_dma_db[TIM_DMA_DB_SIZE] = {
    {1, DMA1_Channel5},
    {2, DMA1_Channel2},
    {3, DMA1_Channel3},
    {6, DMA1_Channel3},

    {15, DMA1_Channel5},
    {16, DMA1_Channel3},
    {17, DMA1_Channel1},
};

const timer_pin_db_t timer_etr_db[TIM_ETR_DB_SIZE] = {
    /* TODO: fill the DB */
};
