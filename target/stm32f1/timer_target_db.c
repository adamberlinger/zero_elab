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
    TIM_CHANNEL_ENTRY(1,1,GPIOA_BASE,8,0),
    TIM_CHANNEL_ENTRY(1,2,GPIOA_BASE,9,0),
    TIM_CHANNEL_ENTRY(1,3,GPIOA_BASE,10,0),
    TIM_CHANNEL_ENTRY(1,4,GPIOA_BASE,11,0),

    /* TIM2 */
    TIM_CHANNEL_ENTRY(2,1,GPIOA_BASE,0,0),
    TIM_CHANNEL_ENTRY(2,2,GPIOA_BASE,1,0),
    TIM_CHANNEL_ENTRY(2,3,GPIOA_BASE,2,0),
    TIM_CHANNEL_ENTRY(2,4,GPIOA_BASE,3,0),

    /* TIM3 */
    TIM_CHANNEL_ENTRY(3,1,GPIOA_BASE,6,0),
    TIM_CHANNEL_ENTRY(3,2,GPIOA_BASE,7,0),
    TIM_CHANNEL_ENTRY(3,3,GPIOB_BASE,0,0),
    TIM_CHANNEL_ENTRY(3,4,GPIOB_BASE,1,0),

    /* TIM4 */
    TIM_CHANNEL_ENTRY(4,1,GPIOB_BASE,6,0),
    TIM_CHANNEL_ENTRY(4,2,GPIOB_BASE,7,0),
    TIM_CHANNEL_ENTRY(4,3,GPIOB_BASE,8,0),
    TIM_CHANNEL_ENTRY(4,4,GPIOB_BASE,9,0),
};

const timer_config_db_t timer_config_db[TIM_CONFIG_DB_SIZE] = {
    {1, TIM1, TIMER_TYPE_ADVANCED, 4, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM1EN},
    {2, TIM2, TIMER_TYPE_GENERAL, 4, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM2EN},
    {3, TIM3, TIMER_TYPE_GENERAL, 4, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM3EN},
    {4, TIM4, TIMER_TYPE_GENERAL, 4, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM4EN},
#ifdef TIM6
    {6, TIM6, TIMER_TYPE_BASIC, 0, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM6EN},
#endif
#ifdef TIM7
    {7, TIM7, TIMER_TYPE_BASIC, 0, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM7EN},
#endif
#ifdef TIM14
    {14, TIM14, TIMER_TYPE_GENERAL, 2, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM14EN},
#endif
#ifdef TIM15
    {15, TIM15, TIMER_TYPE_GENERAL, 2, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM15EN},
#endif
#ifdef TIM16
    {16, TIM16, TIMER_TYPE_GENERAL, 1, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM16EN},
#endif
#ifdef TIM17
    {17, TIM17, TIMER_TYPE_GENERAL, 1, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM17EN},
#endif
};

const timer_dma_db_t timer_dma_db[TIM_DMA_DB_SIZE] = {
    {1, DMA1_Channel5},
    {2, DMA1_Channel2},
    {3, DMA1_Channel3},
#ifdef TIM4
    {4, DMA1_Channel7},
#endif
};

const timer_slave_db_t timer_slave_db[TIM_SLAVE_DB_SIZE] = {
    /* Master - Slave - ITR - reverse_index */
    {1, 2, 0, 3},       /* index 0 */
    {1, 3, 0, 6},
    {1, 4, 0, 9},

    {2, 1, 1, 0},          /* index 3 */
    {2, 3, 1, 7},
    {2, 4, 1, 10},

    {3, 1, 2, 1},          /* index 6 */
    {3, 2, 2, 4},
    {3, 4, 2, 11},

    {4, 1, 3, 2},          /* index 9 */
    {4, 2, 3, 5},
    {4, 3, 3, 8},
};
