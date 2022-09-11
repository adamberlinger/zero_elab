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

const timer_pin_db_t timer_etr_db[TIM_ETR_DB_SIZE] = {};

const timer_pin_db_t timer_pin_db[TIM_PIN_DB_SIZE] = {
    /* TIM, Channel, Port, Pin, AF number */
    /* TIM2 (9) */
    TIM_CHANNEL_ENTRY(2,1,GPIOA_BASE,0,2),
    TIM_CHANNEL_ENTRY(2,2,GPIOA_BASE,1,2),
    TIM_CHANNEL_ENTRY(2,3,GPIOA_BASE,2,2),
    TIM_CHANNEL_ENTRY(2,4,GPIOA_BASE,3,2),

    TIM_CHANNEL_ENTRY(2,1,GPIOA_BASE,5,5),
    TIM_CHANNEL_ENTRY(2,1,GPIOA_BASE,15,5),

    TIM_CHANNEL_ENTRY(2,2,GPIOB_BASE,3,2),
    TIM_CHANNEL_ENTRY(2,3,GPIOB_BASE,10,2),
    TIM_CHANNEL_ENTRY(2,4,GPIOB_BASE,11,2),
#ifdef TIM3
    TIM_CHANNEL_ENTRY(3,1,GPIOA_BASE,6,2),
    TIM_CHANNEL_ENTRY(3,2,GPIOA_BASE,7,2),
    TIM_CHANNEL_ENTRY(3,3,GPIOB_BASE,0,2),
    TIM_CHANNEL_ENTRY(3,4,GPIOB_BASE,1,2),
#endif
    /* TIM21 (4) */
    TIM_CHANNEL_ENTRY(21,1,GPIOA_BASE,2,0),
    TIM_CHANNEL_ENTRY(21,2,GPIOA_BASE,3,0),

    TIM_CHANNEL_ENTRY(21,1,GPIOB_BASE,13,6),
    TIM_CHANNEL_ENTRY(21,2,GPIOB_BASE,14,6),

    /* TIM22 (4+2) */
    TIM_CHANNEL_ENTRY(22,1,GPIOA_BASE,6,5),
    TIM_CHANNEL_ENTRY(22,2,GPIOA_BASE,7,5),

    TIM_CHANNEL_ENTRY(22,1,GPIOB_BASE,4,4),
    TIM_CHANNEL_ENTRY(22,2,GPIOB_BASE,5,4),
#ifdef GPIOC
    TIM_CHANNEL_ENTRY(22,1,GPIOC_BASE,6,0),
    TIM_CHANNEL_ENTRY(22,2,GPIOC_BASE,7,0),
#endif
};

const timer_config_db_t timer_config_db[TIM_CONFIG_DB_SIZE] = {
    {2, TIM2, TIMER_TYPE_GENERAL, 4, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM2EN},
#ifdef TIM3
    {3, TIM3, TIMER_TYPE_GENERAL, 4, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM3EN},
#endif
#ifdef TIM6
    {6, TIM6, TIMER_TYPE_BASIC, 0, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM6EN},
#endif
#ifdef TIM7
    {7, TIM7, TIMER_TYPE_BASIC, 0, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM7EN},
#endif
    {21, TIM21, TIMER_TYPE_GENERAL, 2, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM21EN},
    {22, TIM22, TIMER_TYPE_GENERAL, 2, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM22EN},
};

const timer_slave_db_t timer_slave_db[TIM_SLAVE_DB_SIZE] = {
    /* Master - Slave - ITR - reverse_index */
    {21, 2, 0, 2},
    {22, 2 ,1, 5},
    {2, 21, 0, 0},
    {22, 21, 1, 4},
    {21, 22, 0, 3},
    {2, 22, 1, 1},

#ifdef TIM3
    {3, 2, 2, 7},
    {2, 3, 0, 6},
    {22, 3, 1, 0},
    {21, 3, 2, 0},
#endif
};

const timer_dma_db_t timer_dma_db[TIM_DMA_DB_SIZE] = {
    {2, DMA1_Channel2, 8},
#ifdef TIM3
    {3, DMA1_Channel3, 10},
#endif
    {6, DMA1_Channel2, 9},
};
