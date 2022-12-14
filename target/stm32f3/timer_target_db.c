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

const timer_pin_db_t timer_etr_db[TIM_ETR_DB_SIZE] = {
    /* ETR signals CH0=ETR */
    /* TIM, Channel, Port, Pin, AF number */
    TIM_CHANNEL_ENTRY(2,0,GPIOA_BASE,0,1),
    TIM_CHANNEL_ENTRY(8,0,GPIOA_BASE,0,10),
    TIM_CHANNEL_ENTRY(2,0,GPIOA_BASE,5,1),
    TIM_CHANNEL_ENTRY(4,0,GPIOA_BASE,8,10),

    TIM_CHANNEL_ENTRY(1,0,GPIOA_BASE,12,11),
    TIM_CHANNEL_ENTRY(2,0,GPIOA_BASE,15,1),
    TIM_CHANNEL_ENTRY(2,0,GPIOB_BASE,3,1),
    TIM_CHANNEL_ENTRY(3,0,GPIOB_BASE,3,10),

    TIM_CHANNEL_ENTRY(8,0,GPIOB_BASE,6,6),
    TIM_CHANNEL_ENTRY(1,0,GPIOC_BASE,4,2),
};

const timer_pin_db_t timer_pin_db[TIM_PIN_DB_SIZE] = {
    /* TIM, Channel, Port, Pin, AF number */
    /* TIM1 without complementary channels (12) */
    TIM_CHANNEL_ENTRY(1,1,GPIOA_BASE,8,6),
    TIM_CHANNEL_ENTRY(1,2,GPIOA_BASE,9,6),
    TIM_CHANNEL_ENTRY(1,3,GPIOA_BASE,10,6),
    TIM_CHANNEL_ENTRY(1,4,GPIOA_BASE,11,10),

    TIM_CHANNEL_ENTRY(1,1,GPIOC_BASE,0,2),
    TIM_CHANNEL_ENTRY(1,2,GPIOC_BASE,1,2),
    TIM_CHANNEL_ENTRY(1,3,GPIOC_BASE,2,2),
    TIM_CHANNEL_ENTRY(1,4,GPIOC_BASE,3,2),

#ifdef GPIOE
    TIM_CHANNEL_ENTRY(1,1,GPIOE_BASE,9,2),
    TIM_CHANNEL_ENTRY(1,2,GPIOE_BASE,11,2),
    TIM_CHANNEL_ENTRY(1,3,GPIOE_BASE,13,2),
    TIM_CHANNEL_ENTRY(1,4,GPIOE_BASE,14,2),
#endif

    /* TIM2 (15) */
    TIM_CHANNEL_ENTRY(2,1,GPIOA_BASE,5,1),
    TIM_CHANNEL_ENTRY(2,2,GPIOA_BASE,1,1),
    TIM_CHANNEL_ENTRY(2,3,GPIOA_BASE,2,1),
    TIM_CHANNEL_ENTRY(2,4,GPIOA_BASE,3,1),

    TIM_CHANNEL_ENTRY(2,1,GPIOA_BASE,0,1),
    TIM_CHANNEL_ENTRY(2,2,GPIOB_BASE,3,1),
    TIM_CHANNEL_ENTRY(2,3,GPIOA_BASE,9,10),
    TIM_CHANNEL_ENTRY(2,4,GPIOA_BASE,10,10),

    TIM_CHANNEL_ENTRY(2,1,GPIOA_BASE,15,1),
    TIM_CHANNEL_ENTRY(2,3,GPIOB_BASE,10,1),
    TIM_CHANNEL_ENTRY(2,4,GPIOB_BASE,11,1),

    TIM_CHANNEL_ENTRY(2,1,GPIOD_BASE,3,2),
    TIM_CHANNEL_ENTRY(2,2,GPIOD_BASE,4,2),
    TIM_CHANNEL_ENTRY(2,3,GPIOD_BASE,6,2),
    TIM_CHANNEL_ENTRY(2,4,GPIOD_BASE,7,2),
    /* TIM3 (16) */
    TIM_CHANNEL_ENTRY(3,1,GPIOA_BASE,6,2),
    TIM_CHANNEL_ENTRY(3,2,GPIOA_BASE,7,2),
    TIM_CHANNEL_ENTRY(3,3,GPIOB_BASE,0,2),
    TIM_CHANNEL_ENTRY(3,4,GPIOB_BASE,1,2),

    TIM_CHANNEL_ENTRY(3,1,GPIOB_BASE,4,2),
    TIM_CHANNEL_ENTRY(3,2,GPIOB_BASE,5,2),
    TIM_CHANNEL_ENTRY(3,2,GPIOA_BASE,4,2),
    TIM_CHANNEL_ENTRY(3,4,GPIOB_BASE,7,10),

    TIM_CHANNEL_ENTRY(3,1,GPIOC_BASE,6,2),
    TIM_CHANNEL_ENTRY(3,2,GPIOC_BASE,7,2),
    TIM_CHANNEL_ENTRY(3,3,GPIOC_BASE,8,2),
    TIM_CHANNEL_ENTRY(3,4,GPIOC_BASE,9,2),
#ifdef GPIOE
    TIM_CHANNEL_ENTRY(3,1,GPIOE_BASE,2,2),
    TIM_CHANNEL_ENTRY(3,2,GPIOE_BASE,3,2),
    TIM_CHANNEL_ENTRY(3,3,GPIOE_BASE,4,2),
    TIM_CHANNEL_ENTRY(3,4,GPIOE_BASE,5,2),
#endif

    /* TIM4 (12) */
#ifdef TIM4
    TIM_CHANNEL_ENTRY(4,1,GPIOA_BASE,11,10),
    TIM_CHANNEL_ENTRY(4,2,GPIOA_BASE,12,10),
    TIM_CHANNEL_ENTRY(4,3,GPIOA_BASE,13,10),

    TIM_CHANNEL_ENTRY(4,1,GPIOB_BASE,6,2),
    TIM_CHANNEL_ENTRY(4,2,GPIOB_BASE,7,2),
    TIM_CHANNEL_ENTRY(4,3,GPIOB_BASE,8,2),
    TIM_CHANNEL_ENTRY(4,4,GPIOB_BASE,9,2),

    TIM_CHANNEL_ENTRY(4,1,GPIOD_BASE,12,2),
    TIM_CHANNEL_ENTRY(4,2,GPIOD_BASE,13,2),
    TIM_CHANNEL_ENTRY(4,3,GPIOD_BASE,14,2),
    TIM_CHANNEL_ENTRY(4,4,GPIOD_BASE,15,2),

    TIM_CHANNEL_ENTRY(4,4,GPIOF_BASE,6,2),
#endif

    /* TIM8 (10) */
#ifdef TIM8
    TIM_CHANNEL_ENTRY(8,1,GPIOA_BASE,15,2),
    TIM_CHANNEL_ENTRY(8,2,GPIOA_BASE,14,5),

    TIM_CHANNEL_ENTRY(8,1,GPIOB_BASE,6,5),
    TIM_CHANNEL_ENTRY(8,2,GPIOB_BASE,8,10),
    TIM_CHANNEL_ENTRY(8,3,GPIOB_BASE,9,10),

    TIM_CHANNEL_ENTRY(8,1,GPIOC_BASE,6,4),
    TIM_CHANNEL_ENTRY(8,2,GPIOC_BASE,7,4),
    TIM_CHANNEL_ENTRY(8,3,GPIOC_BASE,8,4),
    TIM_CHANNEL_ENTRY(8,4,GPIOC_BASE,9,4),

    TIM_CHANNEL_ENTRY(8,4,GPIOD_BASE,1,4),
#endif

    /* TIM15 (4) */
    TIM_CHANNEL_ENTRY(15,1,GPIOA_BASE,2,9),
    TIM_CHANNEL_ENTRY(15,2,GPIOA_BASE,3,9),

    TIM_CHANNEL_ENTRY(15,1,GPIOB_BASE,14,1),
    TIM_CHANNEL_ENTRY(15,2,GPIOB_BASE,15,1),

    /* TIM16 (5) */
    TIM_CHANNEL_ENTRY(16,1,GPIOA_BASE,6,1),
    TIM_CHANNEL_ENTRY(16,1,GPIOA_BASE,12,1),

    TIM_CHANNEL_ENTRY(16,1,GPIOB_BASE,4,1),
    TIM_CHANNEL_ENTRY(16,1,GPIOB_BASE,8,1),
#ifdef GPIOE
    TIM_CHANNEL_ENTRY(16,1,GPIOE_BASE,0,4),
#endif

    /* TIM17 (4) */
    TIM_CHANNEL_ENTRY(17,1,GPIOA_BASE,7,1),
    TIM_CHANNEL_ENTRY(17,1,GPIOB_BASE,5,10),
    TIM_CHANNEL_ENTRY(17,1,GPIOB_BASE,9,1),
#ifdef GPIOE
    TIM_CHANNEL_ENTRY(17,1,GPIOE_BASE,1,4),
#endif

};

#ifdef STM32F303x8
    #define TIMx_PRESCALER  (1)
#else
    #define TIMx_PRESCALER  (-2)
#endif

const timer_config_db_t timer_config_db[TIM_CONFIG_DB_SIZE] = {
    {1, TIM1, TIMER_TYPE_ADVANCED, 6, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM1EN,-2},
    {2, TIM2, TIMER_TYPE_GENERAL, 4, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM2EN,TIMx_PRESCALER},
    {3, TIM3, TIMER_TYPE_GENERAL, 4, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM3EN,TIMx_PRESCALER},
#ifdef TIM4
    {4, TIM4, TIMER_TYPE_GENERAL, 4, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM4EN,TIMx_PRESCALER},
#endif
    {6, TIM6, TIMER_TYPE_BASIC, 0, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM6EN,1},
    {7, TIM7, TIMER_TYPE_BASIC, 0, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM7EN,1},
#ifdef TIM8
    {8, TIM8, TIMER_TYPE_ADVANCED, 6, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM8EN,-2},
#endif
    {15, TIM15, TIMER_TYPE_GENERAL, 2, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM15EN,TIMx_PRESCALER},
    {16, TIM16, TIMER_TYPE_GENERAL | TIMER_TYPE_HAS_COMPLEMENTARY, 1, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM16EN,TIMx_PRESCALER},
    {17, TIM17, TIMER_TYPE_GENERAL, 1, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM17EN,TIMx_PRESCALER},
#ifdef TIM20
    {20, TIM20, TIMER_TYPE_ADVANCED, 6, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM20EN,TIMx_PRESCALER},
#endif
};

const timer_slave_db_t timer_slave_db[TIM_SLAVE_DB_SIZE] = {
    /* Master - Slave - ITR - reverse_index */
    {1, 8, 0, 0},       /* index 0 */
    {1, 20, 0, 0},
    {1, 2, 0, 8},
    {1, 3, 0, 13},
    {1, 4, 0, 18},

    {8, 20, 1, 0},         /* index 5 */
    {8, 2, 1, 9},
    {8, 4, 3, 19},

    {2, 1, 1, 2},          /* index 8 */
    {2, 8, 1, 6},
    {2, 3, 1, 15},
    {2, 4, 1, 21},
    {2, 15, 0, 0},

    {3, 1, 2, 3},          /* index 13 */
    {3, 8, 3, 0},
    {3, 2, 2, 10},
    {3, 4, 2, 22},
    {3, 15, 1, 25},

    {4, 1, 3, 4},          /* index 18 */
    {4, 8, 2, 7},
    {4, 20, 2, 0},
    {4, 2, 3, 11},
    {4, 3, 3, 16},

    {15, 1, 0, 0},          /* index 23 */
    {15, 20, 3, 0},
    {15, 3, 2, 17},

    {16, 15, 2, 0},
    {17, 15, 3, 0},
};

const timer_dma_db_t timer_dma_db[TIM_DMA_DB_SIZE] = {
    {1, DMA1_Channel5},
    {2, DMA1_Channel2},
    {3, DMA1_Channel3},
#ifdef TIM4
    {4, DMA1_Channel7},
#endif
    {6, DMA1_Channel3},
    {7, DMA1_Channel4},
    {15, DMA1_Channel5},
    {16, DMA1_Channel3},
    {17, DMA1_Channel1},
};
