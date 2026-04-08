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
    TIM_CHANNEL_ENTRY(2,1,GPIOA_BASE,5,1),
};

const timer_config_db_t timer_config_db[TIM_CONFIG_DB_SIZE] = {
    {1, TIM1, TIMER_TYPE_ADVANCED | TIMER_TYPE_HAS_COMPLEMENTARY, 4, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM1EN},
    {2, TIM2, TIMER_TYPE_GENERAL, 4, &RCC->APB1LENR, &RCC->APB1LRSTR, RCC_APB1LENR_TIM2EN},
    {5, TIM5, TIMER_TYPE_GENERAL, 4, &RCC->APB1LENR, &RCC->APB1LRSTR, RCC_APB1LENR_TIM5EN},
    {6, TIM6, TIMER_TYPE_BASIC, 0, &RCC->APB1LENR, &RCC->APB1LRSTR, RCC_APB1LENR_TIM6EN},
    {7, TIM7, TIMER_TYPE_BASIC, 0, &RCC->APB1LENR, &RCC->APB1LRSTR, RCC_APB1LENR_TIM7EN},
    {8, TIM8, TIMER_TYPE_ADVANCED | TIMER_TYPE_HAS_COMPLEMENTARY, 4, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM8EN},

    {12, TIM12, TIMER_TYPE_GENERAL, 2, &RCC->APB1LENR, &RCC->APB1LRSTR, RCC_APB1LENR_TIM12EN},
    {15, TIM15, TIMER_TYPE_GENERAL | TIMER_TYPE_HAS_COMPLEMENTARY, 2, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM15EN},
    {16, TIM16, TIMER_TYPE_GENERAL | TIMER_TYPE_HAS_COMPLEMENTARY, 1, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM16EN},
    {17, TIM17, TIMER_TYPE_GENERAL | TIMER_TYPE_HAS_COMPLEMENTARY, 1, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM17EN},
};

const timer_slave_db_t timer_slave_db[TIM_SLAVE_DB_SIZE] = {
};

const timer_dma_db_t timer_dma_db[TIM_DMA_DB_SIZE] = {
    {1,28},
    {2,35},
    {5,41},
    {6,2},
    {7,3},
    {8,70},
    {15,45},
    {16,49},
    {17,51},
};

const timer_pin_db_t timer_etr_db[TIM_ETR_DB_SIZE] = {
    /* TODO: fill the DB */
};
