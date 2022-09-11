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
#include "uart_target_db.h"

const uart_config_db_t uart_config_db[UART_CONFIG_DB_SIZE] = {
    {&RCC->APB2ENR,RCC_APB2ENR_USART1EN,USART1,1,1},
    {&RCC->APB1ENR,RCC_APB1ENR_USART2EN,USART2,2,2},
#ifdef USART3
    {&RCC->APB1ENR,RCC_APB1ENR_USART3EN,USART3,3,2},
#endif
};

const uart_dma_db_t uart_dma_db[UART_DMA_DM_SIZE] = {
    {DMA1_Channel4,1,0},
    {DMA1_Channel5,1,1},
    {DMA1_Channel7,2,0},
    {DMA1_Channel6,2,1},
#ifdef USART3
    {DMA1_Channel2,3,0},
    {DMA1_Channel3,3,1},
#endif
};

const uart_pin_db_t uart_pin_db[UART_PIN_DB_SIZE] = {
    {DEFINE_PIN(GPIOA_BASE,9),1,7,0},
    {DEFINE_PIN(GPIOA_BASE,10),1,7,1},

    {DEFINE_PIN(GPIOA_BASE,2),2,7,0},
    {DEFINE_PIN(GPIOA_BASE,3),2,7,1},

    {DEFINE_PIN(GPIOA_BASE,14),2,7,0},
    {DEFINE_PIN(GPIOA_BASE,15),2,7,1},

    /* TODO: defin all pins */
};
