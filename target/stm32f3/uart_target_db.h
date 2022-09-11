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
#ifndef _UART_TARGET_DB_H_
#define _UART_TARGET_DB_H_

#include <stdint.h>
#include "uart_target.h"

typedef struct {
    gpio_pin_t pin;
    uint8_t uart_id;
    uint8_t af;
    uint8_t rx_direction;
}uart_pin_db_t;

typedef struct {
    dma_handle_t dma;
    uint8_t uart_id;
    uint8_t rx_direction;
}uart_dma_db_t;

typedef struct {
    volatile uint32_t* port_enr;
    uint32_t port_enr_bit;
    USART_TypeDef* regs;
    uint8_t uart_id;
    uint8_t periph_prescaler;
}uart_config_db_t;

#ifdef USART3
    #define UART_CONFIG_DB_SIZE   (3)
#else
    #define UART_CONFIG_DB_SIZE   (2)
#endif

#ifdef USART3
    #define UART_DMA_DM_SIZE   (6)
#else
    #define UART_DMA_DM_SIZE   (4)
#endif

#define UART_PIN_DB_SIZE    (6)

extern const uart_config_db_t uart_config_db[UART_CONFIG_DB_SIZE];
extern const uart_dma_db_t uart_dma_db[UART_DMA_DM_SIZE];
extern const uart_pin_db_t uart_pin_db[UART_PIN_DB_SIZE];

#endif
