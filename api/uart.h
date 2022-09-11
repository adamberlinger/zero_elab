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
#ifndef _API_UART_H_
#define _API_UART_H_

#include "core.h"
#include "uart_target.h"
#include "gpio.h"

#ifdef __cplusplus
    extern "C" {
#endif

typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_ODD = 1,
    UART_PARITY_EVEN = 2
}uart_parity_t;

typedef struct {
    uint32_t baudrate;
    uart_parity_t parity;
    gpio_pin_t tx_pin;
    gpio_pin_t rx_pin;
    int buffer_tx_size;
    int buffer_rx_size;
}uart_init_t;

int uart_init(uart_handle_t* uart_handle, int uart_id,
    const uart_init_t* init_data, uint8_t* rx_buffer);
void uart_deinit(uart_handle_t* uart_handle);
uint32_t uart_send_data(uart_handle_t* uart_handle, const char* data, uint32_t count);
uint32_t uart_receive_data(uart_handle_t* uart_handle, char* data, uint32_t count);

int uart_target_init(uart_handle_t* uart_handle, int uart_id,
    const uart_init_t* init_data, uint8_t* rx_buffer);
uint32_t uart_target_send_data(uart_handle_t* uart_handle, const char* data, uint32_t count);
uint32_t uart_target_receive_data(uart_handle_t* uart_handle, char* data, uint32_t count);

#ifdef __cplusplus
    }
#endif

#endif
