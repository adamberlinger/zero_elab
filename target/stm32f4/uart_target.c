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
#include "uart.h"

void uart_target_init(uart_handle_t uart_handle,const uart_init_t* init_data){
    /* TODO: generalize for all UARTS outside APB2 */
    uint32_t fperiph = (get_core_clock() << 4) >> 1;
    fperiph = (fperiph >> 4); /* fperiph /= 16 */
    uint32_t bauddiv = fperiph / init_data->baudrate;

    if(uart_handle == USART1) RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    if(uart_handle == USART6) RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    uart_handle->BRR = bauddiv;
    uart_handle->CR1 = USART_CR1_TE | USART_CR1_RE | (init_data->parity?USART_CR1_PCE:0)
        | (init_data->parity == UART_PARITY_ODD?USART_CR1_PS:0);
    uart_handle->CR2 = 0x0;
    uart_handle->CR3 = 0x0;

    uart_handle->CR1 |= USART_CR1_UE;

}

void uart_target_send_data(uart_handle_t uart_handle, const char* data, int count){
    int i = 0;
    for(;i < count;++i){
        while((uart_handle->SR & USART_SR_TXE) == 0);
        uart_handle->DR = data[i];
    }
}
