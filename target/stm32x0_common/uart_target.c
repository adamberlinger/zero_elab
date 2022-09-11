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
#include "mem.h"

int uart_target_init(uart_handle_t* uart_handle,int uart_id,
    const uart_init_t* init_data,uint8_t* buffer_rx){
    uint32_t fperiph = get_core_clock();
    uint16_t bauddiv;
    int af_tx,af_rx;
#ifdef STM32L0XX
    uint8_t dma_src;
#endif

    if(uart_id == 1){
        uart_handle->regs = USART1;
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
#ifdef STM32F0XX
        af_tx = af_rx = 1;
	uart_handle->dma_tx = DMA1_Channel2;
	uart_handle->dma_rx = DMA1_Channel3;
#else
	/* TODO: add exeption for GPIOB pins */
	af_tx = af_rx = 4;
	dma_src = 3;
        uart_handle->dma_tx = DMA1_Channel4;
        uart_handle->dma_rx = DMA1_Channel5;
#endif

    }
    else if(uart_id == 2){
        uart_handle->regs = USART2;
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
#ifdef STM32F0XX
        af_tx = af_rx = 1;
	uart_handle->dma_tx = DMA1_Channel4;
	uart_handle->dma_rx = DMA1_Channel5;
#else
	af_tx = af_rx = 4;
	dma_src = 4;
        uart_handle->dma_tx = DMA1_Channel7;
        uart_handle->dma_rx = DMA1_Channel6;
#endif
    }
    else {
        /* TODO: report error */
        return ERROR_NO_CONFIGURATION;
    }

    stm32_gpio_af(init_data->tx_pin,MODE_OUT_PP,af_tx);
    stm32_gpio_pullup(init_data->tx_pin);
    stm32_gpio_af(init_data->rx_pin,MODE_OUT_PP,af_rx);

    bauddiv = (fperiph + (init_data->baudrate / 2)) / init_data->baudrate;

    uart_handle->regs->CR1 = 0x0;
    uart_handle->regs->CR2 = 0x0;
    uart_handle->regs->CR3 = 0x0;
    uart_handle->regs->BRR = bauddiv;
    uart_handle->regs->CR1 = USART_CR1_TE | USART_CR1_RE | (init_data->parity?USART_CR1_PCE:0)
        | (init_data->parity == UART_PARITY_ODD?USART_CR1_PS:0);

    uart_handle->regs->ICR = 0xFFFFFFFF;


    /* Prepare RX buffer */
    if(init_data->buffer_rx_size){
        if(buffer_rx){
            uart_handle->buffer_rx = buffer_rx;
        }
        else {
            uart_handle->buffer_rx = app_malloc(init_data->buffer_rx_size);
        }
        if(uart_handle->buffer_rx){
            dma_init_t dma_rx_init;
            uart_handle->buffer_rx_size = init_data->buffer_rx_size;
            uart_handle->buffer_rx_ptr = 0;

            dma_rx_init.memory_address = uart_handle->buffer_rx;
            dma_rx_init.periph_address = (void*)&uart_handle->regs->RDR;
            dma_rx_init.data_size = uart_handle->buffer_rx_size;
            dma_rx_init.direction = DMA_TO_MEMORY;
            dma_rx_init.bytes = 1;
            dma_init(uart_handle->dma_rx,&dma_rx_init);
#ifdef STM32L0XX
	    dma_set_source(uart_handle->dma_rx,dma_src);
#endif

            dma_start(uart_handle->dma_rx,1);
            uart_handle->regs->CR3 |= USART_CR3_DMAR;
        }
    }

    uart_handle->regs->CR1 |= USART_CR1_UE;

    return ERROR_NONE;
}

uint32_t uart_target_send_data(uart_handle_t* uart_handle, const char* data, uint32_t count){
    uint32_t i = 0;
    for(;i < count;++i){
        while((uart_handle->regs->ISR & USART_ISR_TXE) == 0);
        uart_handle->regs->TDR = data[i];
    }
    return count;
}

uint32_t uart_target_receive_data(uart_handle_t* uart_handle, char* data, uint32_t count){
    uint32_t result = 0,i = 0;
    if(uart_handle->buffer_rx_size){
        for(;i < count;++i){
            if((uart_handle->buffer_rx_size - uart_handle->buffer_rx_ptr) ==
                dma_get_remaining_data(uart_handle->dma_rx)){
                break;
            }
            data[i] = uart_handle->buffer_rx[uart_handle->buffer_rx_ptr];
            uart_handle->buffer_rx_ptr++;
            if(uart_handle->buffer_rx_ptr >= uart_handle->buffer_rx_size)
                uart_handle->buffer_rx_ptr = 0;
            result++;
        }
    }
    else {
        if(uart_handle->regs->ISR & USART_ISR_RXNE){
            data[0] = uart_handle->regs->RDR;
            result = 1;
        }
    }
    return result;
}
