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
#include "stm32_dma.h"

dma_signal_slot_t dma_signals[NUM_DMA_CHANNELS];

DMA_TypeDef *dma_get_controller(dma_handle_t dma_handle){
    return (DMA_TypeDef*)((uint32_t)dma_handle & ~(uint32_t)(0x3FF));
}

int dma_set_callback(dma_handle_t dma_handle,dma_signal_callback_t callback,void* user_arg){
    int index = dma_get_signal_index(dma_handle);
    if(dma_signals[index].callback != 0){
        dma_signals[index].callback = (dma_signal_callback_t)0;
        __DSB();
    }
    dma_signals[index].user_arg = user_arg;
    /* Barrier to ensure user_arg is written first */
    __DSB();
    dma_signals[index].callback = callback;
		return 0;
}

void dma_generic_callback(void* _circular_buffer,dma_event_t dma_event){
    circular_buffer_t* circular_buffer = (circular_buffer_t*)_circular_buffer;
    uint8_t *data = circular_buffer->buffer;
    uint32_t size = (circular_buffer->buffer_size >> 1);
    if(circular_buffer->process_callback){
        if(dma_event == DMA_EVENT_FULL_COMPLETE){
            data = data + size;
        }
        circular_buffer->process_callback(circular_buffer->process_arg,(void*)data,size);
    }
}

int dma_get_channel_num(dma_handle_t dma_handle){
    /* TODO: remove division */
    return (((uint32_t)dma_handle & 0x3FF) - 8) / 20;
}

void dma_call_signal(int index,dma_event_t dma_event){
    if(dma_signals[index].callback){
        dma_signals[index].callback(dma_signals[index].user_arg,dma_event);
    }
}

void dma_global_init(void){
    uint8_t i;
    for(i = 0; i < NUM_DMA_CHANNELS;++i){
        dma_signals[i].callback = (dma_signal_callback_t)0;
    }
}

int dma_init(dma_handle_t dma_handle,const dma_init_t* init_data){
    DMA_TypeDef* dma_controller = dma_get_controller(dma_handle);
    uint32_t ccr = 0;

    if(dma_controller == DMA1){
        RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    }
#ifdef DMA2
    else if(dma_controller == DMA2) {
        RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    }
#endif
    else {
        return -1;
    }

    ccr |= DMA_CCR_MINC | (2 << 12) | (init_data->direction << 4);
    if(init_data->bytes == 2){
        ccr |= (1 << 10) | (1 << 8);
    }
    else if(init_data->bytes == 4){
        ccr |= (2 << 10) | (2 << 8);
    }

    dma_handle->CCR = ccr;
    dma_handle->CPAR = (uint32_t)init_data->periph_address;
    dma_handle->CMAR = (uint32_t)init_data->memory_address;
    dma_handle->CNDTR = init_data->data_size;

    return 0;
}

int dma_deinit(dma_handle_t dma_handle){
    dma_stop(dma_handle);

    DMA_TypeDef* dma_controller = dma_get_controller(dma_handle);

    int channel = dma_get_channel_num(dma_handle);
    dma_controller->IFCR = (0xF << (channel * 4));
    dma_handle->CCR = 0;
#ifdef STM32_HASDMAMUX
    stm32_dma_config_mux(dma_handle,0);
#endif
    return 0;
}

void dma_start(dma_handle_t dma_handle,int circular){
    uint32_t ccr = dma_handle->CCR;
    if(circular){
        /* TODO: move interrupt enable */
        int irqn = dma_get_irqn(dma_handle);
        ccr |= DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_HTIE;
        NVIC_SetPriority((IRQn_Type)irqn, 1);
        NVIC_EnableIRQ((IRQn_Type)irqn);
    }
    else {
        ccr &= ~DMA_CCR_CIRC;
    }
    ccr |= DMA_CCR_EN;
    dma_handle->CCR = ccr;
}

void dma_stop(dma_handle_t dma_handle){
    dma_handle->CCR &= ~DMA_CCR_EN;
}

void dma_wait_for_complete(dma_handle_t dma_handle){
    DMA_TypeDef* dma_controller = dma_get_controller(dma_handle);
    int channel = dma_get_channel_num(dma_handle);
    int flag = (2 << (channel * 4));
    while((dma_controller->ISR & flag) == 0);
    dma_controller->IFCR = flag;
}

int dma_get_remaining_data(dma_handle_t dma_handle){
    return dma_handle->CNDTR;
}

#ifdef STM32_HASDMAMUX
static int dma_alloc_index = 0;

dma_handle_t stm32_find_dma(void){
    /* TODO: now supports only DMA1 */
    dma_handle_t result = NULL;
    if(dma_alloc_index <= 4){
        result = (dma_handle_t)(DMA1_Channel1_BASE + dma_alloc_index * 0x14);
        dma_alloc_index++;
    }
    return result;
}

void stm32_dma_alloc_set_watermark(int value){
    dma_alloc_index = value;
}

int stm32_dma_alloc_get_watermark(){
    return dma_alloc_index;
}

void stm32_dma_config_mux(dma_handle_t dma_handle, uint32_t mux_select){
    /* TODO: now supports only DMA1 */
    uint32_t dma_num = (uint32_t)(dma_handle);
    dma_num -= (uint32_t)(DMA1_Channel1);
    dma_num /= 0x14;
    if(dma_num >= 7){
        return;
    }
    DMAMUX1_Channel0[dma_num].CCR = (0x7F & mux_select);
}
#endif