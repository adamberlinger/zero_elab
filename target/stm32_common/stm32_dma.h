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
#ifndef _STM32_DMA_H_
#define _STM32_DMA_H_

#include <stdint.h>
#include "stm32_common.h"

#ifdef STM32F4XX
    typedef DMA_Stream_TypeDef* dma_handle_t;
#else
    typedef DMA_Channel_TypeDef* dma_handle_t;
#endif

#include "stm32_dma_target.h"
#include "circular_buffer.h"

#ifdef __cplusplus
    extern "C" {
#endif

typedef enum {
    DMA_EVENT_FULL_COMPLETE,
    DMA_EVENT_HALF_COMPLETE
} dma_event_t;

typedef void (*dma_signal_callback_t)(void* arg,dma_event_t dma_event);

typedef struct {
    dma_signal_callback_t callback;
    void* user_arg;
} dma_signal_slot_t;

extern dma_signal_slot_t dma_signals[NUM_DMA_CHANNELS];

typedef enum {
    DMA_TO_MEMORY = 0,
    DMA_TO_PERIPH = 1
} dma_dir_t;

typedef struct {
    void* memory_address;
    void* periph_address;
    uint32_t data_size;
    dma_dir_t direction;
    uint8_t bytes;
}dma_init_t;

typedef void (*dma_callback_t)(dma_handle_t dma_handle,void* data);

void dma_global_init(void);
void dma_call_signal(int index,dma_event_t dma_event);
void dma_generic_callback(void* _circular_buffer,dma_event_t dma_event);

DMA_TypeDef *dma_get_controller(dma_handle_t dma_handle);
int dma_get_channel_num(dma_handle_t dma_handle);

int dma_init(dma_handle_t dma_handle,const dma_init_t* init_data);
int dma_deinit(dma_handle_t dma_handle);
int dma_set_callback(dma_handle_t dma_handle,dma_signal_callback_t callback,void* user_arg);
void dma_start(dma_handle_t dma_handle,int circular);
void dma_stop(dma_handle_t dma_handle);
void dma_wait_for_complete(dma_handle_t dma_handle);
int dma_get_remaining_data(dma_handle_t dma_handle);
int dma_deinit(dma_handle_t dma_handle);

/* Defined in STM32x specific target */
int dma_get_signal_index(dma_handle_t dma_handle);
int dma_get_irqn(dma_handle_t dma_handle);

#ifdef STM32_HASDMAMUX
dma_handle_t stm32_find_dma(void);
void stm32_dma_config_mux(dma_handle_t dma_handle, uint32_t mux_select);
void stm32_dma_alloc_set_watermark(int value);
int stm32_dma_alloc_get_watermark(void);
#endif

#ifdef __cplusplus
    }
#endif

#endif /* _API_DMA_H_ */
