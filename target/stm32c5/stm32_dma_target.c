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

static void dma_generic_handler(dma_handle_t dma_handle){
    int channel = dma_get_channel_num(dma_handle);
    if(dma_handle->CSR & DMA_CSR_TCF){
        dma_call_signal(channel,DMA_EVENT_FULL_COMPLETE);
        dma_handle->CFCR = DMA_CFCR_TCF;
    }
    if(dma_handle->CSR & DMA_CSR_HTF){
        dma_call_signal(channel,DMA_EVENT_HALF_COMPLETE);
        dma_handle->CFCR = DMA_CFCR_HTF;
    }
}

int dma_get_signal_index(dma_handle_t dma_handle){
    return dma_get_channel_num(dma_handle);
}

int dma_get_irqn(dma_handle_t dma_handle){
    int result = dma_get_channel_num(dma_handle);
#if NUM_DMA_CHANNELS > 8
    if(result < 8){
#else
    if(result < 4){
#endif
        return LPDMA1_CH0_IRQn + result;
    }
    else {
#if NUM_DMA_CHANNELS > 8
        return LPDMA2_CH0_IRQn + (result-8);
#else
        return LPDMA2_CH0_IRQn + (result-4);
#endif
    }
}

/* TODO: merge/refactor with non-GPDMA code */
static int dma_alloc_index = 0;

dma_handle_t stm32_find_dma(void){
    dma_handle_t result = NULL;

    if(dma_alloc_index < NUM_DMA_CHANNELS){
#if NUM_DMA_CHANNELS==12
        if(dma_alloc_index >= 8){
            result = (dma_handle_t)(LPDMA1_CH4 + (dma_alloc_index-8)*0x80);
        } else
#endif
        if((dma_alloc_index & 0x1) == 0){
            result = (dma_handle_t)(LPDMA1_CH0 + (dma_alloc_index/2)*0x80);
        }
        else {
            result = (dma_handle_t)(LPDMA2_CH0 + (dma_alloc_index/2)*0x80);
        }
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

void gpdma1_channel0_handler(void){
    dma_generic_handler(LPDMA1_CH0);
}
void gpdma1_channel1_handler(void){
    dma_generic_handler(LPDMA1_CH1);
}
void gpdma1_channel2_handler(void){
    dma_generic_handler(LPDMA1_CH2);
}
void gpdma1_channel3_handler(void){
    dma_generic_handler(LPDMA1_CH3);
}
#if NUM_DMA_CHANNELS > 8
void gpdma1_channel4_handler(void){
    dma_generic_handler(LPDMA1_CH4);
}
void gpdma1_channel5_handler(void){
    dma_generic_handler(LPDMA1_CH5);
}
void gpdma1_channel6_handler(void){
    dma_generic_handler(LPDMA1_CH6);
}
void gpdma1_channel7_handler(void){
    dma_generic_handler(LPDMA1_CH7);
}
#endif

void gpdma2_channel0_handler(void){
    dma_generic_handler(LPDMA2_CH0);
}
void gpdma2_channel1_handler(void){
    dma_generic_handler(LPDMA2_CH1);
}
void gpdma2_channel2_handler(void){
    dma_generic_handler(LPDMA2_CH2);
}
void gpdma2_channel3_handler(void){
    dma_generic_handler(LPDMA2_CH3);
}
#if NUM_DMA_CHANNELS > 12
void gpdma2_channel4_handler(void){
    dma_generic_handler(LPDMA2_CH4);
}
void gpdma2_channel5_handler(void){
    dma_generic_handler(LPDMA2_CH5);
}
void gpdma2_channel6_handler(void){
    dma_generic_handler(LPDMA2_CH6);
}
void gpdma2_channel7_handler(void){
    dma_generic_handler(LPDMA2_CH7);
}
#endif

