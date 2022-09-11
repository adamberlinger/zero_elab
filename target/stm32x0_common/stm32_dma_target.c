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

#ifdef STM32L0XX

void dma_set_source(dma_handle_t dma,uint8_t selection){
    RCC->AHBENR |= RCC_AHBENR_DMAEN;
    int shift = dma_get_channel_num(dma)*4;
    uint32_t mask = (0xF) << shift;
    DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~mask) | (selection << shift);
}

#endif

static void dma1_generic_handler(int channel){
    uint32_t flag = (DMA_ISR_TCIF1 << (channel*4));
    if(DMA1->ISR & flag){
        dma_call_signal(channel,DMA_EVENT_FULL_COMPLETE);
        DMA1->IFCR = flag;
    }
    flag = (DMA_ISR_HTIF1 << (channel*4));
    if(DMA1->ISR & flag){
        dma_call_signal(channel,DMA_EVENT_HALF_COMPLETE);
        DMA1->IFCR = flag;
    }
}

int dma_get_signal_index(dma_handle_t dma_handle){
    int result = dma_get_channel_num(dma_handle);
    return result;
}

int dma_get_irqn(dma_handle_t dma_handle){
    int result = dma_get_channel_num(dma_handle);
    if(result == 0){
        return DMA1_Channel1_IRQn;
    }
    else if(result <= 3){
        return DMA1_Channel2_3_IRQn;
    }
    else {
        return DMA1_Channel4_7_IRQn;
    }
}

void dma1_channel1_handler(void){
    dma1_generic_handler(0);
}

void dma1_channel2_3_handler(void){
    dma1_generic_handler(1);
    dma1_generic_handler(2);
}

void dma1_channel4_7_handler(void){
    dma1_generic_handler(3);
    dma1_generic_handler(4);
    dma1_generic_handler(5);
    dma1_generic_handler(6);
}
