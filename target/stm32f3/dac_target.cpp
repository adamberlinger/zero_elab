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
#include "dac.h"
#include "mem.h"
#include "dac_target_db.h"
#include "error_codes.h"

gpio_pin_t dac_target_find_pin(int dac_id,int channel){
    uint8_t dac_channel = DEFINE_DAC_CHANNEL(dac_id,channel);
    int i;
    for(i = 0; i < DAC_PIN_DB_SIZE;++i){
        if(dac_pin_db[i].dac_channel == dac_channel){
            return dac_pin_db[i].pin;
        }
    }
    return (gpio_pin_t)0;
}

int dac_target_find_timer(int dac_id,uint8_t* trigger_source){
    int i;
    for(i = 0; i < DAC_TIM_DB_SIZE;++i){
        if(dac_tim_db[i].dac_id == dac_id && timer_is_free(dac_tim_db[i].timer_id)){
            (*trigger_source) = dac_tim_db[i].trigger_source;
            return dac_tim_db[i].timer_id;
        }
    }
    return 0;
}

dma_handle_t dac_target_find_dma(int dac_id,int channel){
    uint8_t dac_channel = DEFINE_DAC_CHANNEL(dac_id,channel);
    int i;
    for(i = 0; i < DAC_DMA_DB_SIZE;++i){
        if(dac_dma_db[i].dac_channel == dac_channel){
            return dac_dma_db[i].dma;
        }
    }
    return (dma_handle_t)0;
}
