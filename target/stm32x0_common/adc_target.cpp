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
#include "adc_target.h"
#include "adc_target_db.h"

gpio_pin_t adc_target_find_pin(int adc_id,int channel){
    uint16_t adc_channel = DEFINE_ADC_CHANNEL(adc_id,channel);
    int i;
    for(i = 0; i < ADC_PIN_DB_SIZE;++i){
        if(adc_pin_db[i].adc_channel == adc_channel){
            return adc_pin_db[i].pin;
        }
    }
    return (gpio_pin_t)0;
}

int adc_target_find_config(gpio_pin_t pin_in, int* adc_id_out, adc_channel_t* adc_channel_out){
    int i;
    *adc_id_out = 0;
    *adc_channel_out = 0;
    for(i = 0;i < ADC_PIN_DB_SIZE;++i){
        if(adc_pin_db[i].pin == pin_in){
            *adc_id_out = EXPORT_ADC_ID(adc_pin_db[i].adc_channel);
            *adc_channel_out = EXPORT_ADC_CHANNEL(adc_pin_db[i].adc_channel);
            return 0;
        }
    }
    return ERROR_NO_CONFIGURATION;
}

int adc_target_find_timer_counter(int adc_id,int* timer_id_out,
    int* counter_id_out, uint8_t* trigger_source, uint8_t* counter_itr, uint8_t* timer_itr){
    int i;
    for(i = 0; i < ADC_TIM_DB_SIZE;++i){
        if(adc_tim_db[i].adc_id == adc_id && timer_is_free(adc_tim_db[i].timer_id)){
            if(timer_target_find_looped_timer(adc_tim_db[i].timer_id,
                counter_id_out, timer_itr, counter_itr)){
                (*trigger_source) = adc_tim_db[i].trigger_source;
                (*timer_id_out) = adc_tim_db[i].timer_id;
                return 1;
            }
        }
    }
    return 0;
}
#ifdef STM32L0XX
dma_handle_t adc_target_find_dma(int adc_id, uint8_t* dma_src){
    int i;
    for(i = 0; i < ADC_DMA_DB_SIZE;++i){
        if(adc_dma_db[i].adc_id == adc_id){
            *dma_src = adc_dma_db[i].dma_select;
            return adc_dma_db[i].dma;
        }
    }
    return (dma_handle_t)0;
}
#else
dma_handle_t adc_target_find_dma(int adc_id){
    int i;
    for(i = 0; i < ADC_DMA_DB_SIZE;++i){
        if(adc_dma_db[i].adc_id == adc_id){
            return adc_dma_db[i].dma;
        }
    }
    return (dma_handle_t)0;
}
#endif

void adc_target_set_pin_analog(gpio_pin_t pin){
    stm32_gpio_init(pin,MODE_AN);
}

uint32_t adc_target_find_sampletime(uint32_t total_frequency, uint32_t *maximum_input_impedance){
    for(int i = ADC_SAMPLETIME_DB_SIZE - 1; i >= 0; i--){
        if(adc_sampletime_db[i].max_frequency > total_frequency){
            *maximum_input_impedance = adc_sampletime_db[i].max_impedance;
            return adc_sampletime_db[i].sample_time_config;
        }
    }
    return 0x0;
}
