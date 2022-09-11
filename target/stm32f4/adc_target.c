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
#include "adc.h"

const gpio_pin_t adc_channel_map[19] = {
    /* ADC1 pin mapping */
    DEFINE_PIN(GPIOA,0),DEFINE_PIN(GPIOA,1),DEFINE_PIN(GPIOA,2),
    DEFINE_PIN(GPIOA,3),DEFINE_PIN(GPIOA,4),DEFINE_PIN(GPIOA,5),
    DEFINE_PIN(GPIOA,6),DEFINE_PIN(GPIOA,7),DEFINE_PIN(GPIOB,0),
    DEFINE_PIN(GPIOB,1),DEFINE_PIN(GPIOC,0),DEFINE_PIN(GPIOC,1),
    DEFINE_PIN(GPIOC,2),DEFINE_PIN(GPIOC,3),DEFINE_PIN(GPIOC,4),
    DEFINE_PIN(GPIOC,5),
    /* Internal channels have no pin assignment */
    0,0,0
};

static void adc_target_set_pin_analog(gpio_pin_t pin){
    GPIO_TypeDef* port = stm32_get_port(pin);
    uint8_t pindef = stm32_get_pindef(pin);
    uint8_t pindef_2 = pindef*2;

    port->PUPDR &= ~(uint32_t)(0x3 << pindef_2);
    port->MODER |= (0x3 << pindef_2);
}

uint32_t adc_target_get_max_sample_time(uint32_t sample_time){
    /* TODO: implement */
    return 0;
}

int adc_target_init(adc_handle_t adc_handle,const adc_init_t *init_data,
    const adc_channel_t* channel_info){

    uint32_t i;
    uint8_t sample_time = adc_target_get_max_sample_time(init_data->sample_rate);
    /* TODO: setup ADC */
    adc_handle->CR1 = (ADC_CR1_SCAN);

    /* Use TIM3 and TIM4 */

    /* Set same sampling time for all channels */
    adc_handle->SMPR1 = 0;
    for(i = 0;i < 9;++i){
        adc_handle->SMPR1 |= (sample_time << (3*i));
    }
    adc_handle->SMPR2 = (adc_handle->SMPR1) | (sample_time << (3*9));

    /* Configure channels */

    adc_handle->SQR1 = 0;
    adc_handle->SQR2 = 0;
    adc_handle->SQR3 = 0;

    adc_handle->SQR1 = ((init_data->num_channels-1) << 20);

    for(i = 0;i < init_data->num_channels;++i){
        gpio_pin_t pin = adc_channel_map[channel_info[i]];
        if(pin){
            adc_target_set_pin_analog(pin);
        }

        if(i <= 5){
            adc_handle->SQR3 |= (channel_info[i] << (i*5));
        }
        else if (i <= 11){
            adc_handle->SQR2 |= (channel_info[i] << ((i-6)*5));
        }
        else {
            adc_handle->SQR1 |= (channel_info[i] << ((i-12)*5));
        }
    }

    return 0;
}
