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
#include "core.h"
#include "uart.h"
#include "board_pindefs.h"
#include "nucleo_init.h"

#include "adc.h"
#include "pwm.h"
#include "oscilloscope.h"
#include "generator.h"

/* List of unconnected pins to be set to floating-input */
const gpio_pin_t gpio_unused_pin_range[GPIO_UNUSED_PIN_RANGE_SIZE] = {
        DEFINE_PIN(GPIOA_BASE,0), DEFINE_PIN(GPIOA_BASE,1),   /* PA0 - PA1 */
        DEFINE_PIN(GPIOA_BASE,3), DEFINE_PIN(GPIOA_BASE,10),  /* PA3 - PA12 */
        DEFINE_PIN(GPIOB_BASE,0), DEFINE_PIN(GPIOB_BASE,1),   /* PB0 - PB1 */
        DEFINE_PIN(GPIOB_BASE,3), DEFINE_PIN(GPIOB_BASE,7),   /* PB3 - PB7 */
};

const static adc_init_t osc_adc_init = {
    10000,
    12,
    3,  /* num channels */
    1024,
    ADC_HINT_FAST
};

#ifdef USE_GENERATOR
const static timer_init_t pwm_init_data = {
    TIMER_USAGE_PWM_GENERATOR,
    TIMER_INIT_FREQUENCY,
    10000,
    0x8000,
    DEFINE_PIN(GPIOB_BASE, 0),
    1024
};
#else
const static timer_init_t pwm_init_data = {
    TIMER_USAGE_PWM,
    TIMER_INIT_FREQUENCY,
    10000,
    0x8000,
    DEFINE_PIN(GPIOB_BASE, 0),
    0
};
#endif

uint32_t vdda_value = 3300;

uint32_t get_vdda(void){
    return vdda_value;
}

void target_init(){
    stm32_common_init();
}

static adc_handle_t default_adc_handle;
static timer_handle_t gen_tim_handle;

void functions_init(void){
    adc_channel_t config_adc_channels[3];
    additional_buffer_t* log_buffer;
    int adc_id;
#ifdef USE_GENERATOR
    module_t* gen_module = app_malloc(sizeof(module_t));
#else
    module_t* pwm_module = app_malloc(sizeof(module_t));
#endif
    module_t* osc_module = app_malloc(sizeof(module_t));

    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,1), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,3), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,4), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,5), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,6), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,7), MODE_IN);


    osc_make_module(osc_module,app_malloc(sizeof(osc_handle_t)));
#ifdef USE_GENERATOR
    gen_make_module(gen_module,app_malloc(sizeof(gen_handle_t)));

    timer_init(&gen_tim_handle,3,&pwm_init_data);

    gen_init((gen_handle_t*)gen_module->handle,MODULE_CHANNEL_GEN,
        gen_tim_handle.circular_buffer,10000,512,8);
#else
    pwm_make_module(pwm_module,app_malloc(sizeof(pwm_handle_t)));
    pwm_init((pwm_handle_t*)pwm_module->handle,2,3,&pwm_init_data);
#endif

    adc_find_config(DEFINE_PIN(GPIOA_BASE,0), &adc_id, &config_adc_channels[0]);
    adc_find_config(DEFINE_PIN(GPIOA_BASE,1), &adc_id, &config_adc_channels[1]);
    adc_find_config(DEFINE_PIN(GPIOA_BASE,7), &adc_id, &config_adc_channels[2]);
    adc_init(&default_adc_handle,adc_id,&osc_adc_init,config_adc_channels);

    log_buffer = buffer_additional_alloc(adc_get_circular_buffer(&default_adc_handle),512);
    timer_target_configure_custom_dma(&default_adc_handle.timer,(void*)&GPIOB->IDR,log_buffer);

    osc_init((osc_handle_t*)osc_module->handle,
        adc_get_circular_buffer(&default_adc_handle),MODULE_CHANNEL_OSC,12,osc_adc_init.num_channels);

    osc_set_channel_mask((osc_handle_t*)osc_module->handle, 0x3);

    module_register(osc_module);
#ifdef USE_GENERATOR
    module_register(gen_module);
#else
    module_register(pwm_module);
#endif
}
