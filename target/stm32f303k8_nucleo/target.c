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

#include "pwm.h"
#include "oscilloscope.h"
#include "voltmeter.h"
#include "pwm_input.h"
#include "pulse_counter.h"
#include "dac.h"
#include "generator.h"

/* List of unconnected pins to be set to floating-input */
const gpio_pin_t gpio_unused_pin_range[GPIO_UNUSED_PIN_RANGE_SIZE] = {
        DEFINE_PIN(GPIOA_BASE,0), DEFINE_PIN(GPIOA_BASE,1),   /* PA0 - PA1 */
        DEFINE_PIN(GPIOA_BASE,3), DEFINE_PIN(GPIOA_BASE,12),  /* PA3 - PA12 */
        DEFINE_PIN(GPIOB_BASE,0), DEFINE_PIN(GPIOB_BASE,1),   /* PB0 - PB1 */
        DEFINE_PIN(GPIOB_BASE,3), DEFINE_PIN(GPIOB_BASE,7),   /* PB3 - PB7 */
        DEFINE_PIN(GPIOF_BASE,0), DEFINE_PIN(GPIOF_BASE,1)    /* PF0 - PF1 */
};

static comm_t main_uart;
static uart_handle_t main_uart_handle;
static int main_uart_initialized = 0;
static const uart_init_t main_uart_params = {
    115200,
    UART_PARITY_NONE,
    DEFINE_PIN(GPIOA_BASE,2),/* TX pin */
    DEFINE_PIN(GPIOA_BASE,15),/* RX pin */
    0,
    256
};

comm_t *get_main_comm(void){
    if(!main_uart_initialized){
        uart_init(&main_uart_handle,2,&main_uart_params);
        main_uart.hw_handle = &main_uart_handle;
        main_uart.write_callback = (comm_write_callback_t)uart_send_data;
        main_uart.read_callback = (comm_read_callback_t)uart_receive_data;
        main_uart_initialized = 1;
    }
    return &main_uart;
}

void target_init(){
    stm32_common_init();
}

static adc_handle_t default_adc_handle;

const static adc_init_t default_adc_init = {
    10000, /* sample rate */
    12,
    3,
    1024*3,
    ADC_HINT_FAST
};

const static timer_init_t pwm_init_data = {
    TIMER_USAGE_PWM,
    TIMER_INIT_FREQUENCY,
    100,
    0x8000,
    DEFINE_PIN(GPIOA_BASE,12)
};

const static timer_init_t pwm_in_init_data = {
    TIMER_USAGE_PWM_INPUT,
    TIMER_INIT_FREQUENCY,
    1000000,
    0x8000,
    DEFINE_PIN(GPIOB_BASE,4)
};

const static adc_init_t volt_adc_config = {
    0,
    12,
    4, /* Num channels */
    0,
    ADC_HINT_NONE
};

static dac_handle_t default_dac_handle;

const static dac_init_t gen_dac_config = {
    1000000,
    1,
    512
};

static timer_init_t pulse_counter_config = {
    TIMER_USAGE_PULSE_COUNTER,
    TIMER_INIT_FREQUENCY,
    0,
    0,
    0
};

uint32_t vdda_value = 0;

uint32_t get_vdda(void){
    return vdda_value;
}

void functions_init(void){
    module_t* pwm_module = app_malloc(sizeof(module_t));
    module_t* osc_module = app_malloc(sizeof(module_t));
    module_t* volt_module = app_malloc(sizeof(module_t));
    module_t* pwm_in_module = app_malloc(sizeof(module_t));
    module_t* pulse_counter_module = app_malloc(sizeof(module_t));
    module_t* gen_module = app_malloc(sizeof(module_t));

    int adc_id;
    adc_channel_t config_adc_channels[4];

    osc_make_module(osc_module,app_malloc(sizeof(osc_handle_t)));
    pwm_make_module(pwm_module,app_malloc(sizeof(pwm_handle_t)));
    volt_make_module(volt_module,app_malloc(sizeof(volt_handle_t)));
    pwm_in_make_module(pwm_in_module,app_malloc(sizeof(pwm_in_handle_t)));
    //pulse_counter_make_module(pulse_counter_module,app_malloc(sizeof(pulse_counter_handle_t)));

    pwm_init((pwm_handle_t*)pwm_module->handle,2,16,&pwm_init_data);
    //pulse_counter_init((pulse_counter_handle_t*)pulse_counter_module->handle,6,4,&pulse_counter_config);
    pwm_in_init((pwm_in_handle_t*)pwm_in_module->handle,4,3,&pwm_in_init_data);

    gen_make_module(gen_module,app_malloc(sizeof(gen_handle_t)));

    dac_init(&default_dac_handle, 1, &gen_dac_config);
    gen_init((gen_handle_t*)gen_module->handle,5,dac_get_circular_buffer(&default_dac_handle),
        gen_dac_config.sample_rate,256,12);


    adc_id = 2;
    config_adc_channels[0] = 2;
    config_adc_channels[1] = 3;
    config_adc_channels[2] = 4;
    adc_init(&default_adc_handle,adc_id,&default_adc_init,config_adc_channels);

    osc_init((osc_handle_t*)osc_module->handle,
        adc_get_circular_buffer(&default_adc_handle),1,12,default_adc_init.num_channels);


    adc_id = 1;
    config_adc_channels[0] = 4;
    config_adc_channels[1] = 11;
    config_adc_channels[2] = 21; /* Channel 1 differential */

    config_adc_channels[3] = 18;

    volt_init((volt_handle_t*)volt_module->handle,3,adc_id,&volt_adc_config,config_adc_channels);

    vdda_value = volt_get_vdda(volt_module->handle);

    module_register(pwm_module);
    module_register(volt_module);
    module_register(pwm_in_module);
    //module_register(pulse_counter_module);
    module_register(osc_module);
    module_register(gen_module);
}
