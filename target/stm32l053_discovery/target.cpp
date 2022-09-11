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

#include "adc.h"
#include "pwm.h"
#include "oscilloscope.h"
#include "generator.h"
#include "voltmeter.h"

//#define USE_GENERATOR

/* List of unconnected pins to be set to floating-input */
//const gpio_pin_t gpio_unused_pin_range[GPIO_UNUSED_PIN_RANGE_SIZE] = {
//        DEFINE_PIN(GPIOA_BASE,0), DEFINE_PIN(GPIOA_BASE,1),   /* PA0 - PA1 */
//        DEFINE_PIN(GPIOA_BASE,3), DEFINE_PIN(GPIOA_BASE,12),  /* PA3 - PA12 */
//        DEFINE_PIN(GPIOB_BASE,0), DEFINE_PIN(GPIOB_BASE,1),   /* PB0 - PB1 */
//        DEFINE_PIN(GPIOB_BASE,3), DEFINE_PIN(GPIOB_BASE,7),   /* PB3 - PB7 */
//};

uint32_t vdda_value = 3300;

uint32_t get_vdda(void){
    return vdda_value;
}

void target_init(){
    stm32_common_init();
}

VOLTMETER_DECLARE(volt1, 12, 2);
OSC_DECLARE(osc1, 12, 1, 1024);
GEN_DECLARE(gen1, 100000, 1, 512);

PWM_DECLARE(pwm1, DEFINE_PIN(GPIOB_BASE, 13));

static void init_timer_pwm(){
    PWM_MODULE_INIT(pwm1, 21);
    GEN_MODULE_INIT(gen1,1);
}

static void init_voltmeter_variant(){
    OSC_MODULE_DEINIT(osc1);
    init_timer_pwm();
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,4),0);
    VOLTMETER_SET_REFCHANNEL(volt1,17);
    VOLTMETER_MODULE_INIT(volt1, 1);
    vdda_value = volt1_module->getVDDA();
}

static void init_oscilloscope_variant(){
    init_timer_pwm();

    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE, 4),0);
    OSC_MODULE_PREPARE(osc1, 1);
    OSC_MODULE_INIT(osc1);
}

uint32_t current_config = 0;

void set_next_device_configuration(){
    current_config = (1+current_config) & 0x1;
    Module::removeAll();
    mem_free();
    timer_unlock_all();
    if(!current_config){
        target_configuration_name = "Voltmeter";
        init_voltmeter_variant();
    }
    else {
        target_configuration_name = "Oscilloscope";
        init_oscilloscope_variant();
    }
}

void functions_init(void){
    target_configuration_name = "Voltmeter";
    init_voltmeter_variant();
}
