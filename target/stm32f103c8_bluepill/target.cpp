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
#include "board_pindefs.h"

#include "adc.h"
#include "pwm.h"
#include "pwm_input.h"
#include "voltmeter.h"
#include "oscilloscope.h"

#if 0
/* List of unconnected pins to be set to floating-input */
const gpio_pin_t gpio_unused_pin_range[GPIO_UNUSED_PIN_RANGE_SIZE] = {
        DEFINE_PIN(GPIOA_BASE,0), DEFINE_PIN(GPIOA_BASE,1),   /* PA0 - PA1 */
        DEFINE_PIN(GPIOA_BASE,4), DEFINE_PIN(GPIOA_BASE,12),  /* PA4 - PA12 */
        DEFINE_PIN(GPIOA_BASE,15), DEFINE_PIN(GPIOA_BASE,15), /* PA15 */
        DEFINE_PIN(GPIOB_BASE,0), DEFINE_PIN(GPIOB_BASE,2),   /* PB0 - PB2 */
        DEFINE_PIN(GPIOB_BASE,4), DEFINE_PIN(GPIOB_BASE,15),  /* PB4 - PB15 */
        DEFINE_PIN(GPIOF_BASE,1), DEFINE_PIN(GPIOF_BASE,1),   /* PF1 */
        DEFINE_PIN(GPIOC_BASE,0), DEFINE_PIN(GPIOF_BASE,12),  /* PC0 - PC12 */
        DEFINE_PIN(GPIOC_BASE,14), DEFINE_PIN(GPIOF_BASE,15)  /* PC14 - PC15 */
};
#else
const gpio_pin_t gpio_unused_pin_range[1] = {0};
#endif

VOLTMETER_DECLARE(volt1, 12, 4);
PWM_DECLARE(pwm1, DEFINE_PIN(GPIOB_BASE, 6));
OSC_DECLARE(osc1, 12, 4, 10*1024);
PWM_IN_DECLARE(pwm_in1, DEFINE_PIN(GPIOA_BASE,8));

uint32_t vdda_value = 3300;

uint32_t get_vdda(void){
    return vdda_value;
}

void target_init(){
    stm32_common_init();

    stm32_gpio_init(DEFINE_PIN(GPIOA, 12), MODE_OUT_OD);
    GPIOA->BRR = (1 << 12);
    wait_ms(10);
    stm32_gpio_init(DEFINE_PIN(GPIOA, 12), MODE_IN);
}

uint32_t current_config = 0;

uint32_t get_target_capabilities(){
  if(current_config == 0){
    return 0x1C;
  }
  else {
    return 0x17;
  }
}

static void init_common(){
    PWM_MODULE_INIT(pwm1,4);
    PWM_IN_MODULE_INIT(pwm_in1, 1);
}

static void init_voltmeter_variant(){
    init_common();

    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,0),0);
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,1),1);
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,2),2);
    VOLTMETER_SET_REFCHANNEL(volt1,17);

    VOLTMETER_MODULE_INIT(volt1, 1);
}

static void init_oscilloscope_variant(){
    init_common();

    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,0),0);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,1),1);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,2),2);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,3),3);

    OSC_MODULE_PREPARE(osc1,1);

    OSC_MODULE_INIT(osc1);
    OSC_MODULE_LIMIT_FREQUENCY(osc1,200000);
}

void set_next_device_configuration(){
    current_config = (1+current_config) & 0x1;
    Module::removeAll();
    mem_free();
    timer_unlock_all();

    if(current_config == 0){
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

    vdda_value = volt1_module->getVDDA();
}
