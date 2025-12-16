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
#include "pwm_input.h"
#include "voltmeter.h"
#include "oscilloscope.h"
#include "pwm_generator.h"
#include "stm32_dma.h"
#include "package.h"

/* TODO: set list of unused pins
    - this is demo torwards 8-pin package => no unused pins on 8-pin pacakge */
const gpio_pin_t gpio_unused_pin_range[1] = {0};

int dma_watermark;

VOLTMETER_DECLARE(volt1, 12, 4);
/* PB7 -> D0 */
PWM_DECLARE(pwm1_nucleo, DEFINE_PIN(GPIOB_BASE, 7));
PWM_DECLARE(pwm1_pkg, DEFINE_PIN(GPIOA_BASE, 4));

OSC_DECLARE(osc1, 12, 4, 16*1024);
PWM_IN_DECLARE(pwm_in1, DEFINE_PIN(GPIOA_BASE,5));

uint32_t vdda_value = 3300;

uint32_t get_vdda(void){
    return vdda_value;
}

void target_init(){
    gpio_pin_t toggle_pin;

    c0_package_init();
    stm32_common_init();

    /* Alive signaling */
    toggle_pin = (pkg_index == 1)?DEFINE_PIN(GPIOA_BASE, 8):DEFINE_PIN(GPIOA_BASE, 7);
    stm32_gpio_init(toggle_pin, MODE_OUT_PP);
    for(int i = 0;i < 6;++i){
        gpio_toggle(toggle_pin);
        wait_ms(100);
    }
    stm32_gpio_init(toggle_pin, MODE_IN);
}

uint32_t current_config = 0;

uint32_t get_target_capabilities(){
  if(current_config == 0){
    return 0x3C;
  }
  else {
    return 0x37;
  }
}

const timer_init_t tim16_init = {
    .usage = TIMER_USAGE_PWM_GENERATOR,
    .time_type = TIMER_INIT_FREQUENCY,
    .time_value = 187500,
    .duty_cycle = 50,
    .pin = DEFINE_PIN(GPIOA_BASE,6),
    .buffer_size = 64*2,
};

timer_handle_t tim16_handle;

static void init_generator(){

    timer_init(&tim16_handle, 16,  &tim16_init);

    ModulePWMGenerator* gen_module = new ModulePWMGenerator(5, &tim16_handle, 187500);
    (void)gen_module; 
}

static void init_common(){
    if(C0_IS_NUCLEO){
        PWM_MODULE_INIT(pwm1_nucleo,17);
    }
    else {
        PWM_MODULE_INIT(pwm1_pkg,17);
    }
    PWM_IN_MODULE_INIT(pwm_in1, 2);
}

static void init_voltmeter_variant(){
    init_common();
    init_generator();

    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,0),0);
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,1),1);
    if(C0_IS_NUCLEO){
        VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,4),2);
    }
    else {
        VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,2),2);
    }
    VOLTMETER_SET_REFCHANNEL(volt1,10);

    VOLTMETER_MODULE_INIT(volt1, 1);
}


static void init_oscilloscope_variant(){
    init_common();
    init_generator();

    /* AN0 - AN3 on arduino header */
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,0),0);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,1),1);
    if(C0_IS_NUCLEO){
        OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,4),2);
        OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOB_BASE,1),3);
    }
    else {
        OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,2),2);
        OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,3),3);
    }

    OSC_MODULE_PREPARE(osc1,1);

    OSC_MODULE_INIT(osc1);
    OSC_MODULE_LIMIT_FREQUENCY(osc1,200000);
}

void set_next_device_configuration(){
    current_config = (1+current_config) & 0x1;
    Module::removeAll();
    OSC_MODULE_DEINIT(osc1);
    mem_free();
    timer_unlock_all();
    stm32_dma_alloc_set_watermark(dma_watermark);

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
    dma_watermark = stm32_dma_alloc_get_watermark();

    target_configuration_name = "Voltmeter";

    init_voltmeter_variant();

    vdda_value = volt1_module->getVDDA();
}

const uint8_t* get_target_pinout(uint16_t* length){
  static uint8_t pinout_val[PINOUT_SIZE(16)];
  int size = 1;
  uint8_t af = ((current_config & 0x1) == 0)?PINOUT_VOLT:PINOUT_OSC;

  if((pkg_index == 0) || (pkg_index == 1)){
    pinout_val [0] = PINOUT_TSSOP20;

    PINOUT_ADD_SYS( pinout_val, size, 4, PINOUT_VDD);
    PINOUT_ADD_SYS( pinout_val, size, 5, PINOUT_GND);
    PINOUT_ADD_SYS( pinout_val, size, 6, PINOUT_NRST);
    PINOUT_ADD_SPEC(pinout_val, size, 7, 'A', 0, af, 0);
    PINOUT_ADD_SPEC(pinout_val, size, 8, 'A', 1, af, 1);
    PINOUT_ADD_SPEC(pinout_val, size, 9, 'A', 2, af, 2);
    if(current_config & 0x1){
      PINOUT_ADD_SPEC(pinout_val, size,10, 'A', 3, af, 3);
    }
    PINOUT_ADD_SPEC(pinout_val, size,11, 'A', 4, PINOUT_PWM, 0);
    PINOUT_ADD_SPEC(pinout_val, size,12, 'A', 5, PINOUT_PWM_IN, 0);
    PINOUT_ADD_SPEC(pinout_val, size,13, 'A', 6, PINOUT_GEN, 0);
    PINOUT_ADD_CORE(pinout_val, size,14, 'A', 7, PINOUT_START_BLINK);
    PINOUT_ADD_CORE(pinout_val, size,16, 'A',11, PINOUT_USB_DM);
    PINOUT_ADD_CORE(pinout_val, size,17, 'A',12, PINOUT_USB_DP);
    PINOUT_ADD_CORE(pinout_val, size,18, 'A',13, PINOUT_SWDIO);
    PINOUT_ADD_SYS( pinout_val, size,19, PINOUT_BOOT0);
    PINOUT_ADD_CORE(pinout_val, size,19, 'A',14, PINOUT_SWCLK);
  }
  else if((pkg_index == 2) || (pkg_index == 3)){
    pinout_val[0] = PINOUT_LQFP32;

    PINOUT_ADD_SYS( pinout_val, size, 4, PINOUT_VDD);
    PINOUT_ADD_SYS( pinout_val, size, 5, PINOUT_GND);
    PINOUT_ADD_SYS( pinout_val, size, 6, PINOUT_NRST);
    PINOUT_ADD_SPEC(pinout_val, size, 7, 'A', 0, af, 0);
    PINOUT_ADD_SPEC(pinout_val, size, 8, 'A', 1, af, 1);
    PINOUT_ADD_SPEC(pinout_val, size, 9, 'A', 2, af, 2);
    if(current_config & 0x1){
      PINOUT_ADD_SPEC(pinout_val, size,10, 'A', 3, af, 3);
    }
    PINOUT_ADD_SPEC(pinout_val, size,11, 'A', 4, PINOUT_PWM, 0);
    PINOUT_ADD_SPEC(pinout_val, size,12, 'A', 5, PINOUT_PWM_IN, 0);
    PINOUT_ADD_SPEC(pinout_val, size,13, 'A', 6, PINOUT_GEN, 0);
    PINOUT_ADD_CORE(pinout_val, size,14, 'A', 7, PINOUT_START_BLINK);
    PINOUT_ADD_CORE(pinout_val, size,22, 'A',11, PINOUT_USB_DM);
    PINOUT_ADD_CORE(pinout_val, size,23, 'A',12, PINOUT_USB_DP);
    PINOUT_ADD_CORE(pinout_val, size,24, 'A',13, PINOUT_SWDIO);
    PINOUT_ADD_SYS( pinout_val, size,25, PINOUT_BOOT0);
    PINOUT_ADD_CORE(pinout_val, size,25, 'A',14, PINOUT_SWCLK);
  }
  else if((pkg_index == 4) || (pkg_index == 5)){
    pinout_val[0] = PINOUT_ARDUINO;

    PINOUT_ADD_SPEC(pinout_val, size,   0, 'A', 0, af, 0);
    PINOUT_ADD_SPEC(pinout_val, size,   1, 'A', 1, af, 1);
    PINOUT_ADD_SPEC(pinout_val, size,   2, 'A', 4, af, 2);
    if(current_config & 0x1){
      PINOUT_ADD_SPEC(pinout_val, size,   3, 'B', 0, af, 3);
    }
    PINOUT_ADD_SPEC(pinout_val, size,6+ 0, 'B', 7, PINOUT_PWM, 0);
    PINOUT_ADD_CORE(pinout_val, size,7+11, 'A', 7, PINOUT_START_BLINK);
    PINOUT_ADD_SPEC(pinout_val, size,7+12, 'A', 6, PINOUT_GEN, 0);
    PINOUT_ADD_SPEC(pinout_val, size,7+13, 'A', 5, PINOUT_PWM_IN, 0);
  }
  else {
    pinout_val[0] = PINOUT_NONE;
  }

  *length = size;
  return pinout_val;
}
