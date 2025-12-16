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

/* List of unconnected pins to be set to floating-input */
const gpio_pin_t gpio_unused_pin_range[GPIO_UNUSED_PIN_RANGE_SIZE] = {
        DEFINE_PIN(GPIOA_BASE,0), DEFINE_PIN(GPIOA_BASE,10),   /* PA0 - PA10 */
        DEFINE_PIN(GPIOA_BASE,13), DEFINE_PIN(GPIOA_BASE,15),  /* PA13 - PA15 */
        DEFINE_PIN(GPIOB_BASE,0), DEFINE_PIN(GPIOB_BASE,1),   /* PB0 - PB1 */
        DEFINE_PIN(GPIOB_BASE,3), DEFINE_PIN(GPIOB_BASE,7),   /* PB3 - PB7 */
};

OSC_DECLARE(osc1, 12, 3, (1024+512));
OSC_DECLARE(osc2, 12, 3, (2048+512));
VOLTMETER_DECLARE(volt1, 12, 4);

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
PWM_DECLARE(pwm1, DEFINE_PIN(GPIOB_BASE, 0));
#endif

uint32_t vdda_value = 3300;

gpio_pin_t toggle_pin = DEFINE_PIN(GPIOA_BASE, 4);

uint32_t get_vdda(void){
    return vdda_value;
}

void target_init(){
    int i;
    stm32_common_init();
    stm32_gpio_init(toggle_pin, MODE_OUT_PP);
    for(i = 0;i < 6;++i){
        gpio_toggle(toggle_pin);
        wait_ms(100);
    }
    stm32_gpio_init(toggle_pin, MODE_IN);
}

#ifdef USE_GENERATOR
static timer_handle_t gen_tim_handle;
#endif

static void init_timer_or_generator(){
#ifdef USE_GENERATOR
    module_t* gen_module = app_malloc(sizeof(module_t));
    gen_make_module(gen_module,app_malloc(sizeof(gen_handle_t)));

    timer_init(&gen_tim_handle,3,&pwm_init_data);

    gen_init((gen_handle_t*)gen_module->handle,MODULE_CHANNEL_GEN,
        gen_tim_handle.circular_buffer,10000,512+256,8);
    module_register(gen_module);
#else
    PWM_MODULE_INIT(pwm1,3);
#endif
}

static void init_voltmeter_variant(){
    init_timer_or_generator();
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,0),0);
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,1),1);
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,7),2);
    VOLTMETER_SET_REFCHANNEL(volt1,17);

    VOLTMETER_MODULE_INIT(volt1, 1);
}

static void init_analyzer_variant(){
    additional_buffer_t* log_buffer;
    init_timer_or_generator();


    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,1), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,3), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,4), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,5), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,6), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,7), MODE_IN);

    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,1));
    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,3));
    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,4));
    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,5));
    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,6));
    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,7));

    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,0),0);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,1),1);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,7),2);

    OSC_MODULE_PREPARE(osc1,1);

    log_buffer = buffer_additional_alloc(OSC_MODULE_GET_BUFFER(osc1),512+256);
    timer_target_configure_custom_dma(osc1_adc_handle->getTimer(),(void*)&GPIOB->IDR,log_buffer);

    OSC_MODULE_INIT(osc1);
    OSC_MODULE_LIMIT_FREQUENCY(osc1,600000);
}

static void init_oscilloscope_variant(){
    additional_buffer_t* log_buffer;
    init_timer_or_generator();


    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,1), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,3), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,4), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,5), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,6), MODE_IN);
    stm32_gpio_init(DEFINE_PIN(GPIOB_BASE,7), MODE_IN);

    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,1));
    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,3));
    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,4));
    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,5));
    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,6));
    stm32_gpio_pulldown(DEFINE_PIN(GPIOB_BASE,7));

    OSC_ADD_CHANNEL(osc2,DEFINE_PIN(GPIOA_BASE,0),0);
    OSC_ADD_CHANNEL(osc2,DEFINE_PIN(GPIOA_BASE,1),1);
    OSC_ADD_CHANNEL(osc2,DEFINE_PIN(GPIOA_BASE,7),2);

    OSC_MODULE_PREPARE(osc2,1);

    OSC_MODULE_INIT(osc2);
    OSC_MODULE_LIMIT_FREQUENCY(osc2,600000);
}

uint32_t current_config = 0;

uint32_t get_target_capabilities(){
    if(current_config > 0){
        return 0x7;
    }
    else {
        return 0xD;
    }
}

void set_next_device_configuration(){
    current_config = (1+current_config) % 3;
    Module::removeAll();
    OSC_MODULE_DEINIT(osc1);
    mem_free();
    timer_unlock_all();
    if(current_config == 0){
        target_configuration_name = "Voltmeter";
        init_voltmeter_variant();
    }
    else if(current_config == 1) {
        target_configuration_name = "Oscilloscope";
        init_oscilloscope_variant();
    }
    else if(current_config == 2){
        target_configuration_name = "Oscilloscope + Log. analyzer";
        init_analyzer_variant();
    }
}

void functions_init(void){
    target_configuration_name = "Voltmeter";
    init_voltmeter_variant();
    vdda_value = volt1_module->getVDDA();
}

const uint8_t* get_target_pinout(uint16_t* length){
  static uint8_t pinout_val[PINOUT_SIZE(20)];
  int size = 1;
  uint8_t af = (current_config == 0)?PINOUT_VOLT:PINOUT_OSC;

  pinout_val [0] = PINOUT_LQFP32;

  PINOUT_ADD_SYS (pinout_val, size, 4, PINOUT_NRST);
  PINOUT_ADD_SYS (pinout_val, size, 5, PINOUT_VDD);
  PINOUT_ADD_SPEC(pinout_val, size, 6, 'A', 0, af, 0);
  PINOUT_ADD_SPEC(pinout_val, size, 7, 'A', 1, af, 1);
  PINOUT_ADD_SPEC(pinout_val, size,13, 'A', 7, af, 2);
#ifdef USE_GENERATOR
  PINOUT_ADD_SPEC(pinout_val, size,14, 'B', 0, PINOUT_GEN, 0);
#else
  PINOUT_ADD_SPEC(pinout_val, size,14, 'B', 0, PINOUT_PWM, 0);
#endif
  if(current_config == 2) {
    PINOUT_ADD_SPEC(pinout_val, size,14, 'B', 0, PINOUT_LOG, 0);
    PINOUT_ADD_SPEC(pinout_val, size,15, 'B', 1, PINOUT_LOG, 1);
  }
  PINOUT_ADD_SYS (pinout_val, size,16, PINOUT_GND);
  PINOUT_ADD_SYS (pinout_val, size,17, PINOUT_VDD);
  PINOUT_ADD_CORE(pinout_val, size,21, 'A',11, PINOUT_USB_DM);
  PINOUT_ADD_CORE(pinout_val, size,22, 'A',12, PINOUT_USB_DP);
  PINOUT_ADD_CORE(pinout_val, size,23, 'A',13, PINOUT_SWDIO);
  PINOUT_ADD_CORE(pinout_val, size,24, 'A',14, PINOUT_SWCLK);
  if(current_config == 2) {
    PINOUT_ADD_SPEC(pinout_val, size,26, 'B', 3, PINOUT_LOG, 3);
    PINOUT_ADD_SPEC(pinout_val, size,27, 'B', 4, PINOUT_LOG, 4);
    PINOUT_ADD_SPEC(pinout_val, size,28, 'B', 5, PINOUT_LOG, 5);
    PINOUT_ADD_SPEC(pinout_val, size,29, 'B', 6, PINOUT_LOG, 6);
    PINOUT_ADD_SPEC(pinout_val, size,30, 'B', 7, PINOUT_LOG, 7);
  }
  PINOUT_ADD_CORE(pinout_val, size,31, 'B', 8, PINOUT_BOOT0);

  *length = size;
  return pinout_val;
}
