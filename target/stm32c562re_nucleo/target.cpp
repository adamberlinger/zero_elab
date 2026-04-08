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
//#include "board_pindefs.h"

//#include "adc.h"
#include "pwm.h"
#include "generator.h"
#include "dac.h"
//#include "pwm_input.h"
//#include "voltmeter.h"
//#include "oscilloscope.h"
//#include "pwm_generator.h"
#include "stm32_dma.h"

/* TODO: set list of unused pins
    - this is demo torwards 8-pin package => no unused pins on 8-pin pacakge */
const gpio_pin_t gpio_unused_pin_range[1] = {0};

int dma_watermark;

/* PA5 -> D13 */
//PWM_DECLARE(pwm1, DEFINE_PIN(GPIOA_BASE, 5));
GEN_DECLARE(gen1, 1000000, 1, 512);

uint32_t vdda_value = 3300;

uint32_t get_vdda(void){
    return vdda_value;
}

void target_init(){
    stm32_common_init();
}

uint32_t current_config = 0;

uint32_t get_target_capabilities(){
  return 0x0;
}

static void init_common(){
    //PWM_MODULE_INIT(pwm1,2);
    GEN_MODULE_INIT_SIMPLE(gen1,1);
}

static void init_voltmeter_variant(){
    init_common();
    /*init_generator();

    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,0),0);
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,1),1);
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,4),2);
    VOLTMETER_SET_REFCHANNEL(volt1,10);

    VOLTMETER_MODULE_INIT(volt1, 1);*/
}


static void init_oscilloscope_variant(){
    init_common();
//    init_generator();

    /* AN0 - AN3 on arduino header */
    /*OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,0),0);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,1),1);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,4),2);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOB_BASE,1),3);

    OSC_MODULE_PREPARE(osc1,1);

    OSC_MODULE_INIT(osc1);
    OSC_MODULE_LIMIT_FREQUENCY(osc1,200000);*/
}

void set_next_device_configuration(){
}

void functions_init(void){
    dma_watermark = stm32_dma_alloc_get_watermark();

    target_configuration_name = "Voltmeter";

    init_voltmeter_variant();

}

const uint8_t* get_target_pinout(uint16_t* length){
  static uint8_t pinout_val[PINOUT_SIZE(16)];
  int size = 1;
  uint8_t af = ((current_config & 0x1) == 0)?PINOUT_VOLT:PINOUT_OSC;

  pinout_val[0] = PINOUT_ARDUINO;

  PINOUT_ADD_SPEC(pinout_val, size,   0, 'A', 0, af, 0);
  PINOUT_ADD_SPEC(pinout_val, size,   1, 'A', 1, af, 1);
  PINOUT_ADD_SPEC(pinout_val, size,   2, 'A', 4, af, 2);
  if(current_config & 0x1){
    PINOUT_ADD_SPEC(pinout_val, size,   3, 'B', 1, af, 3);
  }
  PINOUT_ADD_SPEC(pinout_val, size,6+ 0, 'B', 7, PINOUT_PWM, 0);
  PINOUT_ADD_SPEC(pinout_val, size,6+ 1, 'B', 6, PINOUT_GEN, 0);

  *length = size;
  return pinout_val;
}

