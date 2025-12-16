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
#include "dac.h"
#include "pwm.h"
#include "pwm_input.h"
#include "voltmeter.h"
#include "oscilloscope.h"
#include "generator.h"
#include "stm32_dma.h"

/* TODO: set list of unused pins
    - this is demo torwards 8-pin package => no unused pins on 8-pin pacakge */
const gpio_pin_t gpio_unused_pin_range[1] = {0};

#define UART_RX_BUFFER_SIZE     (512)

static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

static comm_t main_uart;
static uart_handle_t main_uart_handle;
static int main_uart_initialized = 0;
static const uart_init_t main_uart_params = {
    115200,
    UART_PARITY_NONE,
    DEFINE_PIN(GPIOA_BASE,9),/* TX pin */
    DEFINE_PIN(GPIOA_BASE,10) /* RX pin */,
    0,
    UART_RX_BUFFER_SIZE
};

int dma_watermark;

comm_t *get_main_comm(void){
    if(!main_uart_initialized){
        uart_init(&main_uart_handle,1,&main_uart_params,uart_rx_buffer);
        main_uart.hw_handle = &main_uart_handle;
        main_uart.write_callback = (comm_write_callback_t)uart_send_data;
        main_uart.read_callback = (comm_read_callback_t)uart_receive_data;
        main_uart_initialized = 1;
    }
    return &main_uart;
}

VOLTMETER_DECLARE_HINT(volt1, 12, 4, VOLTMETER_DEFAULT_HINT);
PWM_DECLARE(pwm1, DEFINE_PIN(GPIOB_BASE, 6));
OSC_DECLARE_HINT(osc1, 12, 3, 12*1024, OSC_DEFAULT_HINT);
PWM_IN_DECLARE(pwm_in1, DEFINE_PIN(GPIOA_BASE,0));
GEN_DECLARE(gen1, 1000000, 1, 512);
DAC_OUPUT_DECLARE(gen_dc,2);

uint32_t current_config = 0;

const uint8_t* get_target_pinout(uint16_t* length){
  static uint8_t pinout_val[PINOUT_SIZE(15)];
  int size = 1;
  uint8_t af = ((current_config & 0x1) == 0)?PINOUT_VOLT:PINOUT_OSC;

  pinout_val [0] = PINOUT_TSSOP20;

  PINOUT_ADD_SYS (pinout_val, size, 4, PINOUT_VDD);
  PINOUT_ADD_SYS (pinout_val, size, 5, PINOUT_GND);
  PINOUT_ADD_SYS (pinout_val, size, 6, PINOUT_NRST);
  PINOUT_ADD_SPEC(pinout_val, size, 7, 'A', 0, PINOUT_PWM_IN, 0);
  PINOUT_ADD_SPEC(pinout_val, size, 8, 'A', 1, af, 0);
  PINOUT_ADD_SPEC(pinout_val, size, 9, 'A', 2, af, 1);
  PINOUT_ADD_SPEC(pinout_val, size,10, 'A', 3, af, 2);
  PINOUT_ADD_SPEC(pinout_val, size,11, 'A', 4, PINOUT_GEN, 0);
  PINOUT_ADD_SPEC(pinout_val, size,12, 'A', 5, PINOUT_GEN, 1);
  PINOUT_ADD_CORE(pinout_val, size,16, 'A', 9, PINOUT_UART_TX);
  PINOUT_ADD_CORE(pinout_val, size,17, 'A',10, PINOUT_UART_RX);
  PINOUT_ADD_CORE(pinout_val, size,18, 'A',13, PINOUT_SWDIO);
  PINOUT_ADD_CORE(pinout_val, size,19, 'A',14, PINOUT_SWCLK);
  PINOUT_ADD_CORE(pinout_val, size,19, 'A',14, PINOUT_BOOT0);
  PINOUT_ADD_SPEC(pinout_val, size,20, 'B', 6, PINOUT_PWM, 0);

  *length = size;
  return pinout_val;
}

uint32_t vdda_value = 3300;

uint32_t get_vdda(void){
    return vdda_value;
}

gpio_pin_t toggle_pin = DEFINE_PIN(GPIOB_BASE, 6);

void target_init(){
    int i;

    /* Remap PA pins */
    SYSCFG->CFGR1 |= 0x18;

    stm32_common_init();
    stm32_gpio_init(toggle_pin, MODE_OUT_PP);
    for(i = 0;i < 6;++i){
        gpio_toggle(toggle_pin);
        wait_ms(100);
    }
    stm32_gpio_init(toggle_pin, MODE_IN);
}

uint32_t get_target_capabilities(){
  if((current_config & 0x1) == 0){
    return 0x3C;
  }
  else {
    return 0x37;
  }
}

static void init_common(){
    PWM_MODULE_INIT(pwm1,16);
    PWM_IN_MODULE_INIT(pwm_in1, 2);
    GEN_MODULE_INIT_SIMPLE(gen1,1);
    gen1_module->ignoreDCComands();
    DAC_OUTPUT_INIT(gen_dc,1);
}

static void init_voltmeter_variant(){
    init_common();

    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,1),0);  //CH2
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,2),1);  //CH1
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,3),2);  //CH0

    VOLTMETER_SET_REFCHANNEL(volt1,13);                       //CH13
    VOLTMETER_MODULE_INIT(volt1, 1);
//    volt1_module->setVREFIndex(0);
}
static void init_oscilloscope_variant(){
    init_common();

    /* NOTE: TIM1 & TIM3 should be used by oscilloscope */
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,1),0);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,2),1);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,3),2);
    
    OSC_MODULE_PREPARE(osc1,1);

    OSC_MODULE_INIT(osc1);
    OSC_MODULE_LIMIT_FREQUENCY(osc1,2000000);
    OSC_MODULE_LIMIT_FREQUENCY_SCALE(osc1, true);
}

void set_next_device_configuration(){
    current_config = (1+current_config) & 0x1;
    Module::removeAll();
    OSC_MODULE_DEINIT(osc1);
    mem_free();
    timer_unlock_all();
    stm32_dma_alloc_set_watermark(dma_watermark);

    if((current_config & 0x1) == 0){
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

