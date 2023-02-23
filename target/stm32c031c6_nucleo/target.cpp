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
#include "stm32_dma.h"

/* TODO: set list of unused pins
    - this is demo torwards 8-pin package => no unused pins on 8-pin pacakge */
const gpio_pin_t gpio_unused_pin_range[1] = {0};

#define UART_RX_BUFFER_SIZE     (1500)

static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

static comm_t main_uart;
static uart_handle_t main_uart_handle;
static int main_uart_initialized = 0;
static const uart_init_t main_uart_params = {
    115200,
    UART_PARITY_NONE,
    DEFINE_PIN(GPIOA_BASE,2),/* TX pin */
    DEFINE_PIN(GPIOA_BASE,3) /* RX pin */,
    0,
    UART_RX_BUFFER_SIZE
};

int dma_watermark;

comm_t *get_main_comm(void){
    if(!main_uart_initialized){
        uart_init(&main_uart_handle,2,&main_uart_params,uart_rx_buffer);
        main_uart.hw_handle = &main_uart_handle;
        main_uart.write_callback = (comm_write_callback_t)uart_send_data;
        main_uart.read_callback = (comm_read_callback_t)uart_receive_data;
        main_uart_initialized = 1;
    }
    return &main_uart;
}

VOLTMETER_DECLARE(volt1, 12, 4);
PWM_DECLARE(pwm1, DEFINE_PIN(GPIOA_BASE, 7));
OSC_DECLARE(osc1, 12, 4, 2*1024);

uint32_t vdda_value = 3300;

uint32_t get_vdda(void){
    return vdda_value;
}

void target_init(){
    stm32_common_init();
}

uint32_t current_config = 0;

uint32_t get_target_capabilities(){
  if(current_config == 0){
    return 0x0C;
  }
  else {
    return 0x07;
  }
}

static void init_common(){
    PWM_MODULE_INIT(pwm1,14);
}

static void init_voltmeter_variant(){
    init_common();

    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,0),0);
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,1),1);
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,4),2);
    VOLTMETER_SET_REFCHANNEL(volt1,10);

    VOLTMETER_MODULE_INIT(volt1, 1);
}

static void init_oscilloscope_variant(){
    init_common();

    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,0),0);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,1),1);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,4),2);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOB_BASE,1),3);

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
