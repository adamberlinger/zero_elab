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

VOLTMETER_DECLARE_HINT(volt1, 12, 4, VOLTMETER_DEFAULT_HINT | ADC_HINT_REVERSE_SCAN);
PWM_DECLARE(pwm1, DEFINE_PIN(GPIOB_BASE, 6));
OSC_DECLARE_HINT(osc1, 12, 3, 4*1024, OSC_DEFAULT_HINT | ADC_HINT_REVERSE_SCAN);

uint32_t vdda_value = 3300;

uint32_t get_vdda(void){
    return vdda_value;
}

gpio_pin_t toggle_pin = DEFINE_PIN(GPIOB_BASE, 6);
gpio_pin_t rx_stop_pin = DEFINE_PIN(GPIOA, 10);

void target_init(){
    int i;
    stm32_common_init();
    stm32_gpio_init(toggle_pin, MODE_OUT_PP);
    stm32_gpio_init(rx_stop_pin, MODE_IN);
    stm32_gpio_pullup(rx_stop_pin);
    if(stm32_gpio_read(rx_stop_pin) == 0){
        int keep_lock = 1;
        uint32_t start = get_ms_ticks();
        while((get_ms_ticks() - start) < 100){
            if(stm32_gpio_read(rx_stop_pin)){
                keep_lock = 0;
                break;
            }
        }
        if(keep_lock){
            stm32_gpio_init(toggle_pin, MODE_IN);
            while(1);
        }
    }
    for(i = 0;i < 6;++i){
        gpio_toggle(toggle_pin);
        wait_ms(100);
    }
    stm32_gpio_init(rx_stop_pin, MODE_IN);
    stm32_gpio_init(toggle_pin, MODE_IN);

    /* Configure "unused" pins on multi-bonding */
    gpio_pin_unused( DEFINE_PIN(GPIOA_BASE, 1));
    gpio_pin_unused( DEFINE_PIN(GPIOA_BASE, 2));
    gpio_pin_unused( DEFINE_PIN(GPIOA_BASE, 8));
    gpio_pin_unused( DEFINE_PIN(GPIOA_BASE, 11));
    gpio_pin_unused( DEFINE_PIN(GPIOA_BASE, 12));
    gpio_pin_unused( DEFINE_PIN(GPIOA_BASE, 15));

    
    gpio_pin_unused( DEFINE_PIN(GPIOB_BASE, 0));
    gpio_pin_unused( DEFINE_PIN(GPIOB_BASE, 1));
    gpio_pin_unused( DEFINE_PIN(GPIOB_BASE, 5));
    gpio_pin_unused( DEFINE_PIN(GPIOB_BASE, 8));
    gpio_pin_unused( DEFINE_PIN(GPIOB_BASE, 9));
    
    gpio_pin_unused( DEFINE_PIN(GPIOC_BASE, 14));

    gpio_pin_unused( DEFINE_PIN(GPIOF_BASE, 2));
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
    PWM_MODULE_INIT(pwm1,16);
}

static void init_voltmeter_variant(){
    init_common();

    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,13),0);
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOB_BASE,7),1);
    VOLTMETER_ADD_CHANNEL(volt1,DEFINE_PIN(GPIOA_BASE,0),2);
    VOLTMETER_SET_REFCHANNEL(volt1,13);

    VOLTMETER_MODULE_INIT(volt1, 1);
    volt1_module->setVREFIndex(1);
}
static void init_oscilloscope_variant(){
    init_common();

    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,13),0);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOB_BASE,7),1);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOA_BASE,0),2);
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
