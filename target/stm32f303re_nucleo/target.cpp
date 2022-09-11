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
#include "pulse_counter.h"
#include "oscilloscope.h"
#include "generator.h"
#include "voltmeter.h"

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

static uint8_t uart_rx_buffer[256];

static comm_t main_uart;
static uart_handle_t main_uart_handle;
static int main_uart_initialized = 0;
static const uart_init_t main_uart_params = {
    115200,
    UART_PARITY_NONE,
    DEFINE_PIN(GPIOA_BASE,2),/* TX pin */
    DEFINE_PIN(GPIOA_BASE,3) /* RX pin */,
    0,
    256
};

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

OSC_DECLARE(osc1, 12, 2, 1024*6);
GEN_DECLARE(gen1, 1000000, 1, 512);
VOLTMETER_DECLARE(volt1, 12, 3);

PWM_DECLARE(pwm1, PIN_D11);

PWM_IN_DECLARE(pwm_in1, DEFINE_PIN(GPIOB_BASE,14));
PULSE_COUNTER_DECLARE(pc1,DEFINE_PIN(GPIOA_BASE,8));

uint32_t vdda_value = 3300;

uint32_t get_vdda(void){
    return vdda_value;
}

void target_init(){
    stm32_common_init();
}

uint32_t get_target_capabilities(){
    return 0x7F;
}

void functions_init(void){
    PWM_MODULE_INIT(pwm1, 3);

    /* Configure pin for pulse counter */
    PULSE_COUNTER_MODULE_INIT(pc1,4);
    PWM_IN_MODULE_INIT(pwm_in1, 15);

    GEN_MODULE_INIT(gen1,1);

    OSC_ADD_CHANNEL(osc1,PIN_D12,0);
    OSC_ADD_CHANNEL(osc1,DEFINE_PIN(GPIOC_BASE,4),1);

    OSC_MODULE_PREPARE(osc1,2);

    OSC_MODULE_INIT(osc1);

    VOLTMETER_ADD_CHANNEL(volt1,PIN_A0,0);
    VOLTMETER_ADD_CHANNEL(volt1,PIN_A1,1);
    VOLTMETER_SET_REFCHANNEL(volt1,18);

    VOLTMETER_MODULE_INIT(volt1, 1);

    vdda_value = volt1_module->getVDDA();
}
