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
#include <stdint.h>

#ifdef STM32F303x8
    #define INTERRUPT_COUNT 82
#else
    #define INTERRUPT_COUNT 85
#endif

static void default_handler(void){
    /* TODO: maybe report error ?? */
}

#define DEFAULT_HANDLER(irq_handler) void irq_handler(void) \
    __attribute__((weak, alias("default_handler")))

/* Define default handlers */
DEFAULT_HANDLER(wwdg_handler);
DEFAULT_HANDLER(pvd_handler);
DEFAULT_HANDLER(tamper_stamp_handler);
DEFAULT_HANDLER(rtc_wkup_handler);
DEFAULT_HANDLER(flash_handler);
DEFAULT_HANDLER(rcc_handler);
DEFAULT_HANDLER(exti0_handler);
DEFAULT_HANDLER(exit1_handler);
DEFAULT_HANDLER(exti2_ts_handler);
DEFAULT_HANDLER(exit3_handler);
DEFAULT_HANDLER(exit4_handler);
DEFAULT_HANDLER(dma1_channel1_handler);
DEFAULT_HANDLER(dma1_channel2_handler);
DEFAULT_HANDLER(dma1_channel3_handler);
DEFAULT_HANDLER(dma1_channel4_handler);
DEFAULT_HANDLER(dma1_channel5_handler);
DEFAULT_HANDLER(dma1_channel6_handler);
DEFAULT_HANDLER(dma1_channel7_handler);
DEFAULT_HANDLER(adc12_handler);
DEFAULT_HANDLER(can_tx_handler);
DEFAULT_HANDLER(can_rx0_handler);
DEFAULT_HANDLER(can_rx1_handler);
DEFAULT_HANDLER(can_sce_handler);
DEFAULT_HANDLER(exti9_5_handler);
DEFAULT_HANDLER(tim1_brk_tim15_handler);
DEFAULT_HANDLER(tim1_up_tim16_handler);
DEFAULT_HANDLER(tim1_trg_com_tim17_handler);
DEFAULT_HANDLER(tim1_cc_handler);
DEFAULT_HANDLER(tim2_handler);
DEFAULT_HANDLER(tim3_handler);
DEFAULT_HANDLER(tim4_handler);
DEFAULT_HANDLER(i2c1_ev_handler);
DEFAULT_HANDLER(i2c1_er_handler);
DEFAULT_HANDLER(i2c2_ev_handler);
DEFAULT_HANDLER(i2c2_er_handler);
DEFAULT_HANDLER(spi1_handler);
DEFAULT_HANDLER(spi2_handler);
DEFAULT_HANDLER(usart1_handler);
DEFAULT_HANDLER(usart2_handler);
DEFAULT_HANDLER(usart3_handler);
DEFAULT_HANDLER(exti15_10_handler);
DEFAULT_HANDLER(rtc_alarm_handler);
DEFAULT_HANDLER(usb_wake_up_handler);
DEFAULT_HANDLER(tim8_brk_handler);
DEFAULT_HANDLER(tim8_up_handler);
DEFAULT_HANDLER(tim8_trg_com_handler);
DEFAULT_HANDLER(tim8_cc_handler);
DEFAULT_HANDLER(adc3_handler);
DEFAULT_HANDLER(fmc_handler);
DEFAULT_HANDLER(spi3_handler);
DEFAULT_HANDLER(uart4_handler);
DEFAULT_HANDLER(uart5_handler);
DEFAULT_HANDLER(tim6_dac_handler);
DEFAULT_HANDLER(tim7_handler);
DEFAULT_HANDLER(dma2_channel1_handler);
DEFAULT_HANDLER(dma2_channel2_handler);
DEFAULT_HANDLER(dma2_channel3_handler);
DEFAULT_HANDLER(dma2_channel4_handler);
DEFAULT_HANDLER(dma2_channel5_handler);
DEFAULT_HANDLER(adc4_handler);
DEFAULT_HANDLER(comp123_handler);
DEFAULT_HANDLER(comp456_handler);
DEFAULT_HANDLER(comp7_handler);
DEFAULT_HANDLER(i2c3_ev_handler);
DEFAULT_HANDLER(i2c3_er_handler);
DEFAULT_HANDLER(usb_hp_handler);
DEFAULT_HANDLER(usb_lp_handler);
DEFAULT_HANDLER(tim20_brk_handler);
DEFAULT_HANDLER(tim20_up_handler);
DEFAULT_HANDLER(tim20_trg_com_handler);
DEFAULT_HANDLER(tim20_cc_handler);
DEFAULT_HANDLER(fpu_handler);
DEFAULT_HANDLER(spi4_handler);

void* the_nvic_vector_device[INTERRUPT_COUNT]
__attribute__ ((section(".nvic_vector_device")))= {
    (void*)wwdg_handler,
    (void*)pvd_handler,
    (void*)tamper_stamp_handler,
    (void*)rtc_wkup_handler,
    (void*)flash_handler,
    (void*)rcc_handler,
    (void*)exti0_handler,
    (void*)exit1_handler,
    (void*)exti2_ts_handler,
    (void*)exit3_handler,
    (void*)exit4_handler,

    (void*)dma1_channel1_handler,
    (void*)dma1_channel2_handler,
    (void*)dma1_channel3_handler,
    (void*)dma1_channel4_handler,
    (void*)dma1_channel5_handler,
    (void*)dma1_channel6_handler,
    (void*)dma1_channel7_handler,

    (void*)adc12_handler,
    (void*)can_tx_handler,
    (void*)can_rx0_handler,
    (void*)can_rx1_handler,
    (void*)can_sce_handler,

    (void*)exti9_5_handler,
    (void*)tim1_brk_tim15_handler,
    (void*)tim1_up_tim16_handler,
    (void*)tim1_trg_com_tim17_handler,
    (void*)tim1_cc_handler,
    (void*)tim2_handler,
    (void*)tim3_handler,
    (void*)tim4_handler,

    (void*)i2c1_ev_handler,
    (void*)i2c1_er_handler,
    (void*)i2c2_ev_handler,
    (void*)i2c2_er_handler,

    (void*)spi1_handler,
    (void*)spi2_handler,

    (void*)usart1_handler,
    (void*)usart2_handler,
    (void*)usart3_handler,

    (void*)exti15_10_handler,
    (void*)rtc_alarm_handler,
    (void*)usb_wake_up_handler,
    (void*)tim8_brk_handler,
    (void*)tim8_up_handler,
    (void*)tim8_trg_com_handler,
    (void*)tim8_cc_handler,
    (void*)adc3_handler,
    (void*)fmc_handler,
    0,0,    /* Reserved */
    (void*)spi3_handler,
    (void*)uart4_handler,
    (void*)uart5_handler,
    (void*)tim6_dac_handler,
    (void*)tim7_handler,

    (void*)dma2_channel1_handler,
    (void*)dma2_channel2_handler,
    (void*)dma2_channel3_handler,
    (void*)dma2_channel4_handler,
    (void*)dma2_channel5_handler,

    (void*)adc4_handler,
    (void*)0,(void*)0,    /* Reserved */
    (void*)comp123_handler,
    (void*)comp456_handler,
    (void*)comp7_handler,
    (void*)0,(void*)0,(void*)0,(void*)0,(void*)0, /* Reserved */

    (void*)i2c3_ev_handler,
    (void*)i2c3_er_handler,
    (void*)usb_hp_handler,
    (void*)usb_lp_handler,
    (void*)usb_wake_up_handler,

    (void*)tim20_brk_handler,
    (void*)tim20_up_handler,
    (void*)tim20_trg_com_handler,
    (void*)tim20_cc_handler,
    (void*)fpu_handler,
#ifndef STM32F303x8
    0,0, /* Reserved */
    (void*)spi4_handler,
#endif
};
