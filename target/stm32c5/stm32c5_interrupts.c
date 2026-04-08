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
#include "arm_interrupts.h"

#define INTERRUPT_COUNT 99

static void default_handler(void){
    /* TODO: maybe report error ?? */
}

#define DEFAULT_HANDLER(irq_handler) void irq_handler(void) \
    __attribute__((weak, alias("default_handler")))


DEFAULT_HANDLER(wwdg_handler);
DEFAULT_HANDLER(pvd_handler);
DEFAULT_HANDLER(rtc_wkup_handler);
DEFAULT_HANDLER(tamp_handler);
DEFAULT_HANDLER(ramcfg_handler);
DEFAULT_HANDLER(flash_handler);
DEFAULT_HANDLER(rcc_handler);
DEFAULT_HANDLER(exti0_handler);
DEFAULT_HANDLER(exti1_handler);
DEFAULT_HANDLER(exti2_handler);
DEFAULT_HANDLER(exti3_handler);
DEFAULT_HANDLER(exti4_handler);
DEFAULT_HANDLER(exti5_handler);
DEFAULT_HANDLER(exti6_handler);
DEFAULT_HANDLER(exti7_handler);
DEFAULT_HANDLER(exti8_handler);
DEFAULT_HANDLER(exti9_handler);
DEFAULT_HANDLER(exti10_handler);
DEFAULT_HANDLER(exti11_handler);
DEFAULT_HANDLER(exti12_handler);
DEFAULT_HANDLER(exti13_handler);
DEFAULT_HANDLER(exti14_handler);
DEFAULT_HANDLER(exti15_handler);
DEFAULT_HANDLER(gpdma1_channel0_handler);
DEFAULT_HANDLER(gpdma1_channel1_handler);
DEFAULT_HANDLER(gpdma1_channel2_handler);
DEFAULT_HANDLER(gpdma1_channel3_handler);
DEFAULT_HANDLER(gpdma1_channel4_handler);
DEFAULT_HANDLER(gpdma1_channel5_handler);
DEFAULT_HANDLER(gpdma1_channel6_handler);
DEFAULT_HANDLER(gpdma1_channel7_handler);
DEFAULT_HANDLER(iwdg_handler);
DEFAULT_HANDLER(adc1_handler);
DEFAULT_HANDLER(adc2_handler);
DEFAULT_HANDLER(fdcan1_handler0);
DEFAULT_HANDLER(fdcan1_handler1);
DEFAULT_HANDLER(tim1_brk_handler);
DEFAULT_HANDLER(tim1_up_handler);
DEFAULT_HANDLER(tim1_trg_com_handler);
DEFAULT_HANDLER(tim1_cc_handler);
DEFAULT_HANDLER(tim2_handler);
DEFAULT_HANDLER(tim5_handler);
DEFAULT_HANDLER(tim6_handler);
DEFAULT_HANDLER(tim7_handler);
DEFAULT_HANDLER(i2c1_ev_handler);
DEFAULT_HANDLER(i2c1_er_handler);
DEFAULT_HANDLER(i3c1_ev_handler);
DEFAULT_HANDLER(i3c1_er_handler);
DEFAULT_HANDLER(spi1_handler);
DEFAULT_HANDLER(spi2_handler);
DEFAULT_HANDLER(spi3_handler);
DEFAULT_HANDLER(uart1_handler);
DEFAULT_HANDLER(uart2_handler);
DEFAULT_HANDLER(uart3_handler);
DEFAULT_HANDLER(uart4_handler);
DEFAULT_HANDLER(uart5_handler);
DEFAULT_HANDLER(lpuart1_handler);
DEFAULT_HANDLER(lptim1_handler);
DEFAULT_HANDLER(tim12_handler);
DEFAULT_HANDLER(tim15_handler);
DEFAULT_HANDLER(tim16_handler);
DEFAULT_HANDLER(tim17_handler);
DEFAULT_HANDLER(usb_handler);
DEFAULT_HANDLER(crs_handler);
DEFAULT_HANDLER(rng_handler);
DEFAULT_HANDLER(fpu_handler);
DEFAULT_HANDLER(icache_handler);
DEFAULT_HANDLER(cordic_handler);
DEFAULT_HANDLER(aes_handler);
DEFAULT_HANDLER(hash_handler);
DEFAULT_HANDLER(i2c2_ev_handler);
DEFAULT_HANDLER(i2c2_er_handler);
DEFAULT_HANDLER(tim8_brk_handler);
DEFAULT_HANDLER(tim8_up_handler);
DEFAULT_HANDLER(tim8_trg_com_handler);
DEFAULT_HANDLER(tim8_cc_handler);
DEFAULT_HANDLER(comp1_handler);
DEFAULT_HANDLER(dac1_handler);
DEFAULT_HANDLER(gpdma2_channel0_handler);
DEFAULT_HANDLER(gpdma2_channel1_handler);
DEFAULT_HANDLER(gpdma2_channel2_handler);
DEFAULT_HANDLER(gpdma2_channel3_handler);
DEFAULT_HANDLER(gpdma2_channel4_handler);
DEFAULT_HANDLER(gpdma2_channel5_handler);
DEFAULT_HANDLER(gpdma2_channel6_handler);
DEFAULT_HANDLER(gpdma2_channel7_handler);
DEFAULT_HANDLER(fdcan2_handler0);
DEFAULT_HANDLER(fdcan2_handler1);
DEFAULT_HANDLER(comp2_handler);
DEFAULT_HANDLER(tim3_handler);
DEFAULT_HANDLER(tim4_handler);
DEFAULT_HANDLER(xspi1_handler);
DEFAULT_HANDLER(saes_handler);
DEFAULT_HANDLER(pka_handler);
DEFAULT_HANDLER(eth1_handler);
DEFAULT_HANDLER(eth1_wkup_handler);
DEFAULT_HANDLER(uart6_handler);
DEFAULT_HANDLER(uart7_handler);
DEFAULT_HANDLER(adc3_handler);


void* the_nvic_vector_device[INTERRUPT_COUNT]
NVIC_VENDOR_ATTRIBUTE = {
    (void*)wwdg_handler,
    (void*)pvd_handler,
    (void*)rtc_wkup_handler,
    (void*)tamp_handler,
    (void*)ramcfg_handler,
    (void*)flash_handler,
    (void*)rcc_handler,
    (void*)exti0_handler,
    (void*)exti1_handler,
    (void*)exti2_handler,
    (void*)exti3_handler,
    (void*)exti4_handler,
    (void*)exti5_handler,
    (void*)exti6_handler,
    (void*)exti7_handler,
    (void*)exti8_handler,
    (void*)exti9_handler,
    (void*)exti10_handler,
    (void*)exti11_handler,
    (void*)exti12_handler,
    (void*)exti13_handler,
    (void*)exti14_handler,
    (void*)exti15_handler,
    (void*)gpdma1_channel0_handler,
    (void*)gpdma1_channel1_handler,
    (void*)gpdma1_channel2_handler,
    (void*)gpdma1_channel3_handler,
    (void*)gpdma1_channel4_handler,
    (void*)gpdma1_channel5_handler,
    (void*)gpdma1_channel6_handler,
    (void*)gpdma1_channel7_handler,
    (void*)iwdg_handler,
    (void*)adc1_handler,
    (void*)adc2_handler,
    (void*)fdcan1_handler0,
    (void*)fdcan1_handler1,
    (void*)tim1_brk_handler,
    (void*)tim1_up_handler,
    (void*)tim1_trg_com_handler,
    (void*)tim1_cc_handler,
    (void*)tim2_handler,
    (void*)tim5_handler,
    (void*)tim6_handler,
    (void*)tim7_handler,
    (void*)i2c1_ev_handler,
    (void*)i2c1_er_handler,
    (void*)i3c1_ev_handler,
    (void*)i3c1_er_handler,
    (void*)spi1_handler,
    (void*)spi2_handler,
    (void*)spi3_handler,
    (void*)uart1_handler,
    (void*)uart2_handler,
    (void*)uart3_handler,
    (void*)uart4_handler,
    (void*)uart5_handler,
    (void*)lpuart1_handler,
    (void*)lptim1_handler,
    (void*)tim12_handler,
    (void*)tim15_handler,
    (void*)tim16_handler,
    (void*)tim17_handler,
    (void*)usb_handler,
    (void*)crs_handler,
    (void*)rng_handler,
    (void*)fpu_handler,
    (void*)icache_handler,
    (void*)cordic_handler,
    (void*)aes_handler,
    (void*)hash_handler,
    (void*)i2c2_ev_handler,
    (void*)i2c2_er_handler,
    (void*)tim8_brk_handler,
    (void*)tim8_up_handler,
    (void*)tim8_trg_com_handler,
    (void*)tim8_cc_handler,
    (void*)comp1_handler,
    (void*)dac1_handler,
    (void*)gpdma2_channel0_handler,
    (void*)gpdma2_channel1_handler,
    (void*)gpdma2_channel2_handler,
    (void*)gpdma2_channel3_handler,
    (void*)gpdma2_channel4_handler,
    (void*)gpdma2_channel5_handler,
    (void*)gpdma2_channel6_handler,
    (void*)gpdma2_channel7_handler,
    (void*)fdcan2_handler0,
    (void*)fdcan2_handler1,
    (void*)comp2_handler,
    (void*)tim3_handler,
    (void*)tim4_handler,
    (void*)xspi1_handler,
    (void*)saes_handler,
    (void*)pka_handler,
    (void*)eth1_handler,
    (void*)eth1_wkup_handler,
    (void*)uart6_handler,
    (void*)uart7_handler,
    (void*)adc3_handler,
};

