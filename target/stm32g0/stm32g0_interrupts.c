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

#define INTERRUPT_COUNT 32

static void default_handler(void){
    /* TODO: maybe report error ?? */
}

#define DEFAULT_HANDLER(irq_handler) void irq_handler(void) \
    __attribute__((weak, alias("default_handler")))

/* Define default handlers */
DEFAULT_HANDLER(wwdg_handler);
DEFAULT_HANDLER(pvd_handler);

DEFAULT_HANDLER(rtc_wkup_handler);
DEFAULT_HANDLER(flash_handler);
DEFAULT_HANDLER(rcc_crs_handler);
DEFAULT_HANDLER(exti0_1_handler);
DEFAULT_HANDLER(exit2_3_handler);
DEFAULT_HANDLER(exit4_15_handler);

DEFAULT_HANDLER(usb_handler);
DEFAULT_HANDLER(dma1_channel1_handler);
DEFAULT_HANDLER(dma1_channel2_3_handler);
DEFAULT_HANDLER(dma1_channel4_7_handler);
DEFAULT_HANDLER(adc_comp_handler);
DEFAULT_HANDLER(tim1_brk_up_trg_com_handler);
DEFAULT_HANDLER(tim1_cc_handler);
DEFAULT_HANDLER(tim2_handler);
DEFAULT_HANDLER(tim3_handler);
DEFAULT_HANDLER(tim6_handler);
DEFAULT_HANDLER(tim7_handler);
DEFAULT_HANDLER(tim14_handler);
DEFAULT_HANDLER(tim15_handler);
DEFAULT_HANDLER(tim16_handler);
DEFAULT_HANDLER(tim17_handler);
DEFAULT_HANDLER(i2c1_handler);
DEFAULT_HANDLER(i2c2_handler);
DEFAULT_HANDLER(spi1_handler);
DEFAULT_HANDLER(spi2_handler);
DEFAULT_HANDLER(usart1_handler);
DEFAULT_HANDLER(usart2_handler);
DEFAULT_HANDLER(usart3_8_handler);
DEFAULT_HANDLER(cec_handler);
DEFAULT_HANDLER(aes_rng_handler);

void* the_nvic_vector_device[INTERRUPT_COUNT]
__attribute__ ((section(".nvic_vector_device")))= {
    (void*)wwdg_handler,
    (void*)pvd_handler,
    (void*)rtc_wkup_handler,
    (void*)flash_handler,
    (void*)rcc_crs_handler,
    (void*)exti0_1_handler,
    (void*)exit2_3_handler,
    (void*)exit4_15_handler,
    (void*)usb_handler,

    (void*)dma1_channel1_handler,
    (void*)dma1_channel2_3_handler,
    (void*)dma1_channel4_7_handler,

    (void*)adc_comp_handler,
    (void*)tim1_brk_up_trg_com_handler,
    (void*)tim1_cc_handler,
    (void*)tim2_handler,
    (void*)tim3_handler,
    (void*)tim6_handler,
    (void*)tim7_handler,
    (void*)tim14_handler,
    (void*)tim15_handler,
    (void*)tim16_handler,
    (void*)tim17_handler,

    (void*)i2c1_handler,
    (void*)i2c2_handler,

    (void*)spi1_handler,
    (void*)spi2_handler,

    (void*)usart1_handler,
    (void*)usart2_handler,
    (void*)usart3_8_handler,

    (void*)cec_handler,
    (void*)aes_rng_handler,
};
