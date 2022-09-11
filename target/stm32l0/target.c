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
#include "target.h"
#include "stm32_common.h"
#include "uart.h"
#include "comm.h"

static uint32_t core_clock = 8000000;

/* NOTE: currently works for series with internal 48MHz oscillator */
int init_cpu(void){
    int tries = 200;
    int status = 0;
    uint32_t tmpreg;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* Set voltage scale 1 */
    while(PWR->CSR & PWR_CSR_VOSF);
    PWR->CR = (PWR->CR & ~(uint32_t)0x1800) | (0x800);
    while(PWR->CSR & PWR_CSR_VOSF);

    RCC->CR |= RCC_CR_HSION;
    while(tries && (RCC->CR & RCC_CR_HSIRDY) == 0){
        tries--;
    }

    if((RCC->CR & RCC_CR_HSIRDY) == 0){
        /* Fall back to internal 8MHz oscillator */
        status = -2;
    }
    else {
        /* Configure PLL = HSI16 x2 */
        RCC->CFGR = (RCC->CFGR & 0xFF00FFFF) | (1 << 18) | (1 << 22);

        RCC->CR |= RCC_CR_PLLON;

        while(tries && (RCC->CR & RCC_CR_PLLRDY) == 0) {
            tries--;
        }

        if((RCC->CR & RCC_CR_PLLRDY) == 0){
            status = -2;
        }
        else {
            /* Set flash latency for higher core clock */
            tmpreg = FLASH->ACR;
            tmpreg &= ~FLASH_ACR_LATENCY;
            /* Set 1 wait-state */
            tmpreg |= (1);
            FLASH->ACR |= tmpreg;

            /* Switch to HSI48 clock */
            tmpreg = RCC->CFGR;
            tmpreg &= ~RCC_CFGR_SW;
            tmpreg |= RCC_CFGR_SW_PLL;
            RCC->CFGR = tmpreg;

            core_clock = 32000000;
        }
    }
    return status;
}

uint32_t get_core_clock(){
    return core_clock;
}

void stm32_target_clken(GPIO_TypeDef* port){
    if(port == GPIOA) RCC->IOPENR |= RCC_IOPENR_IOPAEN;
#ifdef GPIOB
    if(port == GPIOB) RCC->IOPENR |= RCC_IOPENR_IOPBEN;
#endif
#ifdef GPIOC
    if(port == GPIOC) RCC->IOPENR |= RCC_IOPENR_IOPCEN;
#endif
#ifdef GPIOD
    if(port == GPIOD) RCC->IOPENR |= RCC_IOPENR_IOPDEN;
#endif
#ifdef GPIOE
    if(port == GPIOE) RCC->IOPENR |= RCC_IOPENR_IOPEEN;
#endif
#ifdef GPIOH
    if(port == GPIOH) RCC->IOPENR |= RCC_IOPENR_IOPHEN;
#endif
}
