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

static uint32_t core_clock = 16000000;

int init_cpu(void){
    int tries = 200;
    int status = 0;
    uint32_t tmpreg;

    int use_hse = 0;
#ifdef TARGET_USE_HSE
    use_hse = 1;
#endif

    if(use_hse){
        #ifdef TARGET_HSE_BYP
            RCC->CR |= RCC_CR_HSEBYP;
        #else
            RCC->CR &= ~(uint32_t)RCC_CR_HSEBYP;
        #endif
        RCC->CR |= RCC_CR_HSEON;

        while(tries-- || (RCC->CR & RCC_CR_HSERDY) == 0){
        }

        if((RCC->CR & RCC_CR_HSERDY) == 0){
            /* Fall back to internal oscillator */
            use_hse = 0;
            status = -2;
        }
    }

    /* TODO: compute PLL parameters */
    /* M = 8, N = 336, P = 2, Q = 7 */
    tmpreg = (8 << 0) | (336 << 6) | (7 << 24);
    if(use_hse) tmpreg |= RCC_PLLCFGR_PLLSRC_HSE;
    RCC->PLLCFGR = tmpreg;

    RCC->CR |= RCC_CR_PLLON;

    /* Set AHB & APB prescalers */
    tmpreg = RCC->CFGR;
    tmpreg &= ~(uint32_t)0xFCF0;
    tmpreg |= 0x9400;
    RCC->CFGR |= tmpreg;

    tries = 200;
    while(tries-- || (RCC->CR & RCC_CR_PLLRDY) == 0){
    }

    if((RCC->CR & RCC_CR_PLLRDY) == 0){
        return -1;
    }
    else {
        /* Set flash latency for higher core clock */
        tmpreg = FLASH->ACR;
        tmpreg &= ~FLASH_ACR_LATENCY;
        tmpreg |= FLASH_ACR_LATENCY_5WS;
        FLASH->ACR |= tmpreg;

        /* Switch to PLL clocks */
        tmpreg = RCC->CFGR;
        tmpreg &= ~RCC_CFGR_SW;
        tmpreg |= RCC_CFGR_SW_PLL;
        RCC->CFGR = tmpreg;

        core_clock = 168000000;
    }
    return status;
}

uint32_t get_core_clock(){
    return core_clock;
}
