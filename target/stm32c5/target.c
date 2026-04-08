/*
 * BSD 3-Clause License
 * 
 * Copyright (c) 2026, Adam Berlinger
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
#include "comm.h"

static uint32_t core_clock = 48000000;

int init_cpu(void){
    uint32_t tmpreg;

    RCC->AHB1ENR |= RCC_AHB1ENR_LPDMA1EN | RCC_AHB1ENR_LPDMA2EN;
    RCC->APB3ENR |= RCC_APB3ENR_SBSEN;

    /* Set flash latency for higher core clock */
    tmpreg = FLASH->ACR;
    tmpreg &= ~FLASH_ACR_LATENCY;
    /* Set 4 wait-states */
    tmpreg |= (4) | FLASH_ACR_PRFTEN;
    FLASH->ACR = tmpreg; 

    /* HSIK = HSI / 4 = 36 MHz (max. ADC) */
    RCC->CR2 = 7;
    RCC->CR1 = 0x7;

    while((RCC->CR1 & 0x70) != 0x70);

    /* Set HSIS as system clock */
    RCC->CFGR1 = 0x1;

    while((RCC->CFGR1 & 0x18) != 0x8);

    core_clock = 144000000;

    /* USB = HSI/3, ADC = HSIK */
    RCC->CCIPR2 = 0x02000C00;

    return 0;
}

uint32_t get_core_clock(){
    return core_clock;
}

void stm32_target_clken(GPIO_TypeDef* port){
    if(port == GPIOA) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
#ifdef GPIOB
    if(port == GPIOB) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
#endif
#ifdef GPIOC
    if(port == GPIOC) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
#endif
#ifdef GPIOD
    if(port == GPIOD) RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
#endif
#ifdef GPIOE
    if(port == GPIOE) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
#endif
#ifdef GPIOF
    if(port == GPIOF) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
#endif
#ifdef GPIOG
    if(port == GPIOG) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
#endif
#ifdef GPIOH
    if(port == GPIOH) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
#endif
}
