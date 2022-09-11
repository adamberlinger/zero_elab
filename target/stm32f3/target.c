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

        while(tries && (RCC->CR & RCC_CR_HSERDY) == 0){
            tries--;
        }

        if((RCC->CR & RCC_CR_HSERDY) == 0){
            /* Fall back to internal oscillator */
            use_hse = 0;
            status = -2;
        }
    }

    /* TODO: compute PLL parameters */
    /* TODO: make work for HSE */
    /* PLLMUL = 9*/
    /* Set AHB & APB prescalers */
    if(use_hse){
        tmpreg = RCC_CFGR_PLLMUL9 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLSRC_HSE_PREDIV;
    }
    else {
#ifdef STM32F303x8
        tmpreg = RCC_CFGR_PLLMUL16 | RCC_CFGR_PPRE1_DIV2;
#else
        tmpreg = RCC_CFGR_PLLMUL9 | RCC_CFGR_PPRE1_DIV2;
#endif
    }
    RCC->CFGR = tmpreg;

    RCC->CR |= RCC_CR_PLLON;

    tries = 200;
    while(tries && (RCC->CR & RCC_CR_PLLRDY) == 0){
        tries--;
    }

    if((RCC->CR & RCC_CR_PLLRDY) == 0){
        return -1;
    }
    else {
        /* Set flash latency for higher core clock */
        tmpreg = FLASH->ACR;
        tmpreg &= ~FLASH_ACR_LATENCY;
        /* Set 2 wait-states */
        tmpreg |= (2);
        FLASH->ACR |= tmpreg;

        /* Switch to PLL clocks */
        tmpreg = RCC->CFGR;
        tmpreg &= ~RCC_CFGR_SW;
        tmpreg |= RCC_CFGR_SW_PLL;
        RCC->CFGR = tmpreg;

        if(use_hse){
            core_clock = 72000000;
        }
        else {
            core_clock = 64000000;
        }
    }

    /* Configure ADC clock */
    RCC->CFGR2 = (0x10 << 9) | (0x10 << 4);

    /* Init configuration */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
#ifndef DMA2
    /* Map DAC DMA requests to DMA1 */
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP |
        SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP | SYSCFG_CFGR1_DAC2Ch1_DMA_RMP;
    /* Remap ADC DMA requets to DMA1 Channel 4 */
    SYSCFG->CFGR3 |= SYSCFG_CFGR3_ADC2_DMA_RMP;
#endif

    /* Boost timers to 2 * core frequency */
#ifdef STM32F303x8
    RCC->CFGR3 = 0xF00;
#else
    RCC->CFGR3 = 0x300AF00;
#endif

    /* TODO: check if correct ??? */
    RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;
    return status;
}

uint32_t get_core_clock(){
    return core_clock;
}

void stm32_target_clken(GPIO_TypeDef* port){
    if(port == GPIOA) RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
#ifdef GPIOB
    if(port == GPIOB) RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
#endif
#ifdef GPIOC
    if(port == GPIOC) RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
#endif
#ifdef GPIOD
    if(port == GPIOD) RCC->AHBENR |= RCC_AHBENR_GPIODEN;
#endif
#ifdef GPIOE
    if(port == GPIOE) RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
#endif
#ifdef GPIOF
    if(port == GPIOF) RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
#endif
#ifdef GPIOG
    if(port == GPIOG) RCC->AHBENR |= RCC_AHBENR_GPIOGEN;
#endif
}
