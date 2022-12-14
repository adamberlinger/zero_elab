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

typedef void (*bootloader_callback_t)(void);
#define BOOTLOADER_ADDRESS  (0x1FFFC518)
#define BOOTLOADER_START    (0x1FFFC400)

static uint32_t core_clock = 16000000;

void enter_bootloader(){
    NVIC_SystemReset();
}

int init_cpu(void){
    int tries = 2000;
    int status = 0;
    uint32_t tmpreg;

    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APBENR2 |=RCC_APBENR2_SYSCFGEN;

    /* TODO: compute PLL parameters */
    /* TODO: make work for HSE */
    /* PLLMUL = 9*/
    /* Set AHB & APB prescalers */

    /* PLL_OUT = 16 * 8 / 2 = 64 */

    tmpreg = RCC_PLLCFGR_PLLSRC_HSI | (8 << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLR_0;
    RCC->PLLCFGR = tmpreg;

    RCC->CR |= RCC_CR_PLLON;

    tries = 2000;
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
        tmpreg |= (2) | FLASH_ACR_ICEN;
        FLASH->ACR = tmpreg;

        /* Switch to PLL clocks */
        tmpreg = RCC->CFGR;
        tmpreg &= ~RCC_CFGR_SW;
        tmpreg |= RCC_CFGR_SW_1;
        RCC->CFGR = tmpreg;

        core_clock = 64000000;
    }
#ifdef STM32G0_SWITCH_PA
    SYSCFG->CFGR1 |= 0x18;
#endif

    return status;
}

uint32_t get_core_clock(){
    return core_clock;
}

void stm32_target_clken(GPIO_TypeDef* port){
    if(port == GPIOA) RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
#ifdef GPIOB
    if(port == GPIOB) RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
#endif
#ifdef GPIOC
    if(port == GPIOC) RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
#endif
#ifdef GPIOD
    if(port == GPIOD) RCC->IOPENR |= RCC_IOPENR_GPIODEN;
#endif
#ifdef GPIOE
    if(port == GPIOE) RCC->IOPENR |= RCC_IOPENR_GPIOEEN;
#endif
#ifdef GPIOF
    if(port == GPIOF) RCC->IOPENR |= RCC_IOPENR_GPIOFEN;
#endif
#ifdef GPIOG
    if(port == GPIOG) RCC->IOPENR |= RCC_IOPENR_GPIOGEN;
#endif
}
