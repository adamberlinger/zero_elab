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

static uint32_t core_clock = 8000000;

void enter_bootloader(){
    NVIC_SystemReset();
}

/* NOTE: currently works for series with internal 48MHz oscillator */
int init_cpu(void){
    int tries = 200;
    int status = 0;
    uint32_t tmpreg;

#if defined (__GNUC__) && defined(ENABLE_BL_ENTER)
    if(RCC->CSR & RCC_CSR_SFTRSTF){
        /* Jump to BOOTLOADER_ADDRESS */
        /*bootloader_callback_t bootloader =
            (bootloader_callback_t)BOOTLOADER_ADDRESS;
        bootloader();*/
        volatile uint32_t addr = *(uint32_t*)(BOOTLOADER_START+4);
        __ASM volatile ( "BX %0" : __CMSIS_GCC_OUT_REG(addr) );
    }
#endif

    RCC->CR2 |= RCC_CR2_HSI48ON;
    while(tries && (RCC->CR2 & RCC_CR2_HSI48RDY) == 0){
        tries--;
    }

    if((RCC->CR2 & RCC_CR2_HSI48RDY) == 0){
        /* Fall back to internal 8MHz oscillator */
        status = -2;
    }
    else {
#ifdef STM32F0_USE_HSI
        tmpreg = RCC->CFGR;
        tmpreg &= ~(0xFF8000);
        tmpreg |= 0x280000; /* PLLMUL = x12 */
        RCC->CFGR = tmpreg;

        RCC->CR |= RCC_CR_PLLON;
        tries = 200;
        while(tries && (RCC->CR & RCC_CR_PLLRDY) == 0){
            tries--;
        }

        if((RCC->CR & RCC_CR_PLLRDY) == 0){
            return -2;
        }
#endif
        /* Set flash latency for higher core clock */
        tmpreg = FLASH->ACR;
        tmpreg &= ~FLASH_ACR_LATENCY;
        /* Set 1 wait-state */
        tmpreg |= (1);
        FLASH->ACR |= tmpreg;

        /* Switch to HSI48 clock */
        tmpreg = RCC->CFGR;
        tmpreg &= ~RCC_CFGR_SW;
#ifdef STM32F0_USE_HSI
        tmpreg |= RCC_CFGR_SW_PLL;
#else
        tmpreg |= RCC_CFGR_SW_HSI48;
#endif
        RCC->CFGR = tmpreg;

        core_clock = 48000000;
    }
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
