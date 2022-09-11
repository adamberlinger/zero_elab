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
#include "stm32_common.h"
#include "stm32_dma.h"

GPIO_TypeDef* stm32_get_port(gpio_pin_t pin){
    return (GPIO_TypeDef*)((GPIO_PORT_MASK & pin) | (GPIOA_BASE & ~GPIO_PORT_MASK));
}

uint8_t stm32_get_pindef(gpio_pin_t pin){
    return (uint8_t)(GPIO_PIN_MASK & pin);
}

void stm32_common_init(void){
    dma_global_init();
}

void gpio_toggle(gpio_pin_t pin){
    GPIO_TypeDef* port = stm32_get_port(pin);
    uint32_t pinmask = (1 << stm32_get_pindef(pin));

    if(port->ODR & (pinmask)){
        port->BSRR = (pinmask) << 16;
    }
    else {
        port->BSRR = (pinmask);
    }
}

void stm32_gpio_init(gpio_pin_t pin,gpio_mode_t mode){
    GPIO_TypeDef* port = stm32_get_port(pin);
    uint8_t pinnum = stm32_get_pindef(pin);

    stm32_target_clken(port);
#ifdef STM32F1XX
    volatile uint32_t* cr = (pinnum >= 8)?&port->CRH:&port->CRL;
    uint8_t pinnum2 = (pinnum >= 8)?(pinnum-8):pinnum;
    uint32_t config = 0x4;
    pinnum2 <<= 2;

    if(mode == MODE_OUT_PP){
        config = 0x3;
    }
    else if(mode == MODE_OUT_OD){
        config = 0x7;
    }
    else if(mode == MODE_AN){
        config = 0x0;
    }
    else {
        config = 0x4;
    }

    *cr = (*cr & ~(uint32_t)(0xF << pinnum2)) | (config << pinnum2);
#else
    /* Works except for F1 */
    uint8_t pinnum2 = pinnum << 1;

    port->MODER &= ~(uint32_t)(0x3 << pinnum2);
    port->OTYPER &= ~(uint32_t)(0x1 << pinnum);
    port->PUPDR &= ~(uint32_t)(0x3 << pinnum2);

    if(pinnum >= 8){
        port->AFR[1] &= ~(uint32_t)(0xF << ((pinnum - 8) * 4));
    }
    else {
        port->AFR[0] &= ~(uint32_t)(0xF << (pinnum * 4));
    }

    if(mode == MODE_OUT_PP){
        port->MODER |= (0x1 << pinnum2);
        port->OSPEEDR |= (0x3 << pinnum2);
    }
    else if(mode == MODE_OUT_OD) {
        port->MODER |= (0x1 << pinnum2);
        port->OTYPER |= (0x1 << pinnum);
        port->OSPEEDR |= (0x3 << pinnum2);
    }
    else if(mode == MODE_AN){
        port->MODER |= (0x3 << pinnum2);
    }
#endif
}

void stm32_gpio_af(gpio_pin_t pin,gpio_mode_t mode,int af_number){
    GPIO_TypeDef* port = stm32_get_port(pin);
    uint8_t pinnum = stm32_get_pindef(pin);

    stm32_target_clken(port);
#ifdef STM32F1XX
    volatile uint32_t* cr = (pinnum >= 8)?&port->CRH:&port->CRL;
    uint8_t pinnum2 = (pinnum >= 8)?(pinnum-8):pinnum;
    uint32_t config = 0x4;
    pinnum2 <<= 2;

    if(mode == MODE_IN){
        config = 0x4;
    }
    else if(mode == MODE_OUT_OD){
        config = 0xF;
    }
    else{
        config = 0xB;
    }

    *cr = (*cr & ~(uint32_t)(0xF << pinnum2)) | (config << pinnum2);

    /* There is no AF number used */
    (void)af_number;
#else
    /* Works except for F1 */
    uint8_t pinnum2 = pinnum << 1;
    port->MODER &= ~(uint32_t)(0x3 << pinnum2);
    port->OTYPER &= ~(uint32_t)(0x1 << pinnum);
    port->PUPDR &= ~(uint32_t)(0x3 << pinnum2);

    port->MODER |= (0x2 << pinnum2);
    if(mode == MODE_OUT_OD) {
        port->OTYPER |= (0x1 << pinnum);
    }
    port->OSPEEDR |= (0x3 << pinnum2);

    if(pinnum >= 8){
        port->AFR[1] &= ~(uint32_t)(0xF << ((pinnum - 8) * 4));
        port->AFR[1] |= ((0xFF & af_number) << ((pinnum - 8) * 4));
    }
    else {
        port->AFR[0] &= ~(uint32_t)(0xF << (pinnum * 4));
        port->AFR[0] |= ((0xFF & af_number) << (pinnum * 4));
    }
#endif
}

void stm32_gpio_pullup(gpio_pin_t pin){
#ifndef STM32F1XX
    /* Works except for F1 */
    GPIO_TypeDef* port = stm32_get_port(pin);
    uint8_t pinnum = stm32_get_pindef(pin);
    uint8_t pinnum2 = pinnum << 1;

    port->PUPDR &= ~(uint32_t)(0x3 << pinnum2);
    port->PUPDR |= (uint32_t)(0x1 << pinnum2);
#endif
}

void stm32_gpio_pulldown(gpio_pin_t pin){
#ifndef STM32F1XX
    /* Works except for F1 */
    GPIO_TypeDef* port = stm32_get_port(pin);
    uint8_t pinnum = stm32_get_pindef(pin);
    uint8_t pinnum2 = pinnum << 1;

    port->PUPDR &= ~(uint32_t)(0x3 << pinnum2);
    port->PUPDR |= (uint32_t)(0x2 << pinnum2);
#endif
}

int stm32_gpio_read(gpio_pin_t pin){
    GPIO_TypeDef* port = stm32_get_port(pin);
    uint8_t pinnum = stm32_get_pindef(pin);
    return (port->IDR & (0x1 << pinnum)) > 0;
}