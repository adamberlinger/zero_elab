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
#ifndef _NUCLEO_PINDEF_
#define _NUCLEO_PINDEF_

/* Nucleo to STM definitions */
#define PIN_D0 DEFINE_PIN(GPIOA_BASE,3)
#define PIN_D1 DEFINE_PIN(GPIOA_BASE,2)
#define PIN_D2 DEFINE_PIN(GPIOA_BASE,10)
#define PIN_D3 DEFINE_PIN(GPIOB_BASE,3)
#define PIN_D4 DEFINE_PIN(GPIOB_BASE,5)
#define PIN_D5 DEFINE_PIN(GPIOB_BASE,4)
#define PIN_D6 DEFINE_PIN(GPIOB_BASE,10)
#define PIN_D7 DEFINE_PIN(GPIOA_BASE,8)
#define PIN_D8 DEFINE_PIN(GPIOA_BASE,9)
#define PIN_D9 DEFINE_PIN(GPIOC_BASE,7)
#define PIN_D10 DEFINE_PIN(GPIOB_BASE,6)
#ifdef STM32F302x8
    #define PIN_D11 DEFINE_PIN(GPIOB_BASE,15)
    #define PIN_D12 DEFINE_PIN(GPIOB_BASE,14)
    #define PIN_D13 DEFINE_PIN(GPIOB_BASE,13)
#else
    #define PIN_D11 DEFINE_PIN(GPIOA_BASE,7)
    #define PIN_D12 DEFINE_PIN(GPIOA_BASE,6)
    #define PIN_D13 DEFINE_PIN(GPIOA_BASE,5)
#endif
#define PIN_D14 DEFINE_PIN(GPIOB_BASE,9)
#define PIN_D15 DEFINE_PIN(GPIOB_BASE,8)

#define PIN_A0 DEFINE_PIN(GPIOA_BASE,0)
#define PIN_A1 DEFINE_PIN(GPIOA_BASE,1)
#define PIN_A2 DEFINE_PIN(GPIOA_BASE,4)
#define PIN_A3 DEFINE_PIN(GPIOB_BASE,0)
#define PIN_A4 DEFINE_PIN(GPIOC_BASE,1)
#define PIN_A5 DEFINE_PIN(GPIOC_BASE,0)

#define PIN_D0_NAME "PA3"
#define PIN_D1_NAME "PA2"
#define PIN_D2_NAME "PA10"
#define PIN_D3_NAME "PB3"
#define PIN_D4_NAME "PB5"
#define PIN_D5_NAME "PB4"
#define PIN_D6_NAME "PB10"
#define PIN_D7_NAME "PA8"
#define PIN_D8_NAME "PA9"
#define PIN_D9_NAME "PC7"
#define PIN_D10_NAME "PB6"
#ifdef STM32F302x8
    #define PIN_D11_NAME "PB15"
    #define PIN_D12_NAME "PB14"
    #define PIN_D13_NAME "PB13"
#else
    #define PIN_D11_NAME "PA7"
    #define PIN_D12_NAME "PA6"
    #define PIN_D13_NAME "PA5"
#endif
#define PIN_D14_NAME "PB9"
#define PIN_D15_NAME "PB8"

#define PIN_A0_NAME "PA0"
#define PIN_A1_NAME "PA1"
#define PIN_A2_NAME "PA4"
#define PIN_A3_NAME "PB0"
#define PIN_A4_NAME "PC1"
#define PIN_A5_NAME "PC0"

/* Reverse name definitions */
#define NUCLEO_PIN_PA3 "D0"
#define NUCLEO_PIN_PA2 "D1"
#define NUCLEO_PIN_PA10 "D2"
#define NUCLEO_PIN_PB3 "D3"
#define NUCLEO_PIN_PB5 "D4"
#define NUCLEO_PIN_PB4 "D5"
#define NUCLEO_PIN_PB10 "D6"
#define NUCLEO_PIN_PA8 "D7"
#define NUCLEO_PIN_PA9 "D8"
#define NUCLEO_PIN_PC7 "D9"
#define NUCLEO_PIN_PB6 "D10"
#ifdef STM32F302x8
    #define NUCLEO_PIN_PB15 "D11"
    #define NUCLEO_PIN_PB14 "D12"
    #define NUCLEO_PIN_PB13 "D13"
#else
    #define NUCLEO_PIN_PA7 "D11"
    #define NUCLEO_PIN_PA6 "D12"
    #define NUCLEO_PIN_PA5 "D13"
#endif
#define NUCLEO_PIN_PB9 "D14"
#define NUCLEO_PIN_PB8 "D15"

#define NUCLEO_PIN_PA0 "A0"
#define NUCLEO_PIN_PA1 "A1"
#define NUCLEO_PIN_PA4 "A2"
#define NUCLEO_PIN_PB0 "A3"
#define NUCLEO_PIN_PC1 "A4"
#define NUCLEO_PIN_PC0 "A5"

#endif /* _NUCLEO_PINDEF_ */
