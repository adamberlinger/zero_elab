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
#ifndef _NUCLEO32_PINDEF_
#define _NUCLEO32_PINDEF_

/* Nucleo to STM definitions */
#define PIN_D0 DEFINE_PIN(GPIOA_BASE,10)
#define PIN_D1 DEFINE_PIN(GPIOA_BASE,9)
#define PIN_D2 DEFINE_PIN(GPIOA_BASE,12)
#define PIN_D3 DEFINE_PIN(GPIOB_BASE,0)
#define PIN_D4 DEFINE_PIN(GPIOB_BASE,7)
#define PIN_D5 DEFINE_PIN(GPIOB_BASE,6)
#define PIN_D6 DEFINE_PIN(GPIOB_BASE,1)
#define PIN_D7 DEFINE_PIN(GPIOF_BASE,0)
#define PIN_D8 DEFINE_PIN(GPIOF_BASE,1)
#define PIN_D9 DEFINE_PIN(GPIOA_BASE,8)
#define PIN_D10 DEFINE_PIN(GPIOA_BASE,11)
#define PIN_D11 DEFINE_PIN(GPIOB_BASE,5)
#define PIN_D12 DEFINE_PIN(GPIOB_BASE,4)
#define PIN_D13 DEFINE_PIN(GPIOB_BASE,3)

#define PIN_A0 DEFINE_PIN(GPIOA_BASE,0)
#define PIN_A1 DEFINE_PIN(GPIOA_BASE,1)
#define PIN_A2 DEFINE_PIN(GPIOA_BASE,3)
#define PIN_A3 DEFINE_PIN(GPIOA_BASE,4)
#define PIN_A4 DEFINE_PIN(GPIOA_BASE,5)
#define PIN_A5 DEFINE_PIN(GPIOA_BASE,6)
#define PIN_A6 DEFINE_PIN(GPIOA_BASE,7)
#define PIN_A7 DEFINE_PIN(GPIOA_BASE,2)

#define PIN_D0_NAME "PA10"
#define PIN_D1_NAME "PA9"
#define PIN_D2_NAME "PA12"
#define PIN_D3_NAME "PB0"
#define PIN_D4_NAME "PB7"
#define PIN_D5_NAME "PB6"
#define PIN_D6_NAME "PB1"
#define PIN_D7_NAME "PF0"
#define PIN_D8_NAME "PF1"
#define PIN_D9_NAME "PA8"
#define PIN_D10_NAME "PA11"
#define PIN_D11_NAME "PB5"
#define PIN_D12_NAME "PB4"
#define PIN_D13_NAME "PB3"

#define PIN_A0_NAME "PA0"
#define PIN_A1_NAME "PA1"
#define PIN_A2_NAME "PA3"
#define PIN_A3_NAME "PA4"
#define PIN_A4_NAME "PA5"
#define PIN_A5_NAME "PA6"
#define PIN_A6_NAME "PA7"
#define PIN_A7_NAME "PA2"

/* Reverse name definitions */
#define NUCLEO_PIN_PA10 "D0"
#define NUCLEO_PIN_PA9 "D1"
#define NUCLEO_PIN_PA12 "D2"
#define NUCLEO_PIN_PB0 "D3"
#define NUCLEO_PIN_PB7 "D4"
#define NUCLEO_PIN_PB6 "D5"
#define NUCLEO_PIN_PB1 "D6"
#define NUCLEO_PIN_PF0 "D7"
#define NUCLEO_PIN_PF1 "D8"
#define NUCLEO_PIN_PA8 "D9"
#define NUCLEO_PIN_PA11 "D10"
#define NUCLEO_PIN_PB5 "D11"
#define NUCLEO_PIN_PB4 "D12"
#define NUCLEO_PIN_PB3 "D13"

#define NUCLEO_PIN_PA0 "A0"
#define NUCLEO_PIN_PA1 "A1"
#define NUCLEO_PIN_PA3 "A2"
#define NUCLEO_PIN_PA4 "A3"
#define NUCLEO_PIN_PA5 "A4"
#define NUCLEO_PIN_PA6 "A5"
#define NUCLEO_PIN_PA7 "A6"
#define NUCLEO_PIN_PA2 "A7"

#endif /* _NUCLEO32_PINDEF_ */
