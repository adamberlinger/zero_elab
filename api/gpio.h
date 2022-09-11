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
/**
 * \file gpio.h
 * \brief API for GPIO and configuring unused pins
 *
 * It is useful to configure unused pins to output or analog mode
 * because unconnected input pin can generate additional current consumption
 * and noise.
 */
#ifndef _API_GPIO_H_
#define _API_GPIO_H_

#include "core.h"
#include "gpio_target.h"

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * \typedef gpio_pin_t
 * \brief Type representing GPIO pin
 *
 * This type is target dependent.
 * This type shouldn't be bigger than 32-bit integer, since
 * it is being passed to functions by value
 */

#if GPIO_UNUSED_PIN_RANGE_SIZE
    extern const gpio_pin_t gpio_unused_pin_range[GPIO_UNUSED_PIN_RANGE_SIZE];
#endif

/**
 * \brief Inializes unused pins
 */
void gpio_init_pins(void);

/**
 * \brief Configures unused pin
 * \param pin Pin to be configured
 *
 * This function is target dependent
 */
void gpio_pin_unused(gpio_pin_t pin);
/**
 * \brief Moves to next pin
 * \param pin Pin to be "incremented"
 *
 * This function is target dependent.
 * It enables to iterate over pin range
 */
void gpio_pin_next(gpio_pin_t* pin);
/**
 * \brief Compares two pins
 * \return Non-zero value if pins match
 */
int gpio_pin_compare(gpio_pin_t a,gpio_pin_t b);

void gpio_toggle(gpio_pin_t pin);

#ifdef __cplusplus
    }
#endif

#endif /* _API_GPIO_H_ */
