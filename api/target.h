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
 * \dir api
 * \brief Folder contains API for interfaceing with hardware
 *
 * API function usually have two prototypes, general and target specific.
 * The general function calls the target specific, but can
 * additionally check input parameters.
 * E.g. adc_init() calls adc_target_init(), but it implements
 * parameters check routine which is common for all implementations.
 */
/**
 * \file target.h
 * \brief This file declares functions, which need to be implemented
 * in target specific modules
 */
#ifndef _API_TARGET_H_
#define _API_TARGET_H_

#include <stdint.h>
#include "comm.h"

#ifdef __cplusplus
    extern "C" {
#endif
/**
 * \brief Initialize CPU to work in C environment
 *
 * This function usually setups MCU clocks.
 */
int init_cpu(void);
/**
 * \brief Gets clock frequency
 * \return Clock frequency in Hz
 */
uint32_t get_core_clock(void);
/**
 * \brief Function for accessing default communication interface
 * \return Pointer to communication interface
 *
 * This can be e.g. UART interface or USB VCOM interface
 */
comm_t *get_main_comm(void);
/**
 * \brief Initializes target
 *
 * This initializes additional features related to target
 */
void target_init(void);
/**
 * \brief Initializes functions
 *
 * This initializes all functional modules
 */
void functions_init(void);

/**
 * \brief Returns supply voltage in millivolts
 * \return Supply voltage in millivolts
 *
 * This function is target dependent.
 * It can also return constant pre-defined value.
 */
uint32_t get_vdda(void);

/**
 * \brief Returns tick in milliseconds from system counter
 * \return Current tick in milliseconds
 *
 * This function is target dependent
 */
uint32_t get_ms_ticks(void);
/**
 * \brief Initializes system tick counter
 *
 * This function is target dependent
 */
void init_ms_ticks(void);
#ifdef __cplusplus
    }
#endif

#endif /* _API_TARGET_H_ */
