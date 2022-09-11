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
#include "usb_core.h"
#include "usb_cdc.h"
#include "core.h"

static uint8_t usb_initialized = 0;
static comm_t main_usb;

static uint32_t _usb_cdc_write(void* arg, const char* buffer, uint32_t size){
    usb_cdc_write((const uint8_t*)buffer,size);
    return size;
}

static uint32_t _usb_cdc_read(void* arg, char* buffer, uint32_t size){
    return usb_cdc_read((uint8_t*)buffer,size);
}

comm_t *get_main_comm(void){
    if(!usb_initialized){

#ifdef STM32F0_TSSOP20
        /* Remap USB pins for TSSOP20 package */
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
        SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
#endif

        usb_core_init();

        main_usb.hw_handle = NULL;
        main_usb.write_callback = (comm_write_callback_t)_usb_cdc_write;
        main_usb.read_callback = (comm_read_callback_t)_usb_cdc_read;
        usb_initialized = 1;
    }
    return &main_usb;
}
