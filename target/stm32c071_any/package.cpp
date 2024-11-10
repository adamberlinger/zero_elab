/*
 * BSD 3-Clause License
 * 
 * Copyright (c) 2024, Adam Berlinger
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

#include "package.h"

const c0_package_config_t pkg_config[C0_PKG_COUNT] = {
    C0_PKG_DEFINE(TSSOP20_GP),
    C0_PKG_DEFINE(TSSOP20_N),
    C0_PKG_DEFINE(LQFP32_GP),
    C0_PKG_DEFINE(LQFP32_N),
    C0_PKG_DEFINE_WITH_NAME(LQFP64_GP, "stm32c071rb_nucleo"),
    C0_PKG_DEFINE_WITH_NAME(LQFP64_N, "stm32c071rb_nucleo (N suffix)"),
};

int pkg_index = 0;

void c0_package_init(void){
    /* Identify package */
    uint8_t pkg = (*(uint16_t*)(0x1FFF7500)) & 0xF;
    for(int i = 0; i < C0_PKG_COUNT;++i){
        if(pkg == pkg_config[i].pkg_id){
            pkg_index = i;
            break;
        }
    }
    target_name = pkg_config[pkg_index].target_name;
    target_name_length = pkg_config[pkg_index].target_name_length;
}