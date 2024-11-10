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
#ifndef _PACKAGE_H_
#define _PACKAGE_H_

#include "core.h"

typedef enum {
    TSSOP20_GP = 0x3,
    TSSOP20_N  = 0x4,
    LQFP32_GP  = 0x7,
    LQFP32_N   = 0x8,
    LQFP64_GP  = 0xB,
    LQFP64_N   = 0xC,
}c0_package_t;

typedef struct {
    const char* target_name;
    uint16_t target_name_length;
    uint8_t pkg_id;
}c0_package_config_t;

#define C0_PKG_COUNT   (6)
#define C0_PKG_NAME(name) "stm32c071 (" #name ")"
#define C0_PKG_DEFINE_WITH_NAME(pkg, name) {name, sizeof(name), pkg}
#define C0_PKG_DEFINE(pkg) C0_PKG_DEFINE_WITH_NAME(pkg,C0_PKG_NAME(pkg))

extern const c0_package_config_t pkg_config[C0_PKG_COUNT];
extern int pkg_index;

void c0_package_init(void);

#define C0_IS_NUCLEO (pkg_index >= 4)

#endif /* _PACKAGE_H_ */