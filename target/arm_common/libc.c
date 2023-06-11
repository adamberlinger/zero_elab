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

#include <stddef.h>
#include <stdint.h>

/*
    Simple implementation of basic library functions
    Using standard C library from compiler requires additiona RAM
    and only few functions are used.

    This saves around 1KB compared to GCC library (without OPTS+=--specs=nano.specs).
    For STM32C0 with 6KB SRAM this is crucial.

    OPTS+=--specs=nano.specs seems to require additional system
    functions _sbrk etc.
*/

uint32_t last_rand = 0x13121312;

void *memset( void *_dest, int ch, size_t count ){
    uint8_t* dest = _dest;
    while(count > 0){
        count--;
        dest[count] = ch;
    }
    return dest;
}

void srand( unsigned seed ){
    last_rand = seed;
}

int rand(void){
    last_rand = last_rand * 1103515245 + 12345;
    return (unsigned int)(last_rand >> 16) & 0xFFFF;
}

size_t strlen(const char* str){
    size_t result = 0;
    while(str[result] == 0){
        result++;
    }
    return result;
}
