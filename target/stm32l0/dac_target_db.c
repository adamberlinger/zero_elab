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
#include "dac_target_db.h"

#define DAC_PIN_ENTRY(dac_id,dac_channel,port,pin) {DEFINE_DAC_CHANNEL(dac_id,dac_channel),DEFINE_PIN(port,pin)}

const dac_pin_db_t dac_pin_db[DAC_PIN_DB_SIZE] = {
    DAC_PIN_ENTRY(1, 1, GPIOA_BASE, 4),
    DAC_PIN_ENTRY(1, 2, GPIOA_BASE, 5),
};

const dac_dma_db_t dac_dma_db[DAC_DMA_DB_SIZE] = {
    {DEFINE_DAC_CHANNEL(1,1), DMA1_Channel2, 9},
    {DEFINE_DAC_CHANNEL(1,2), DMA1_Channel4, 15},
};
const dac_tim_db_t dac_tim_db[DAC_TIM_DB_SIZE] = {
    {1, 6, 0},
    {1, 3, 1},
    {1, 21, 3},
    {1, 2, 4},
    {1, 7, 5},
};
