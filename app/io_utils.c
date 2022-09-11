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
#include "io_utils.h"
#include "timing.h"
#include <string.h>

void send_binary_data(comm_t* comm,uint8_t channel,char* data,uint16_t length){
    send_binary_data_header(comm,channel,length);
    comm_write(comm,data,length);
}

void send_binary_data_header(comm_t* comm,uint8_t channel,uint16_t length){
    uint8_t header[4] = {
        0xFF,channel,UINT16_TO_U8ARRAY_LE(length)
    };
    comm_write(comm,(char*)header,4);
}

void send_command(comm_t* comm, uint8_t channel, uint8_t command_id, uint32_t value){
    uint8_t data[COMMAND_DATA_SIZE] = {
        0xFE, UINT32_TO_U8ARRAY_LE(value),command_id,
        channel
    };
    comm_write(comm,(char*)data,COMMAND_DATA_SIZE);
}

static uint32_t cmd_prev_time=0;

int receive_command(comm_t* comm,command_t* command){
    uint32_t current_time = get_ms_ticks();
    int tmp;
    /* Discard previous malformed command or timeout */
    if(command->data_readed >= COMMAND_DATA_SIZE ||
        (current_time - cmd_prev_time) > 100){
        command->data_readed = 0;
        cmd_prev_time = current_time;
    }
    tmp = comm_read(comm,(char*)(command->raw_data)+command->data_readed,COMMAND_DATA_SIZE - command->data_readed,0);
    if(tmp){
        command->data_readed += tmp;
        cmd_prev_time = current_time;
    }
    if(command->data_readed >= COMMAND_DATA_SIZE && command->raw_data[0] == 0xFE){
        command->value = command->raw_data[1] | (command->raw_data[2] << 8) | (command->raw_data[3] << 16) |
            (command->raw_data[4] << 24);
        command->command_id = command->raw_data[5];
        command->channel = command->raw_data[6];
        return 1;
    }
    return 0;
}

uint32_t convert_samplerate(uint16_t input){
    uint32_t multiplier = 1;
    uint16_t exponent = (input >> 14) & 0x3;
    while(exponent){
        exponent--;
        multiplier *= 1000;
    }
    return (input & 0x3FFF) * multiplier;
}
