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
#include "pulse_counter.h"
#include "core.h"

ModulePulseCounter::ModulePulseCounter(int channel, int timer_id, const timer_init_t* init_data){
    if(init_data->usage != TIMER_USAGE_PULSE_COUNTER){
        return;
    }

    this->channel = channel;
    this->state = IDLE;
    this->value = 0;

    timer_init(&this->timer_handle, timer_id, init_data);
}

ModulePulseCounter::~ModulePulseCounter(){
    timer_deinit(&this->timer_handle);
}

void ModulePulseCounter::command(comm_t* comm, const command_t* command){
    if(command->channel != this->channel){
        return;
    }

    if(command->command_id == 'S'){
        if(command->value){
            timer_start(&this->timer_handle);
            this->state = RUNNING;
        }
        else {
            this->state = IDLE;
            timer_stop(&this->timer_handle);
            this->value = 0;
        }
    }
}

void ModulePulseCounter::thread(comm_t* comm){
    if(this->state == RUNNING){
        if(timer_get_counter_value(&this->timer_handle,&this->value)){
            send_binary_data_header(comm,this->channel,4);
            //value *= 100;
            comm_write(comm,(char*)&this->value,4);

            timer_start(&this->timer_handle);
            this->value = 0;
        }
    }
}
