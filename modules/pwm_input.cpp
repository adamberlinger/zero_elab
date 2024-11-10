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
#include "pwm_input.h"
#include "core.h"

ModulePWMIn::ModulePWMIn(int channel, int timer_id, const timer_init_t* init_data){
    if(init_data->usage != TIMER_USAGE_PWM_INPUT){
        return;
    }

    this->channel = channel;
    this->state = IDLE;
    this->is_calibrating = 0;

    timer_init(&this->timer_handle, timer_id, init_data);
}

ModulePWMIn::~ModulePWMIn(){
    timer_deinit(&this->timer_handle);
}

void ModulePWMIn::command(comm_t* comm, const command_t* command){
    if(command->channel != this->channel){
        return;
    }

    if(command->command_id == 'S'){
        if(command->value){
            timer_start(&this->timer_handle);
            this->state = RUNNING;
            this->prevTick = get_ms_ticks();
        }
        else {
            this->state = IDLE;
            timer_stop(&this->timer_handle);
        }
    }
}

void ModulePWMIn::thread(comm_t* comm){
    /* TODO: rewrite for general case */
    /* TODO: stop calibrating when at limits */
    if(this->state == RUNNING){
        if(timer_get_flag(&this->timer_handle, TIMER_FLAG_UPDATE)){
            /* Timer running too fast */
            if(this->is_calibrating == 0){
                timer_target_update_prescaler(&this->timer_handle, 1);
                this->is_calibrating = 5;
            }
            else {
                this->is_calibrating--;
            }
            timer_clear_flag(&this->timer_handle, TIMER_FLAG_UPDATE);
        }
        else if(timer_get_flag(&this->timer_handle, TIMER_FLAG_CHANNEL_CAPTURED)){
            uint32_t data[3];
            data[0] = timer_get_resolution(&this->timer_handle);
            timer_target_get_pwm_input_value(&this->timer_handle,
                &data[1],&data[2]);
            timer_clear_flag(&this->timer_handle, TIMER_FLAG_CHANNEL_CAPTURED);

            if(this->is_calibrating == 0){
                if((get_ms_ticks() - this->prevTick) > 99){
                    send_binary_data_header(comm,this->channel,12);
                    comm_write(comm,(char*)data,12);
                    this->prevTick = get_ms_ticks();
                }
                if(data[1] < 0x8000){
                    /* Timer running too slow */
                    timer_target_update_prescaler(&this->timer_handle, -1);
                    this->is_calibrating = 5;
                }
            }
            else {
                this->is_calibrating--;
            }
        }
    }
}
