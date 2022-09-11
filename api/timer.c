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
#include "timer.h"

static uint32_t timer_lock_register = 0;

uint16_t prescaler_split(uint32_t in,uint16_t* out2){
    uint16_t p1 = 1;
    uint32_t p2 = in;
    while(p2 > 0xFFFF){
        if((p2 & 0x1) == 0){
            p1 <<= 1;
            p2 >>= 1;
        }
        else if((p2 % 5) == 0){
            p1 *= 5;
            p2 /= 5;
        }
        else if((p2 % 3) == 0){
            p1 *= 3;
            p2 /= 3;
        }
        else {
            break;
        }
    }

    if(p2 > 0xFFFF){
        p1 = (in >> 16) + 1;
        p2 = (in / p1);
    }
    *out2 = (uint16_t)p2;
    return p1;
}

int timer_init(timer_handle_t* timer_handle, int timer_id, const timer_init_t* init_data){
    int status;
    if(!timer_lock(timer_id)){
        return ERROR_LOCK_FAILED;
    }

    status = timer_target_init(timer_handle,timer_id,init_data);
    if(status != ERROR_NONE){
        timer_unlock(timer_id);
    }
    return status;
}

int timer_deinit(timer_handle_t* timer_handle){
    return timer_target_deinit(timer_handle);
}

uint32_t timer_get_resolution(timer_handle_t* timer_handle){
    return timer_target_get_resolution(timer_handle);
}

int timer_start(timer_handle_t* timer_handle){
    return timer_target_start(timer_handle);
}

uint32_t timer_get_clocks(timer_handle_t* timer_handle){
    return timer_target_get_clocks(timer_handle);
}

uint32_t timer_get_frequency_divider(timer_handle_t* timer_handle){
    return timer_target_get_frequency_divider(timer_handle);
}

int timer_stop(timer_handle_t* timer_handle){
    return timer_target_stop(timer_handle);
}

int timer_change_time(timer_handle_t* timer_handle, timer_time_type_t type, uint32_t value){
    return timer_target_change_time(timer_handle, type, value);
}

uint32_t timer_change_frequency(timer_handle_t* timer_handle, uint32_t value, uint32_t* prescaler_out){
    timer_change_time(timer_handle,TIMER_INIT_FREQUENCY, value);
    if(prescaler_out){
        *prescaler_out = timer_target_get_frequency_divider(timer_handle);
    }
    return timer_target_get_clocks(timer_handle);
}

int timer_change_duty_cycle(timer_handle_t* timer_handle, uint16_t duty_cycle){
    return timer_target_change_duty_cycle(timer_handle,duty_cycle);
}

int timer_lock(int timer_id){
    return lock_bit(timer_id,&timer_lock_register);
}

int timer_is_free(int timer_id){
    return !lock_test_bit(timer_id,&timer_lock_register);
}

int timer_get_pwm_input_value(timer_handle_t* timer_handle, uint32_t* freq_out, uint32_t* duty_cycle_out){
    return timer_target_get_pwm_input_value(timer_handle, freq_out, duty_cycle_out);
}

int timer_get_flag(timer_handle_t* timer_handle, timer_flag_t flag){
    return timer_target_get_flag(timer_handle, flag);
}

int timer_clear_flag(timer_handle_t* timer_handle, timer_flag_t flag){
    return timer_target_clear_flag(timer_handle, flag);
}


int timer_get_counter_value(timer_handle_t* timer_handle, uint32_t *value_out){
    return timer_target_get_counter_value(timer_handle, value_out);
}

void timer_unlock(int timer_id){
    unlock_bit(timer_id,&timer_lock_register);
}

void timer_unlock_all(){
    timer_lock_register = 0;
}
