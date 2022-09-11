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
#ifndef _MODULE_PWM_INPUT_H_
#define _MODULE_PWM_INPUT_H_

#include "core.h"
#include "timer.h"

#define PWM_IN_DECLARE(name,pin) \
    const static timer_init_t name ## _timer_init_data = { \
        TIMER_USAGE_PWM_INPUT, \
        TIMER_INIT_FREQUENCY, \
        1000000, \
        0x8000, \
        pin, \
        0, \
    }; \
    static ModulePWMIn* name ## _module

#define PWM_IN_MODULE_INIT(name, timer_id) do{ \
        name ## _module = new ModulePWMIn(4, timer_id, &name ## _timer_init_data); \
    }while(0)

class ModulePWMIn : public Module {
protected:
    enum State {
        IDLE,
        RUNNING,
    };

    timer_handle_t timer_handle;
    int channel;
    State state;
    uint32_t prevTick;
    int is_calibrating;
public:
    ModulePWMIn(int channel, int timer_id, const timer_init_t* init_data);
    virtual void command(comm_t* comm, const command_t* command);
    virtual void thread(comm_t* comm);
    virtual ~ModulePWMIn();
};

#endif /* _MODULE_PWM_INPUT_H_ */
