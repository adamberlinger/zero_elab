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
#ifndef _API_DAC_H_
#define _API_DAC_H_

#include "core.h"
#include "circular_buffer.h"

/* This needs to be defined before dac_target.h include */
typedef buffer_process_callback_t dac_write_callback_t;

typedef struct {
    uint32_t sample_rate; // 0 ~ software trigger
    int channel;
    uint32_t buffer_size;
}dac_init_t;

template<class BaseDAC>
class PeriphDAC : public BaseDAC {
public:
    PeriphDAC(int dac_id, const dac_init_t* init_data):
        BaseDAC(dac_id, init_data){

        circular_buffer_t* circular_buffer = this->getCircularBuffer();
        if(circular_buffer){
            circular_buffer->control_arg = this;
            circular_buffer->stop_callback = (buffer_stop_callback_t)dac_stop;
            circular_buffer->start_callback = (buffer_start_callback_t)dac_start_continuous;
            circular_buffer->stop_precise_callback = NULL;
            circular_buffer->get_stop_state_callback = NULL;
            circular_buffer->deinit_callback = (buffer_deinit_callback_t)NULL;
        }
    }

    virtual ~PeriphDAC(){}

    static void dac_stop(PeriphDAC<BaseDAC>* _this){
        _this->stop();
    }

    static void dac_start_continuous(PeriphDAC<BaseDAC>* _this){
        _this->startContinuous();
    }

    void writeDataContinuous(dac_write_callback_t callback,void* user_arg){
        circular_buffer_t* circular_buffer = this->getCircularBuffer();
        if(circular_buffer){
            circular_buffer->process_arg = user_arg;
            circular_buffer->process_callback = callback;
        }
        BaseDAC::startContinuous();
    }

    void startContinuous(){
        BaseDAC::startContinuous();
    }

    void stop(){
        BaseDAC::stop();
    }

    void setLevel(uint32_t level){
        BaseDAC::setLevel(level);
    }

    circular_buffer_t* getCircularBuffer(){
        return BaseDAC::getCircularBuffer();
    }
};

#include "dac_target.h"

#endif /* _API_ADC_H_ */
