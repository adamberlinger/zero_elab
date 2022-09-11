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
#ifndef _VOLTMETER_H_
#define _VOLTMETER_H_

#include "core.h"
#include "adc.h"

#ifndef VOLT_SAMPLE_PERIOD_MS
#define VOLT_SAMPLE_PERIOD_MS   (10)
#endif

#define VOLTMETER_DEFAULT_HINT  (ADC_HINT_NONE)

#define VOLTMETER_DECLARE(name,precision,num_channels) \
    VOLTMETER_DECLARE_HINT(name,precision,num_channels, VOLTMETER_DEFAULT_HINT)

#define VOLTMETER_DECLARE_HINT(name,precision,num_channels,hint) \
    const static adc_init_t name ## _adc_init_data = { \
        0, \
        precision, \
        num_channels, \
        0, \
        hint \
    }; \
    static adc_channel_t name ## _adc_channels[num_channels]; \
    static ModuleVoltmeter* name ## _module

#define VOLTMETER_ADD_CHANNEL(name,pin,index) do { \
        int __tmp__dummy; \
        adc_find_config(pin, &__tmp__dummy, &name ## _adc_channels[index]); \
    } while(0)

#define VOLTMETER_SET_REFCHANNEL(name,num) do { \
        name ## _adc_channels[name ## _adc_init_data.num_channels - 1] = num; \
    } while(0)

#define VOLTMETER_MODULE_INIT(name,adc_id) do { \
        name ## _module = new ModuleVoltmeter(3, adc_id, &name ## _adc_init_data, name ## _adc_channels); \
    } while(0)

class ModuleVoltmeter : public Module {
protected:
    enum State {
        IDLE,
        SAMPLING,
        TRANSFER
    };

    PeriphDefaultADC adc;

    int32_t *accum_registers;
    uint16_t accum_count;

    uint16_t avg_samples;
    int16_t *read_buffer;

    int channel;
    int8_t scale;
    uint8_t num_channels;
    State state;
    uint32_t prevTick;    
    int vrefint_real_index;

    void dataShuffle();
public:
    ModuleVoltmeter(int channel, int adc_id,
        const adc_init_t* init_data, const adc_channel_t* channel_info);
    virtual void command(comm_t* comm, const command_t* command);
    virtual void thread(comm_t* comm);
    uint32_t getVDDA();
    void setVREFIndex(int val){vrefint_real_index = val;}
    virtual ~ModuleVoltmeter();
};

#endif /* _VOLTMETER_H_ */
