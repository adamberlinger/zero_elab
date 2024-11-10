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
#include "voltmeter.h"
#include "core.h"

ModuleVoltmeter::ModuleVoltmeter(int channel, int adc_id,
    const adc_init_t* init_data, const adc_channel_t* channel_info):
        adc(adc_id,init_data,channel_info){

    if(adc.getErrorFlags() != BasePeriph::ERROR_NONE){
        return;
    }

    /* TODO: rewrite for both 16-bit and 8-bit width */
    /* TODO: check if not NULL */
    this->read_buffer = (int16_t*)module_malloc(init_data->num_channels * 2);

    this->accum_registers = (int32_t*)module_malloc(init_data->num_channels * 4);
    memset(this->accum_registers,0,init_data->num_channels*4);

    this->accum_count = 0;
    this->avg_samples = 16;
    this->channel = channel;
    this->num_channels = init_data->num_channels;
    this->state = IDLE;
    this->prevTick = get_ms_ticks();
    this->scale = 1;
    this->fixedVDDA = 0;

    vrefint_real_index = init_data->num_channels-1;
}

ModuleVoltmeter::~ModuleVoltmeter(){
    //adc_deinit(&this->adc_handle);
}

void ModuleVoltmeter::command(comm_t* comm, const command_t* command){
    if(command->channel != this->channel){
        return;
    }

    if(command->command_id == 'S'){
        if(command->value != 0){
            if(this->state == IDLE){
                this->state = SAMPLING;
                this->accum_count = 0;
                memset(this->accum_registers,0,this->num_channels*4);
                this->prevTick = get_ms_ticks();
            }
        }
        else {
            this->state = IDLE;
        }
    }
    else if(command->command_id == 'A'){
        if(command->value != 0){
            this->avg_samples = command->value;
        }
    }
}

void ModuleVoltmeter::dataShuffle(){
    if(this->fixedVDDA != 0) return;
    /* Re-shuffle for devices where VREFINT is not last channel */
    if(vrefint_real_index != this->num_channels-1){
        int16_t tmp = this->read_buffer[vrefint_real_index];
        for(int i = vrefint_real_index;i < this->num_channels-1;++i){
            this->read_buffer[i] = this->read_buffer[i+1];
        }
        this->read_buffer[this->num_channels-1] = tmp;
    }
}

void ModuleVoltmeter::thread(comm_t* comm){
    if(this->state == SAMPLING){
        int i;
        uint32_t tick = get_ms_ticks();
        if((tick - this->prevTick) > VOLT_SAMPLE_PERIOD_MS){
            adc.readData(this->read_buffer);

            this->dataShuffle();

            if(this->fixedVDDA == 0){
                for(i = 0;i < this->num_channels-1;++i){
                    int32_t value = this->read_buffer[i];
                    value *= this->scale;
                    value *= VREFINT_CAL;
                    value /= this->read_buffer[this->num_channels-1];
#if VREFINT_CAL_MV != 3300
                    /* TODO: report VREFINT_CAL_MV to PC application and do adjustments there */
                    value *= VREFINT_CAL_MV;
                    value /= 3300;
#endif
                    this->accum_registers[i] += value;
                }
                this->accum_registers[this->num_channels-1] +=
                    (int32_t)((VREFINT_CAL_MV * VREFINT_CAL) / this->read_buffer[this->num_channels-1]);
            }
            else {
                for(i = 0;i < this->num_channels;++i){
                    int32_t value = this->read_buffer[i] * this->fixedVDDA;
                    value *= this->scale;
                    value /= 4096;
                    this->accum_registers[i] += value;
                }
            }

            this->accum_count++;
            if(this->accum_count >= this->avg_samples){
                this->state = TRANSFER;
            }

            if((tick - this->prevTick) > (VOLT_SAMPLE_PERIOD_MS*10)){
                this->prevTick = tick;
            }
            else {
                this->prevTick = this->prevTick + VOLT_SAMPLE_PERIOD_MS;
            }
        }
    }

    if(this->state == TRANSFER){
        int i;
        send_binary_data_header(comm,this->channel,4 * this->num_channels
            + ((this->fixedVDDA == 0)?2:6));
        comm_write(comm,(char*)&this->accum_count,2);
        for(i = 0;i < this->num_channels;++i){
            comm_write(comm,(char*)&this->accum_registers[i],4);
        }
        if(this->fixedVDDA != 0){
            uint32_t dummy_vdda = this->fixedVDDA * this->accum_count;
            comm_write(comm,(char*)&dummy_vdda,4);
        }
        this->state = SAMPLING;
        this->accum_count = 0;
        memset(this->accum_registers,0,this->num_channels*4);
    }
}

uint32_t ModuleVoltmeter::getVDDA(){
    uint32_t value = VREFINT_CAL_MV;
    adc.readData(this->read_buffer);

    this->dataShuffle();

    value *= VREFINT_CAL;
    value /= this->read_buffer[this->num_channels-1];

    return value;
}
