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
#include "oscilloscope.h"
#include "mem.h"
#include <string.h>

static uint8_t get_trigger_channel(uint8_t channel,uint32_t channel_mask){
    if(channel_mask & (0x1 << channel)){
        uint8_t i,result = 0;
        for(i = 0;i < channel;++i){
            if(channel_mask & (0x1 << i)){
                result++;
            }
        }
        return result;
    }
    else {
        return 0;
    }
}

template <typename T, int data_shift>
void osc_callback_bind(void *_handle,uint8_t* data,uint32_t size){
    ModuleOscilloscope* osc = (ModuleOscilloscope*)_handle;
    osc->process<T,data_shift>(data,size);
}

ModuleOscilloscope::ModuleOscilloscope(circular_buffer_t* circular_buffer,
    int channel,int sample_bits,int num_channels){

    this->circular_buffer = circular_buffer;
    this->channel_mask = (0x1 << num_channels) - 1;

    this->trigger_level = 0;
    this->stop_offset = 0;
    this->osc_flags = (Flags)0;
    this->trigger_channel = 0;
    this->trigger_channel_orig = 0;
    this->record_length = 1024;
    this->num_channels = num_channels;
    this->max_frequency = 0;
    this->max_frequency_scale = false;
    /* Ceil to bytes */
    this->sample_size = (sample_bits + 7) / 8;
    this->transfer_size = circular_buffer->buffer_size / this->sample_size;
    this->state = STATE_IDLE;

    this->pretrigger = ((this->transfer_size / num_channels) >> 2);
    this->pretrigger_index = 0;
    this->pretrigger_permiles = 500;
    this->trigger_polarity = TRIGGER_RISING_EDGE;
    this->channel = channel;
    this->frame_counter = 0;

    this->circular_buffer->process_arg = this;
    if(this->sample_size == 1){
        this->circular_buffer->process_callback = osc_callback_bind<uint8_t,0>;
    }
    else if(this->sample_size == 2){
        this->circular_buffer->process_callback = osc_callback_bind<uint16_t,1>;
    }
    else if(this->sample_size <= 4){
        this->circular_buffer->process_callback = osc_callback_bind<uint32_t,2>;
    }

    this->recomputeBufferSize();

    /* TODO: get initial sampling frequency */
    this->trigger_skip = 1;
}

ModuleOscilloscope::~ModuleOscilloscope(){
    buffer_deinit(this->circular_buffer);
}

int ModuleOscilloscope::start(){
    this->state = STATE_RUNNING;
    buffer_start_rw(this->circular_buffer);
    return ERROR_NONE;
}

void ModuleOscilloscope::resume(){
    this->state = STATE_RUNNING;
    this->pretrigger_index = 0;

    buffer_start_rw(this->circular_buffer);
}

void ModuleOscilloscope::stop(){
    this->state = STATE_IDLE;
    buffer_stop_rw(this->circular_buffer);
}

void ModuleOscilloscope::send(comm_t* comm){
    uint32_t buffer_size = this->sample_size * this->transfer_size;
    uint32_t offset = this->sample_size * this->stop_offset;
    char buff_format;
    int i;
    int buff_count = buffer_get_count(this->circular_buffer);
    int header_size = buff_count+1;

    if(offset >= buffer_size) offset = 0;

    if(this->circular_buffer->additional_buffer){
        send_binary_data_header(comm,this->channel,buffer_size+header_size+
            this->circular_buffer->additional_buffer->buffer_size);
    }
    else {
        send_binary_data_header(comm,this->channel,buffer_size+header_size);
    }

    buff_format = buff_count;
    comm_write(comm,&buff_format,1);
    buff_format = BUFFER_FORMAT(this->num_channels, this->sample_size,
        BUFFER_TYPE_NUMBER);
    comm_write(comm,&buff_format,1);
    for(i =1;i < buff_count;++i){
        buff_format = BUFFER_FORMAT(1, 1, BUFFER_TYPE_BITS);
        comm_write(comm,&buff_format,1);
    }

    comm_write(comm,(char*)this->circular_buffer->buffer+offset,buffer_size - offset);
    comm_write(comm,(char*)this->circular_buffer->buffer,offset);

    if(this->circular_buffer->additional_buffer){
        buffer_size = this->circular_buffer->additional_buffer->buffer_size;
        offset = this->stop_offset / this->num_channels;
        comm_write(comm,(char*)this->circular_buffer->additional_buffer->buffer+offset,buffer_size - offset);
        comm_write(comm,(char*)this->circular_buffer->additional_buffer->buffer,offset);
    }
}

void ModuleOscilloscope::command(comm_t* comm,const command_t* command){
    if(command->channel != this->channel){
        return;
    }
    if(command->command_id == 'T'){
        this->trigger_level = command->value;
    }
    else if(command->command_id == 'S'){
        if(command->value == 1){
            this->osc_flags |= (Flags)FLAG_RUNNING;
        }
        else {
            this->osc_flags &= (Flags)~FLAG_RUNNING;
        }
        if(command->value &&
            (this->state == STATE_IDLE)){
            this->start();
        }
    }
    else if(command->command_id == 'F'){
        uint32_t base_frequency, divider, input_impedance;
        uint32_t sample_rate = convert_samplerate(command->value);
        uint32_t ts;

        this->sample_rate = sample_rate;

        buffer_stop_rw(this->circular_buffer);
        if(this->max_frequency > 0){
            if(this->max_frequency_scale){
                if((sample_rate * this->num_channels) > this->max_frequency){
                    sample_rate = this->max_frequency / this->num_channels;
                }
            }
            else {
                if(sample_rate > this->max_frequency){
                    sample_rate = this->max_frequency;
                }
            }
        }
        ts = ((uint64_t)sample_rate * OSC_INSTRUCTIONS_PER_SAMPLE) / get_core_clock();
        ts += 1;
        if(ts > OSC_MAX_SAMPLE_SKIP) ts = OSC_MAX_SAMPLE_SKIP;
        this->trigger_skip =ts;

        base_frequency = buffer_change_samplerate(this->circular_buffer, sample_rate, &divider);
        send_command(comm, this->channel, 'F',base_frequency);
        send_command(comm, this->channel, 'D',divider);
        if(buffer_get_attribute(this->circular_buffer, BUFFER_ATTRIBUTE_MAX_IMPEDANCE, &input_impedance) == 0){
            send_command(comm, this->channel, 'R', input_impedance);
        }

        this->state = STATE_IDLE;
        if(this->osc_flags & FLAG_RUNNING){
            this->frame_counter = 0;
            this->resume();
        }
    }
    else if(command->command_id == 'P'){
        this->trigger_polarity = (TriggerPolarity)command->value;
    }
    else if(command->command_id == 'D'){
        this->pretrigger_permiles = command->value;
        uint32_t newValue = (this->transfer_size * this->pretrigger_permiles) / 1000;
        newValue /= this->num_channels;
        if(newValue < this->transfer_size){
            this->pretrigger = newValue;
        }
    }
    else if(command->command_id == 'M'){
        if(command->value){
            this->osc_flags |= (Flags)FLAG_AUTO_TRIGGER;
        }
        else {
            this->osc_flags &= (Flags)~FLAG_AUTO_TRIGGER;
        }
    }
    else if(command->command_id == 'B'){
        buffer_stop_rw(this->circular_buffer);
        this->state = STATE_IDLE;
        this->record_length = command->value;
        this->recomputeBufferSize();
        if(this->osc_flags & FLAG_RUNNING){
            this->frame_counter = 1;
            this->resume();
        }
    }
    else if(command->command_id == 'C'){
        if(command->value > 0){
            uint32_t input_impedance;
            this->setChannelMask(command->value,comm);
            if(buffer_get_attribute(this->circular_buffer, BUFFER_ATTRIBUTE_MAX_IMPEDANCE, &input_impedance) == 0){
                send_command(comm, this->channel, 'R', input_impedance);
            }
        }
    }
    else if(command->command_id == 'R'){
        this->trigger_channel_orig = command->value;
        this->trigger_channel = get_trigger_channel(this->trigger_channel_orig, this->channel_mask);
    }
}

void ModuleOscilloscope::recomputeBufferSize(){
    uint32_t newSize = this->record_length * this->num_channels * this->sample_size;
    uint32_t newPretriggerValue;
    uint32_t remain;
    if(newSize > this->circular_buffer->allocated_size){
        newSize = this->circular_buffer->allocated_size;
    }
    /* Check if buffer alignment is correct */
    remain = (newSize)
        % (this->circular_buffer->item_size * 2);
    newSize -= remain;
    buffer_resize(this->circular_buffer, newSize);
    this->transfer_size = this->circular_buffer->buffer_size / this->sample_size;
    newPretriggerValue = (this->transfer_size * this->pretrigger_permiles) / 1000;
    newPretriggerValue /= this->num_channels;
    if(newPretriggerValue < this->transfer_size){
        this->pretrigger = newPretriggerValue;
    }
}

void ModuleOscilloscope::setChannelMask(uint32_t value,comm_t* comm){
    buffer_stop_rw(this->circular_buffer);
    this->num_channels = buffer_set_channels(this->circular_buffer, value);
    this->trigger_channel = get_trigger_channel(this->trigger_channel_orig, value);

    this->recomputeBufferSize();

    if(this->max_frequency > 0){
        uint32_t base_frequency, divider, input_impedance;
        uint32_t sample_rate = this->sample_rate;
        if(this->max_frequency_scale){
            if((sample_rate * this->num_channels) > this->max_frequency){
                sample_rate = this->max_frequency / this->num_channels;
            }
        }
        else {
            if(sample_rate > this->max_frequency){
                sample_rate = this->max_frequency;
            }
        }
        if(comm){
            base_frequency = buffer_change_samplerate(this->circular_buffer, sample_rate, &divider);
            send_command(comm, this->channel, 'F',base_frequency);
            send_command(comm, this->channel, 'D',divider);
            if(buffer_get_attribute(this->circular_buffer, BUFFER_ATTRIBUTE_MAX_IMPEDANCE, &input_impedance) == 0){
                send_command(comm, this->channel, 'R', input_impedance);
            }
        }
    }
    

    this->state = STATE_IDLE;
    if(this->osc_flags & FLAG_RUNNING){
        this->frame_counter = 0;
        this->resume();
    }
}

void ModuleOscilloscope::thread(comm_t* comm){
    if(this->state == STATE_TRIGGERED){
        if(buffer_get_stop_state(this->circular_buffer,&this->stop_offset)){
            this->state = STATE_TRANSFER;
        }
    }

    if(this->state == STATE_TRANSFER) {
        this->send(comm);
        if(this->osc_flags & FLAG_RUNNING){
            this->frame_counter = 1;
            this->resume();
        }
        else {
            this->stop();
        }
    }
}

void ModuleOscilloscope::setMaxFrequency(uint32_t value){
    this->max_frequency = value;
}

void ModuleOscilloscope::setMaxFrequencyScale(bool value){
    this->max_frequency_scale = value;
}