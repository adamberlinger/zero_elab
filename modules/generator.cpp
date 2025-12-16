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
#include "generator.h"


#include "generator_sin_table.h"

static int is_pow2(uint32_t x){
    return (x > 0) && !(x & (x-1));
}

static int get_pow2_shift(uint32_t source,uint32_t dest){
    if(source > dest){
        uint32_t x = source / dest;
        int result = 0;
        while(x > 1){
            x >>=1;
            result++;
        }
        return -result;
    }
    else {
        uint32_t x = dest / source;
        int result = 0;
        while(x > 1){
            x >>=1;
            result++;
        }
        return result;
    }
}

template <typename T, int data_shift>
void gen_callback_bind(void *_handle,uint8_t* data,uint32_t size){
    ModuleGenerator* gen = (ModuleGenerator*)_handle;
    gen->process<T,data_shift>(data,size);
}

#ifdef DAC_PERIPH_ENABLED

ModuleDACOutput::ModuleDACOutput(int channel, PeriphDefaultDAC* dac, int resolution){
    this->channel = channel;
    this->dac = dac;
    this->bit_shift = SAMPLE_TABLE_RESOLUTION - resolution;
}


void ModuleDACOutput::command(comm_t* comm, const command_t* command){
    if(this->channel != command->channel){
        return;
    }

    if(command->command_id == 'V'){
        dac->setLevel(command->value >> this->bit_shift);
    }
}

#endif

ModuleGenerator::ModuleGenerator(int channel, circular_buffer_t* circular_buffer,
    uint32_t sample_rate,uint32_t table_size,uint8_t bits){

    uint32_t i,index;
    uint16_t *output_buffer;
    uint16_t output_size;

    this->table_bit_shift = SAMPLE_TABLE_RESOLUTION - bits;

    if(!is_pow2(table_size) && table_size){
        return;
    }

    this->circular_buffer = circular_buffer;

    this->forced_value_valid = 0;
    this->shape = SHAPE_SIN;
    if(table_size){
        this->pattern_table = (uint16_t*)module_malloc(2*table_size);
        if(!this->pattern_table){
            return;
        }
    }
    else {
        this->pattern_table = NULL;
    }
    this->pattern_size = table_size;

    this->accum_phase = 0;
    this->phase_addend = 0;
    this->channel = channel;
    this->disable_fix_value = 0;

    this->sample_rate = sample_rate;
    this->scale = 0x10000;
    this->offset = 0;

    this->running = 0;

    if(this->pattern_table){
        int shift = get_pow2_shift(SAMPLE_TABLE_SIZE,table_size);
        this->pattern_size_shift = (32-(SAMPLE_TABLE_SIZE_BITS+shift));
        this->circular_buffer->process_arg = this;
        if(bits <= 8){
            this->circular_buffer->process_callback = gen_callback_bind<uint8_t,0>;
        }
        else {
            this->circular_buffer->process_callback = gen_callback_bind<uint16_t,1>;
        }
        this->computeTable();

        /* TODO: replace with callback */
        /* TODO: rewrite for different sample size */
        output_buffer = (uint16_t*)circular_buffer->buffer;
        output_size = circular_buffer->buffer_size / 2;
        for(i = 0;i < output_size;++i){
            this->accum_phase += this->phase_addend;
            index = (this->accum_phase >> this->pattern_size_shift);
            output_buffer[i] = this->pattern_table[index];
        }
    }
    else {
        this->setFrequency(1000);
    }
}

void ModuleGenerator::ignoreDCComands(){
    this->disable_fix_value = 1;
}

int ModuleGenerator::start(){
    buffer_start_rw(this->circular_buffer);
    return ERROR_NONE;
}

int ModuleGenerator::stop(){
    buffer_stop_rw(this->circular_buffer);
    return ERROR_NONE;
}

void ModuleGenerator::setFrequency(uint32_t frequency){
    if(this->pattern_table){
        /* DDS algorithm */
        uint64_t addend = ((uint64_t)frequency << 32);
        addend /= this->sample_rate;
        if(addend > 0x3FFFFFFF)
            addend = 0x3FFFFFFF;
        else if(addend == 0)
            addend = 1;
        this->phase_addend = addend;
    }
    else {
        /* Fixed buffer algorithm */
        /* Frequency = TIM sample rate / buffer size */
        uint32_t new_size = 8;
        uint32_t max_size = this->circular_buffer->allocated_size / this->circular_buffer->item_size;
        uint32_t tim_freq = frequency * new_size;
        uint32_t tim_prescaler;

        if(this->running){
            this->stop();
        }

        this->sample_rate = buffer_change_samplerate(this->circular_buffer, tim_freq, &tim_prescaler);

        if(tim_prescaler > 2){
            /* prescaler set inside TIM -> we can increase buffer */
            new_size *= (tim_prescaler / 2);
            if(new_size > max_size) new_size = max_size;

            tim_freq = frequency * new_size;
            this->sample_rate = buffer_change_samplerate(this->circular_buffer, tim_freq, &tim_prescaler);
        }
        this->phase_addend = tim_prescaler * new_size;

        this->pattern_size = new_size;

        this->computeTable();

        buffer_resize(this->circular_buffer, new_size * this->circular_buffer->item_size);

        if(this->running){
            this->start();
        }
    }
}

void ModuleGenerator::setScale(uint16_t _scale){
    uint32_t scale = _scale + 1;
    this->scale = scale;
    this->computeTable();
}

void ModuleGenerator::setOffset(uint16_t offset){
    this->offset = offset;
    this->computeTable();
}

void ModuleGenerator::setShape(GeneratorShape shape){
    this->shape = shape;
    this->computeTable();
}

void ModuleGenerator::command(comm_t* comm,const command_t* command){
    if(this->channel != command->channel){
        return;
    }

    if(command->command_id == 'S'){
        if(command->value){
            this->start();
            this->running = 1;
        }
        else {
            this->stop();
            this->running = 0;
        }
    }
    else if(command->command_id == 'P'){
        this->setShape((GeneratorShape)command->value);
    }
    else if(command->command_id == 'F'){
        uint32_t frequency = convert_samplerate(command->value);
        this->setFrequency(frequency);
    }
    else if(command->command_id == 'R'){
        if(this->pattern_table){
            uint32_t val = command->value * (1000000 / this->sample_rate);
            this->phase_addend = (uint32_t)val;
        }
        else {
            uint64_t temp = ((uint64_t)command->value * 1000000);
            this->setFrequency(temp >> 32);
            send_command(comm, this->channel, 'F', this->sample_rate);
            send_command(comm, this->channel, 'D', this->phase_addend);
        }
    }
    else if(command->command_id == 'A'){
        this->setScale(command->value);
    }
    else if(command->command_id == 'O'){
        this->setOffset(command->value >> this->table_bit_shift);
    }
    else if(this->disable_fix_value == 0){
        if(command->command_id == 'V'){
            this->forced_value = command->value >> this->table_bit_shift;
            this->forced_value_valid = 1;
        }
        else if(command->command_id == 'G'){
            this->forced_value_valid = 0;
        }
    }
}

void ModuleGenerator::computeTable(){
    uint32_t i;
    uint32_t pattern_halfsize = (this->pattern_size / 2);
    int shift = (this->pattern_table)?get_pow2_shift(this->pattern_size,SAMPLE_TABLE_SIZE):0;
    this->forced_value_valid = 0;

    uint16_t* output_buffer = (this->pattern_table)?this->pattern_table:(uint16_t*)this->circular_buffer->buffer;

    if(this->shape == SHAPE_NOISE){
        return;
    }

    for(i = 0; i < this->pattern_size;++i){
        uint32_t sample = 0;
        if(this->shape == SHAPE_SIN){
            uint32_t table_index;
            if(this->pattern_table){
                table_index = (shift > 0)?(i << shift):(i >> (-shift));
            }
            else {
                table_index = (i * SAMPLE_TABLE_SIZE) / this->pattern_size;
            }
            sample = sin_samples[table_index];
        }
        else if(this->shape == SHAPE_TRIANGLE){
            if(i >= pattern_halfsize){
                sample = (0xFFFF * (this->pattern_size - i)) / pattern_halfsize;
            }
            else {
                sample = (0xFFFF * i) / pattern_halfsize;
            }
        }
        else if(this->shape == SHAPE_SQUARE){
            sample = (i >= (this->pattern_size/2))?0xFFFF:0;
        }
        else if(this->shape == SHAPE_SAW){
            sample = (0xFFFF * i) / this->pattern_size;
        }

        sample = sample  >> this->table_bit_shift;

        output_buffer[i] = ((sample * this->scale) >> 16) + this->offset;
    }
}
