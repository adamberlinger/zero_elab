#include "pwm_generator.h"
#include "generator_sin_table.h"

static int is_pow2(uint32_t x){
    return (x > 0) && !(x & (x-1));
}

ModulePWMGenerator::ModulePWMGenerator(int channel, timer_handle_t* timer_handle,
    uint32_t sample_rate){

    /* Forced to 8-bit at the moment */
    uint8_t bits = 8;

    this->table_bit_shift = SAMPLE_TABLE_RESOLUTION - bits;

    if(timer_handle->circular_buffer == NULL){
        return;
    }

    this->pattern_size = timer_handle->circular_buffer->buffer_size / 2;

    if(!is_pow2(this->pattern_size)){
        return;
    }

    this->timer_handle = timer_handle;

    this->forced_value_valid = 0;
    this->shape = SHAPE_SIN;

    this->channel = channel;

    this->pwm_frequency = (timer_get_clocks(this->timer_handle) >> 8);

    this->sample_rate = sample_rate;
    this->scale = 0x10000;
    this->offset = 0;

    this->timer_handle->circular_buffer->process_arg = NULL;
    this->timer_handle->circular_buffer->process_callback = NULL;

    this->computeTable();
}

int ModulePWMGenerator::start(){
    buffer_start_rw(this->timer_handle->circular_buffer);
    return ERROR_NONE;
}

int ModulePWMGenerator::stop(){
    buffer_stop_rw(this->timer_handle->circular_buffer);
    return ERROR_NONE;
}

void ModulePWMGenerator::setFrequency(uint32_t frequency){
    uint32_t prescaler = ((this->pwm_frequency / this->pattern_size) / frequency) - 1;
    if(prescaler > 255) prescaler = 255;
    timer_target_set_repetitioncounter(this->timer_handle, prescaler);
}

void ModulePWMGenerator::setScale(uint16_t _scale){
    uint32_t scale = _scale + 1;
    this->scale = scale;
    this->computeTable();
}

void ModulePWMGenerator::setOffset(uint16_t offset){
    this->offset = offset;
    this->computeTable();
}

void ModulePWMGenerator::setShape(GeneratorShape shape){
    this->shape = shape;
    this->computeTable();
}

void ModulePWMGenerator::command(comm_t* comm,const command_t* command){
    if(this->channel != command->channel){
        return;
    }

    if(command->command_id == 'S'){
        if(command->value){
            this->start();
        }
        else {
            this->stop();
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
        uint64_t temp = ((uint64_t)command->value * 1000000);
        this->setFrequency(temp >> 32);
        send_command(comm, this->channel, 'F',
            this->pwm_frequency);
        send_command(comm, this->channel, 'D',
            this->pattern_size * (timer_target_get_repetitioncounter(this->timer_handle)+1));
    }
    else if(command->command_id == 'A'){
        this->setScale(command->value);
    }
    else if(command->command_id == 'O'){
        this->setOffset(command->value >> this->table_bit_shift);
    }
    else if(command->command_id == 'V'){
        this->forced_value = command->value >> this->table_bit_shift;
        this->forced_value_valid = 1;
        this->computeTable();
    }
    else if(command->command_id == 'G'){
        this->forced_value_valid = 0;
    }
}

void ModulePWMGenerator::computeTable(){
    uint32_t pattern_halfsize = this->pattern_size >> 1;
    uint32_t sample_skip = SAMPLE_TABLE_SIZE / this->pattern_size;
    uint16_t* pattern_table = (uint16_t*)this->timer_handle->circular_buffer->buffer;
    if(forced_value_valid){
        for(uint32_t i = 0;i < this->pattern_size;++i){
            pattern_table[i] = this->forced_value;
        }
    }
    else {
        for(uint32_t i = 0;i < this->pattern_size;++i){
            uint8_t sample = 0;
            if(this->shape == SHAPE_SIN){
                sample = sin_samples[i*sample_skip] >> this->table_bit_shift;
            }
            else if(this->shape == SHAPE_TRIANGLE){
                if(i >= pattern_halfsize){
                    sample = (0xFF * (this->pattern_size - i)) / pattern_halfsize;
                }
                else {
                    sample = (0xFF * i) / pattern_halfsize;
                }
            }
            else if(this->shape == SHAPE_SQUARE){
                sample = (i >= (this->pattern_size/2))?0xFF:0;
            }
            else if(this->shape == SHAPE_SAW){
                sample = (0xFF * i) / this->pattern_size;
            }

            pattern_table[i] = ((sample * this->scale) >> 16) + this->offset;
        }
    }
}