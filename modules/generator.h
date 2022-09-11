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
/**
 * \file generator.h
 * \brief Module for generating analog signals
 *
 * Generator is advanced module used for generating signals.
 * It uses DDS (direct digital synthesis) to generate samples.
 * It allows generation of different signal shapes.
 *
 * Module uses circular_buffer_t API. This enables interfacing
 * the module with different peripherals.
 * E.g. output can be connected to internal DAC, or -- if DAC is not present --
 * it can be connected to timer and generate signal through PDM modulation.
 *
 * Module can be also used as an DC voltage source (constant signal).
 */
#ifndef _GENERATOR_H_
#define _GENERATOR_H_

#include "circular_buffer.h"

#ifndef SAMPLE_TABLE_SIZE
    /** \brief Default value for size of table stored in flash */
    #define SAMPLE_TABLE_SIZE   (4096)
#endif

#ifndef SAMPLE_TABLE_RESOLUTION
    /** \brief Default resolution of table stored in flash */
    #define SAMPLE_TABLE_RESOLUTION (16)
#endif

#ifdef DAC_PERIPH_ENABLED

#include "dac.h"

#define GEN_DECLARE(name,sample_rate,channel,buffer_size) \
    const static dac_init_t name ## _dac_init_data = { \
        sample_rate, \
        channel, \
        buffer_size, \
    }; \
    static ModuleGenerator* name ## _module; \
    static PeriphDefaultDAC* name ## _dac_handle = NULL

#define GEN_MODULE_INIT(name,dac_id) do {\
        name ## _dac_handle = new PeriphDefaultDAC(1, &name ## _dac_init_data); \
        name ## _module = new ModuleGenerator(5, name ## _dac_handle->getCircularBuffer(), \
            name ## _dac_init_data.sample_rate,SAMPLE_TABLE_SIZE,12); \
    }while(0)

#endif /* DAC_PERIPH_ENABLED */

class ModuleGenerator : public Module {
protected:
    enum Shape{
        /** \brief Sine signal shape */
        SHAPE_SIN = 0,
        /** \brief Triangle signal shape (ascending and descending) */
        SHAPE_TRIANGLE = 1,
        /** \brief Square signal shape */
        SHAPE_SQUARE = 2,
        /** \brief This settings generates pseudo-random samples */
        SHAPE_NOISE = 3,
        /** \brief Saw signal shape (ascending and jump to bottom) */
        SHAPE_SAW = 4,
    };

    /** \brief Circular buffer for outputing the samples
     *
     * This is where all the samples are stored.
     * It is interface between generator (as a sample-generating algorithm)
     * and e.g. ADC (sink of data).
     */
    circular_buffer_t* circular_buffer;
    /** \brief Pattern table used by the DDS algorithm */
    uint16_t* pattern_table;

    /** \brief Accumulated phase used by DDS algorithm */
    uint32_t accum_phase;
    /**
     * \brief Ammount which is added to phase in each step
     *
     * This value is proportional to output frequency
     */
    uint32_t phase_addend;
    /** \brief Sample rate of the output stream */
    uint32_t sample_rate;
    /** \brief Size of pattern_table in samples */
    uint16_t pattern_size;
    /**
     * \brief Number of bits, which needs to be shifted from 32-bit index
     * to address the pattern_table
     */
    uint16_t pattern_size_shift;
    /** \brief Channel used for communication with PC application */
    int channel;

    /** \brief Scale applied on output signal */
    uint32_t scale;
    /** \brief Offset applied on output signal */
    uint16_t offset;
    /** \brief Value when in DC output mode */
    uint16_t forced_value;
    /** \brief Non-zero if module is in DC output mode */
    uint8_t forced_value_valid;
    /** \brief Number of bits which need to be shifted from the source table
     * to match output sample bit resolution */
    uint8_t table_bit_shift;
    /** \brief Generated shape of signal
     * (if not overwritten by forced_value_valid) */
    Shape shape;

    int start();
    int stop();
    void setFrequency(uint32_t frequency);
    void setScale(uint16_t scale);
    void setShape(Shape shape);
    void setOffset(uint16_t offset);
    void computeTable();
public:
    ModuleGenerator(int channel, circular_buffer_t* circular_buffer,
        uint32_t sample_rate,uint32_t table_size,uint8_t bits);
    virtual void command(comm_t* comm, const command_t* command);

    template <typename T, int data_shift>
    void process(uint8_t *_data,uint32_t _size){
        T* data = (T*)_data;
        uint32_t size = (_size >> data_shift);

        uint32_t i,index;
        /* TODO: fill table only once */
        if(this->forced_value_valid){
            for(i = 0; i < size;++i){
                data[i] = (T)this->forced_value;
            }
        }
        else if(this->shape == SHAPE_NOISE){
            for(i = 0; i < size;++i){
                data[i] = (T)(rand()) >> this->table_bit_shift;
            }
        }
        else {
            for(i = 0; i < size;++i){
                this->accum_phase += this->phase_addend;
                index = (this->accum_phase >> this->pattern_size_shift);
                data[i] = (T)this->pattern_table[index];
            }
        }
    }
};

#endif /* _GENERATOR_H_ */
