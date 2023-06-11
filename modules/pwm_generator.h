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
 * \file pwm_generator.h
 * \brief Module for generating analog signals through PWM modulation
 *
 * Generator is advanced module used for generating signals.
 * It uses DDS (direct digital synthesis) to generate samples.
 * It allows generation of different signal shapes.
 * External RC circuit must be connected to produce analog voltage.
 *
 * Module can be also used as an DC voltage source (constant signal).
 */
#ifndef _PWM_GENERATOR_H_
#define _PWM_GENERATOR_H_

#include "circular_buffer.h"
#include "generator_utils.h"

#include "timer.h"

class ModulePWMGenerator : public Module {
protected:
    /** \brief Timer handle generating PWM */
    timer_handle_t* timer_handle;

    /** \brief PWM frequency */
    uint32_t pwm_frequency;

    /** \brief Sample rate of the output stream */
    uint32_t sample_rate;
    /** \brief Size of pattern_table in samples */
    uint16_t pattern_size;

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
    GeneratorShape shape;

    int start();
    int stop();
    void setFrequency(uint32_t frequency);
    void setScale(uint16_t scale);
    void setShape(GeneratorShape shape);
    void setOffset(uint16_t offset);
    void computeTable();
public:
    ModulePWMGenerator(int channel, timer_handle_t* timer_handle,
        uint32_t sample_rate);
    virtual void command(comm_t* comm, const command_t* command);
};

#endif /* _PWM_GENERATOR_H_ */