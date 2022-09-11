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
 * \file oscilloscope.h
 * \brief Module for recording mixed analog/digital signals
 *
 * Oscilloscope is advanced measuring module, that
 * tries to emulate classic oscilloscopes.
 * It enables to set trigger, number of channels,
 * sampling frequency and number of samples.
 *
 * Main function of this module is to search for a trigger
 * in measured data stream and stop recording in the right time.
 *
 * Module uses circular_buffer_t API. This enables interfacing
 * the module with different peripherals, altough it is currently implemented
 * for internal ADC only.
 * It also enables to implent synchronous digital channels,
 * which results in mixed digital/analog oscilloscope.
 */
#ifndef _MODULE_OSCILLOSCOPE_H_
#define _MODULE_OSCILLOSCOPE_H_

#include "circular_buffer.h"

#ifndef OSC_INSTRUCTIONS_PER_SAMPLE
#define OSC_INSTRUCTIONS_PER_SAMPLE 1000
#endif

#ifndef OSC_MAX_SAMPLE_SKIP
#define OSC_MAX_SAMPLE_SKIP     16
#endif

#define OSC_DEFAULT_HINT    (ADC_HINT_FAST)

#define OSC_DECLARE(name,precision,num_channels,buffer_size) \
    OSC_DECLARE_HINT(name,precision,num_channels,buffer_size, OSC_DEFAULT_HINT)

#define OSC_DECLARE_HINT(name,precision,num_channels,buffer_size, hint) \
    const static adc_init_t name ## _adc_init_data = {\
        10000, \
        precision, \
        num_channels, \
        buffer_size, \
        hint \
    }; \
    static PeriphDefaultADC* name ## _adc_handle = NULL; \
    static adc_channel_t name ## _adc_channels[num_channels]; \
    static ModuleOscilloscope* name ## _module

#define OSC_ADD_CHANNEL(name,pin,index) do { \
        int __tmp__dummy; \
        adc_find_config(pin, &__tmp__dummy, &name ## _adc_channels[index]); \
    } while(0)

#define OSC_MODULE_PREPARE(name,adc_id) do { \
        name ## _adc_handle = new PeriphDefaultADC(adc_id, &name ## _adc_init_data, name ## _adc_channels); \
    }while(0)

#define OSC_MODULE_GET_BUFFER(name) name ## _adc_handle->getCircularBuffer()

#define OSC_MODULE_INIT(name) do { \
        name ## _module = new ModuleOscilloscope(name ## _adc_handle->getCircularBuffer(), \
            MODULE_CHANNEL_OSC, name ## _adc_init_data.data_size, name ## _adc_init_data.num_channels); \
        name ## _module->setChannelMask( (1 << name ## _adc_init_data.num_channels) - 1, NULL); \
    }while(0)

#define OSC_MODULE_LIMIT_FREQUENCY(name,value) do { \
        name ## _module->setMaxFrequency(value); \
    }while(0)

#define OSC_MODULE_LIMIT_FREQUENCY_SCALE(name,value) do { \
        name ## _module->setMaxFrequencyScale(value); \
    }while(0)

#define OSC_MODULE_DEINIT(name) do { \
        if(name ## _adc_handle) { \
            delete name ## _adc_handle; \
            name ## _adc_handle = NULL; \
        } \
    }while(0)

class ModuleOscilloscope : public Module {
protected:
    /** \brief Specifies oscilloscope current state
     *
     * This doesn't include if the oscilloscope should stop after
     * current capture.
     */
    enum State{
        /** \brief Oscilloscope is disabled */
        STATE_IDLE,
        /** \brief Oscilloscope was enabled and is recording samples*/
        STATE_RUNNING,
        /** \brief Oscilloscope received enough samples for pre-trigger */
        STATE_PRETRIGGERED,
        /** \brief Oscilloscope detected trigger and is waiting for recording to finish */
        STATE_TRIGGERED,
        /** \brief Oscilloscope received all necesery samples
         * and is transmitting them to PC
         */
        STATE_TRANSFER,
    };

    /** \brief Specifies osciloscope trigger edge polarity */
    enum TriggerPolarity {
        /** \brief Oscilloscope reacts on rising edge only */
        TRIGGER_RISING_EDGE = 0x1,
        /** \brief Oscilloscope reacts on falling edge only */
        TRIGGER_FALLING_EDGE = 0x2,
        /** \brief Oscilloscope reacts on both edges */
        TRIGGER_BOTH_EDGES = 0x3,
    };

    /** \brief Specifies oscilloscope flags which
     * are affecting oscilloscopes behaviour
     */
    enum Flags{
        /** \brief If flag is set, oscilloscope triggers itself immediatelly */
        FLAG_AUTO_TRIGGER = 0x1,
        /** \brief If flag is set, oscilloscope captures data continously */
        FLAG_RUNNING = 0x2,
    };
    /** \brief Circular buffer for reading the samples
     *
     * This is where all the samples are stored.
     * It is interface between oscilloscope (as a trigger-finding algorithm)
     * and e.g. ADC (source of data).
     */
    circular_buffer_t* circular_buffer;
    /** \brief Current state */
    volatile State state;
    /** \brief Trigger level voltage */
    uint32_t trigger_level;
    /** \brief Size of sample in bytes */
    uint8_t sample_size;
    /** \brief Size of buffer in samples */
    uint16_t transfer_size;
    /** \brief Number of samples for pretrigger */
    uint16_t pretrigger;
    /** \brief Trigger position in permiles */
    uint16_t pretrigger_permiles;
    /** \brief Current number of samples in pretrigger buffer */
    uint16_t pretrigger_index;
    /** \brief Flags affecting oscilloscopes behaviour */
    Flags osc_flags;
    /** \brief Channel used for communication with PC application */
    int channel;
    /** \brief Index of the last sample */
    uint32_t stop_offset;
    /** \brief Mask of active channels */
    uint32_t channel_mask;
    /** \brief Maximum allowed frequency, 0 means no limit */
    uint32_t max_frequency;
    /** \brief If true, maximum frequency scales with channels. E.g. 1 channels -> 1MS max, 2 channels -> 500kS max */
    bool max_frequency_scale;
    uint32_t sample_rate;
    uint32_t record_length;
    /** \brief Trigger polarity */
    TriggerPolarity trigger_polarity;
    uint16_t frame_counter;
    /** \brief Index of trigger channel in data stream */
    uint8_t trigger_channel;
    /** \brief Real number of channel for trigger */
    uint8_t trigger_channel_orig;
    /** \brief Number of channels */
    uint8_t num_channels;
    /** \brief Number of samples to be skipped when searching for trigger
     *
     * This is useful for reducing the CPU load on high frequencies.
     */
    uint8_t trigger_skip;

    /** \brief Starts the oscilloscope and sets the OSC_FLAG_RUNNING flag
     */
    int start();
    /** \brief Resumes the oscilloscope function
     */
    void resume();
    /** \brief Stops the oscilloscope
     */
    void stop();
    /** \brief Sends measured data to PC application
     * \param comm Communication interface where to send the data
     */
    void send(comm_t* comm);
    void recomputeBufferSize();
public:
    ModuleOscilloscope(circular_buffer_t* circular_buffer,
        int channel,int sample_bits,int num_channels);
    virtual void command(comm_t* comm, const command_t* command);
    virtual void thread(comm_t* comm);
    virtual ~ModuleOscilloscope();

    void setChannelMask(uint32_t value, comm_t* comm);
    void setMaxFrequency(uint32_t value);
    void setMaxFrequencyScale(bool value);

    template <typename T, int data_shift>
    void process(uint8_t* _data,uint32_t _size){
        const T* data = (const T*)_data;
        uint32_t size = (_size >> data_shift);
        uint32_t i = 0;
        uint32_t callback_offset = (_data - this->circular_buffer->buffer) >> data_shift;

        if(this->state == STATE_RUNNING){
            if((this->pretrigger - this->pretrigger_index) < (size / this->num_channels)){
                this->state = STATE_PRETRIGGERED;
                i = (this->pretrigger - this->pretrigger_index) * this->num_channels;
            }
            else {
                this->pretrigger_index += size / this->num_channels;
            }
        }

        if(this->state == STATE_PRETRIGGERED){
            i+=this->trigger_channel;
            if(i >= size){
                return;
            }
            int old_comp = data[i] < this->trigger_level;
            uint32_t i_add = this->num_channels * this->trigger_skip;
            /* TODO: move auto-trigger outside ?? */
            for(;i < size;i+=i_add){
                int new_comp = data[i] < this->trigger_level;
                /* TODO: write filter */
                if(((new_comp != old_comp) && (((this->trigger_polarity & TRIGGER_RISING_EDGE) && old_comp && !new_comp) ||
                    ((this->trigger_polarity & TRIGGER_FALLING_EDGE) && !old_comp && new_comp))) ||
                    (this->osc_flags & FLAG_AUTO_TRIGGER)){

                    /* Search for trigger value */
                    if(i_add != this->num_channels && !(this->osc_flags & FLAG_AUTO_TRIGGER)){
                        uint32_t real_i = i - i_add;
                        for(;real_i < i;real_i+=this->num_channels){
                            old_comp = data[real_i] < this->trigger_level;
                            if(old_comp == new_comp)
                                break;
                        }
                        i = real_i;
                    }

                    int32_t sample_index = (int32_t)((i + callback_offset + this->transfer_size) / this->num_channels) - this->pretrigger;
                    if(sample_index >= (this->transfer_size / this->num_channels)){
                        sample_index -= (this->transfer_size / this->num_channels);
                    }
                    else if (sample_index < 0){
                        sample_index += (this->transfer_size / this->num_channels);
                    }

                    this->stop_offset = sample_index;
                    buffer_stop_precise_rw(this->circular_buffer, (uint32_t)sample_index);

                    this->state = STATE_TRIGGERED;
                    return;
                }

                old_comp = new_comp;
            }
        }
    }

    friend Flags operator|(Flags a,Flags b);
    friend void operator|=(Flags &a, Flags b);
    friend Flags operator&(Flags a,Flags b);
    friend void operator&=(Flags &a, Flags b);
};

inline ModuleOscilloscope::Flags operator|(ModuleOscilloscope::Flags a, ModuleOscilloscope::Flags b){
    return (ModuleOscilloscope::Flags)((int)(a) | (int)(b));
}

inline void operator|=(ModuleOscilloscope::Flags &a, ModuleOscilloscope::Flags b){
    a = (ModuleOscilloscope::Flags)((int)(a) | (int)(b));
}

inline ModuleOscilloscope::Flags operator&(ModuleOscilloscope::Flags a, ModuleOscilloscope::Flags b){
    return (ModuleOscilloscope::Flags)((int)(a) & (int)(b));
}

inline void operator&=(ModuleOscilloscope::Flags &a, ModuleOscilloscope::Flags b){
    a = (ModuleOscilloscope::Flags)((int)(a) & (int)(b));
}

#endif /* _MODULE_OSCILLOSCOPE_H_ */
