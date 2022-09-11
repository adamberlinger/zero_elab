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
 * \file adc.h
 * \brief API for interfacing analog to digital converters (ADC)
 */
#ifndef _API_ADC_H_
#define _API_ADC_H_

#include "core.h"
#include "circular_buffer.h"
#include "target.h"
#include "gpio.h"


/* This needs to be defined before adc_target.h include */
typedef buffer_process_callback_t adc_read_callback_t;

/**
 * \struct adc_handle_t
 * \brief This structure is platform dependent
 *
 * Please see corrsponding adc_target.h file for implementation.
 * \sa adc_stm32f0_handle_t adc_stm32f3_handle_t
 */
typedef uint8_t adc_channel_t;

#if 0
/** \brief Enum specifying hints for the ADC driver, which may or may not
 * be respected by driver
 */
typedef enum {
    /** \brief No hint */
    ADC_HINT_NONE = 0x00,
    /** \brief Tries to focus the configuration on maximum speed
     *
     * This configuration should be used by oscilloscope module.
     * E.g. on STM32 targets this affects the configured sampling time.
     */
    ADC_HINT_FAST = 0x01,
    /** \brief Reverse SCAN direction for simple ADCs (if they allow it) */
    ADC_HINT_REVERSE_SCAN = 0x2,
}adc_init_hint_t;

/* Workaround for multiple adc_init_hint_t flags */
inline adc_init_hint_t operator|(adc_init_hint_t a, adc_init_hint_t b)
{
    return static_cast<adc_init_hint_t>(static_cast<int>(a) | static_cast<int>(b));
}

inline bool operator&(adc_init_hint_t a, adc_init_hint_t b){
    return static_cast<bool>(static_cast<int>(a) & static_cast<int>(b));
}
#endif
constexpr uint32_t ADC_HINT_NONE = 0x00;
constexpr uint32_t ADC_HINT_FAST = 0x01;
constexpr uint32_t ADC_HINT_REVERSE_SCAN = 0x02;
typedef uint32_t adc_init_hint_t;



/** \brief ADC initialization structure */
typedef struct {
    /** \brief Sample rate in Hz
     *
     * Zero value means triggering by software
     */
    uint32_t sample_rate;
    /** \brief Size of samples in bits */
    uint8_t data_size;
    /** \brief Number of configured channels */
    uint8_t num_channels;
    /** \brief Size of internal buffer in bytes */
    uint32_t buffer_size;
    /** \brief Additional hints, which may affect precision or speed of ADC */
    adc_init_hint_t hints;
}adc_init_t;

template<class BaseADC>
class PeriphADC : public BaseADC {
public:
    /** \brief Initialize ADC peripheral
     * \param adc_id ID specifying the used ADC (platform dependent)
     * \param init_data Configuration of the ADC
     * \param channel_info Array containing configured channels
     */
    PeriphADC(int adc_id,const adc_init_t* init_data,const adc_channel_t* channel_info):
        BaseADC(adc_id,init_data,channel_info){

        circular_buffer_t* circular_buffer = this->getCircularBuffer();
        if(circular_buffer){
            circular_buffer->control_arg = this;
            circular_buffer->stop_callback = (buffer_stop_callback_t)adc_stop;
            circular_buffer->stop_precise_callback = (buffer_stop_precise_callback_t)adc_stop_precise;
            circular_buffer->get_stop_state_callback = (buffer_get_stop_state_callback_t)adc_get_stop_status;
            circular_buffer->start_callback = (buffer_start_callback_t)adc_start_continuous;
            circular_buffer->set_channels_callback = (buffer_set_channels_callback_t)adc_set_channels;
            circular_buffer->get_attribute_callback = (buffer_get_attribute_callback_t)adc_get_attribute;
            circular_buffer->deinit_callback = (buffer_deinit_callback_t)NULL;
        }
    }

    static void adc_stop(PeriphADC<BaseADC> *_this){
        _this->stop();
    }

    static void adc_stop_precise(PeriphADC<BaseADC> *_this, uint32_t sample){
        _this->stopPrecise(sample);
    }

    static int adc_get_stop_status(PeriphADC<BaseADC> *_this, uint32_t *index_out){
        return _this->getStopStatus(index_out);
    }

    static void adc_start_continuous(PeriphADC<BaseADC> *_this){
        _this->startContinuous();
    }

    static uint32_t adc_set_channels(PeriphADC<BaseADC> *_this,uint32_t channel_mask){
        return _this->setChannels(channel_mask);
    }

    static int adc_get_attribute(PeriphADC<BaseADC> *_this, buffer_attribute_t name, void* value_out){
        return _this->getAttribute(name, value_out);
    }

    virtual ~PeriphADC(){}



    /** \brief Read enabled channels by software
     * \param values Pointer to output data. The size of data and alignment depends
     *  on ADC sample size and number of channels
     */
    void readData(void* values){
        BaseADC::readData(values);
    }
    /** \brief Configures callback for periodic reading of data
     * \param callback User callback
     * \param user_arg User argument passed to callback
     */
     void readDataContinuous(adc_read_callback_t callback,void* user_arg){
         circular_buffer_t* circular_buffer = this->getCircularBuffer();
         if(circular_buffer){
             circular_buffer->process_arg = user_arg;
             circular_buffer->process_callback = callback;
         }
         this->startContinuous();
     }
    /** \brief Starts periodic reading of data*/
    void startContinuous(){
        BaseADC::startContinuous();
    }
    /** \brief Stops periodic reading of data */
    void stop(){
        BaseADC::stop();
    }

    int getAttribute(buffer_attribute_t name, void* value_out){
        return BaseADC::getAttribute(name, value_out);
    }
    /** \brief Stops periodic reading of data at precise sample
     * \param sample_number Index of sample where to stop
     *
     * This function is useful for implementing oscilloscope module.
     * This function returns, before the ADC is stopped.
     * Program needs to check via \ref adc_get_stop_status function.
     */
    void stopPrecise(uint32_t sample_number){
        BaseADC::stopPrecise(sample_number);
    }
    /** \brief Gets ADC stop status
     * \param index Index of sample where to ADC really stopped
     * \return non-zero if ADC stopped successfuly
     *
     * This function is useful for implementing oscilloscope module.
     */
    int getStopStatus(uint32_t *index_out){
        return BaseADC::getStopStatus(index_out);
    }
    /** \brief Enables or disables selected channels
     * \param channel_mask 32-bit mask containing if specific channel is enabled.
     * \return Number of active channels
     * E.g. 0x6 value means, that channels 2 and 3 are enabled (starting from channel 1)
     * The return value doesn't have to correspond to active bits in channel_mask,
     * since some channels are not available.
     */
    uint32_t setChannels(uint32_t channel_mask){
        return BaseADC::setChannels(channel_mask);
    }
};

int adc_find_config(gpio_pin_t pin_in, int* adc_id_out, adc_channel_t* adc_channel_out);

#include "adc_target.h"

#endif /* _API_ADC_H_ */
