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
#ifndef _ADC_TARGET_H_
#define _ADC_TARGET_H_

#include "stm32_common.h"
#include "stm32_dma.h"
#include "timer.h"

#include "periph.h"
#include "adc.h"

#define ADC_MAX_CHANNELS 16

#ifdef STM32F0XX
#define VREFINT_CAL (*((uint16_t*)0x1FFFF7BA))
#define VREFINT_CAL_MV  (3300)
#else
#define VREFINT_CAL (*((uint16_t*)0x1FF80078))
#define VREFINT_CAL_MV  (3000)
#endif

gpio_pin_t adc_target_find_pin(int adc_id,int channel);
int adc_target_find_config(gpio_pin_t pin_in, int* adc_id_out, adc_channel_t* adc_channel_out);
int adc_target_find_timer_counter(int adc_id,int* timer_id_out,
    int* counter_id_out, uint8_t* trigger_source, uint8_t* counter_itr, uint8_t* timer_itr);
#ifdef STM32L0XX
dma_handle_t adc_target_find_dma(int adc_id, uint8_t* dma_src);
#else
dma_handle_t adc_target_find_dma(int adc_id);
#endif

uint32_t adc_target_find_sampletime(uint32_t total_frequency, uint32_t *maximum_input_impedance);

void adc_target_set_pin_analog(gpio_pin_t pin);

class PeriphTargetADC_STM32x0 : public BasePeriph {
protected:
    /** \brief Pointer to ADC registers */
    ADC_TypeDef* regs;
    /** \brief Pointer to ADC common registers */
    ADC_Common_TypeDef* common_regs;
    /** \brief DMA handle */
    dma_handle_t dma;
    /** \brief Handle of trigger timer specifying sampling frequency */
    timer_handle_t timer;
    /** \brief Handle of counter timer
     *
     * Counter timer is used for sample precise stoping of ADC
     */
    timer_handle_t timer_counter;
    /** \brief Circular buffer represented by this ADC */
    circular_buffer_t circular_buffer;
    /** \brief Sample rate in Hz */
    uint32_t sample_rate;
    /** \brief Non-zero if external trigger of ADC is used */
    uint8_t external_clocks;
    /** \brief Number of available channels */
    uint8_t num_channels;
    /** \brief Array containing available channel numbers */
    uint8_t *configured_channels;
    /** \brief Number of currently active channels */
    uint8_t active_channels;
    /** \brief Maximum input impedance for current sampling time */
    uint32_t maximum_input_impedance;
public:
    PeriphTargetADC_STM32x0(int adc_id,const adc_init_t* init_data,const adc_channel_t* channel_info){
        int tries = 2000;
        int error = ERROR_NONE;
        int i;
        uint32_t cfgr;
        int timer_id = 0;
        int counter_id;
        uint8_t trigger_source = 0;
        uint8_t timer_itr;
        uint8_t counter_itr;
    #ifdef STM32L0XX
        uint8_t dma_source = 0;
    #endif

        maximum_input_impedance = 0;

        if(adc_id == 1){
            this->regs = ADC1;
            this->common_regs = ADC1_COMMON;
            RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
        }
        else {
            this->setError(ERROR_INVALID_ID);
            return;
        }
    #ifdef STM32L0XX
        this->regs->CR |= ADC_CR_ADVREGEN;
    #endif

        if(init_data->hints & ADC_HINT_FAST){
            /* ADCCLK = 16 MHz */
            this->regs->CFGR2 = 0;
        }
        else {
            /* ADCCLK = 48MHz / 4 = 12 MHz */
            this->regs->CFGR2 = 0x80000000;
        }

        /* Disable ADC if enabled from previous configuration */
        /* TODO: maybe move it to desctructor/deinit ??? */
        if(this->regs->CR & ADC_CR_ADEN){
            this->regs->CR |= ADC_CR_ADDIS;
            while(tries-- || (this->regs->CR & ADC_CR_ADDIS));
        }

        this->external_clocks = 0;
    #ifdef STM32L0XX
        this->dma = adc_target_find_dma(adc_id, &dma_source);
    #else
        this->dma = adc_target_find_dma(adc_id);
    #endif

        if(init_data->sample_rate > 0){
            if(!adc_target_find_timer_counter(adc_id,&timer_id,&counter_id,&trigger_source,&counter_itr,&timer_itr)){
                this->setError(ERROR_NO_CONFIGURATION);
                return;
            }
        }

        if(this->dma == 0){
            this->setError(ERROR_NO_CONFIGURATION);
            return;
        }
    #ifdef STM32L0XX
        else {
            dma_set_source(this->dma, dma_source);
        }
    #endif

        /* Calibration */
        this->regs->CR |= ADC_CR_ADCAL;
        while(tries-- || (this->regs->CR & ADC_CR_ADCAL) != 0);
        if(this->regs->CR & ADC_CR_ADCAL){
            /* ADC calibration timeout */
            this->setError(ERROR_TIMEOUT);
            return;
        }

        cfgr = ADC_CFGR1_DMAEN;
        /* setup size */
        if(init_data->data_size == 8){
            cfgr |= (0x2 << 3);
        }
        else if(init_data->data_size == 12){
            /* nothing to do */
        }
        else {
            /* unallowed data size */
            this->setError(ERROR_BAD_CONFIGURATION);
            return;
        }
        this->regs->CFGR1 = cfgr;

        this->num_channels = init_data->num_channels;
        this->active_channels = init_data->num_channels;
        this->configured_channels = (uint8_t*)module_malloc(init_data->num_channels);

        /* Configure channels */
        this->regs->SMPR = (init_data->sample_rate>0)?0x0:0x7;
        this->regs->CHSELR = 0;
        for(i = 0;i < init_data->num_channels;++i){
            gpio_pin_t pin = adc_target_find_pin(adc_id,channel_info[i]);
            if(pin){
                adc_target_set_pin_analog(pin);
            }

            this->configured_channels[i] = channel_info[i];
            this->regs->CHSELR |= (1 << channel_info[i]);
        }

        /* Enable VREFINT sensing */
        this->common_regs->CCR |= ADC_CCR_VREFEN;

        /* Enable ADC */
        this->regs->CR |= ADC_CR_ADEN;
        tries = 200000;
        while(tries-- && (this->regs->ISR & ADC_ISR_ADRDY) == 0);
        if((this->regs->ISR & ADC_ISR_ADRDY) == 0){
            /* ADC ready flag timeout */
            this->setError(ERROR_TIMEOUT);
            return;
        }

        this->sample_rate = init_data->sample_rate;
        if(init_data->sample_rate > 0){
            int num_bytes = this->getDataBytes();

            timer_init_t timer_init_data;
            timer_init_data.time_type = TIMER_INIT_FREQUENCY;
            timer_init_data.time_value = init_data->sample_rate;
            timer_init_data.usage = TIMER_USAGE_INTERNAL;
            timer_init(&this->timer,timer_id,&timer_init_data);
            this->initTrigger(trigger_source);

            timer_target_configure_counter(&this->timer_counter,counter_id,
                (init_data->buffer_size / num_bytes) / init_data->num_channels,counter_itr);

            timer_target_configure_slave_gated_mode(&this->timer,timer_itr);
        }

        buffer_init(&this->circular_buffer);

        this->circular_buffer.samplerate_arg = this;
        this->circular_buffer.samplerate_callback = (buffer_samplerate_callback_t)adc_target_set_sample_rate;

        /* This should be last, since the memory cannot be freed */
        if(init_data->buffer_size > 0){
            error = buffer_alloc(&this->circular_buffer, init_data->buffer_size);
            if(error != ERROR_NONE){
                this->setError(ERROR_OUT_OF_MEMORY);
                return;
            }
            this->circular_buffer.item_size = (init_data->num_channels *  this->getDataBytes());
        }
        else {
            this->circular_buffer.buffer = 0;
        }
    }

    virtual ~PeriphTargetADC_STM32x0(){
        if(this->sample_rate > 0){
            timer_deinit(&this->timer);
            timer_deinit(&this->timer_counter);
        }

        if(this->dma){
            dma_deinit(this->dma);
        }

        if(this->regs->CR & ADC_CR_ADEN){
            this->regs->CR |= ADC_CR_ADSTP;
            while(this->regs->CR & ADC_CR_ADSTP) continue;

            this->regs->CR |= ADC_CR_ADDIS;
            while(this->regs->CR & ADC_CR_ADDIS) continue;
        }


        RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
    }

    static uint32_t adc_target_set_sample_rate(PeriphTargetADC_STM32x0* _this, uint32_t value, uint32_t* prescaler_out){
        return _this->setSampleRate(value,prescaler_out);
    }

    void updateSampleTime(){
        if(this->sample_rate > 0){
            uint32_t total_frequency = this->active_channels * this->sample_rate;
            uint32_t config = adc_target_find_sampletime(total_frequency, &maximum_input_impedance);
            this->regs->SMPR = config;
        }
        else {
          uint32_t config = adc_target_find_sampletime(0, &maximum_input_impedance);
          this->regs->SMPR = config;
        }
    }

    int getAttribute(buffer_attribute_t name, void* value_out){
        if(name == BUFFER_ATTRIBUTE_MAX_IMPEDANCE){
            *((uint32_t*)value_out) = maximum_input_impedance;
            return 0;
        }
        return -1;
    }

    uint32_t setChannels(uint32_t channel_mask){
        uint32_t chselr = 0;
        int i;
        this->regs->CR |= ADC_CR_ADSTP;
        while(this->regs->CR & ADC_CR_ADSTP) continue;
        /* TODO: implement */

        this->active_channels = 0;
        for(i = 0;i < this->num_channels;++i){
            if(channel_mask & (1 << i)){
                chselr |= (1 << this->configured_channels[i]);
                this->active_channels++;
            }
        }

        this->regs->CHSELR = chselr;

        this->circular_buffer.item_size = (this->active_channels *  this->getDataBytes());
        updateSampleTime();
        return this->active_channels;
    }

    uint32_t setSampleRate(uint32_t value, uint32_t* prescaler_out){
        if(this->external_clocks == 0){
            this->sample_rate = value;
            *prescaler_out = 1;
            /* TODO: stop & resume adc */
            updateSampleTime();
            return timer_change_frequency(&this->timer,value,prescaler_out);
        }
        return value;
    }

    void startContinuous(){
        dma_init_t dma_init_structure;
        int num_bytes = this->getDataBytes();

        this->regs->CFGR1 |= ADC_CFGR1_DMACFG;

        timer_stop(&this->timer);
        timer_stop(&this->timer_counter);
        this->timer_counter.regs->CCR1 = 0xFFFF;
        this->timer_counter.regs->CNT = 0x0;
        timer_target_reconfigure_counter(&this->timer_counter,
            (this->circular_buffer.buffer_size / num_bytes) / this->active_channels);
        this->timer.regs->CNT = 0x0;
        dma_stop(this->dma);

        dma_set_callback(this->dma,dma_generic_callback,&this->circular_buffer);
        dma_init_structure.memory_address = this->circular_buffer.buffer;
        dma_init_structure.periph_address = (void *)&this->regs->DR;
        dma_init_structure.data_size = this->circular_buffer.buffer_size / num_bytes;
        dma_init_structure.bytes = num_bytes;
        dma_init_structure.direction = DMA_TO_MEMORY;
        dma_init(this->dma,&dma_init_structure);

        dma_start(this->dma,1);
        this->regs->CR |= ADC_CR_ADSTART;

        timer_start(&this->timer_counter);
        timer_start(&this->timer);
    }

    void stop(){
        if(this->sample_rate > 0){
            timer_stop(&this->timer);
        }
        dma_stop(this->dma);
    }

    void stopPrecise(uint32_t sample_number){
        /* Set inactive on match */
        this->timer_counter.regs->SR &= ~TIM_SR_CC1IF;
        this->timer_counter.regs->CCR1 = sample_number;
        this->timer_counter.regs->CCMR1 = (0x2) << 4;
    }

    int getStopStatus(uint32_t *index_out){
        if(this->timer_counter.regs->SR & TIM_SR_CC1IF){
            int num_bytes = this->getDataBytes();
            /* Delay for ADC and DMA to finish conversion & data transfers */
            uint32_t wait_cycles = 0x100;
            uint32_t transfered_samples;
            while(wait_cycles--){
                __DSB();
            }
            /* Wait untill DMA number of transfers corresponds to stopped timer */
            wait_cycles = 0x200; /* timeout value */
            do{
                transfered_samples = (this->circular_buffer.buffer_size / num_bytes) - this->dma->CNDTR;
                transfered_samples /= this->active_channels;
                wait_cycles--;
            }while(this->timer_counter.regs->CNT != transfered_samples && wait_cycles);

            timer_stop(&this->timer);
            (*index_out) = (this->circular_buffer.buffer_size / num_bytes) - this->dma->CNDTR;
            (*index_out) -= (*index_out) % this->active_channels;
            return 1;
        }
        return 0;
    }

    int getChannels(){
        return this->active_channels;
    }

    void readData(void* values){
        dma_init_t dma_init_structure;
        dma_init_structure.memory_address = values;
        dma_init_structure.periph_address = (void *)&this->regs->DR;
        dma_init_structure.data_size = this->getChannels();
        dma_init_structure.bytes = this->getDataBytes();
        dma_init_structure.direction = DMA_TO_MEMORY;
        dma_init(this->dma,&dma_init_structure);

        dma_start(this->dma,0);
        this->regs->CR |= ADC_CR_ADSTART;
        dma_wait_for_complete(this->dma);
    }

    circular_buffer_t* getCircularBuffer(){
        return &this->circular_buffer;
    }

    int getDataBytes(){
        return (this->regs->CFGR1 & ADC_CFGR1_RES_1)?1:2;
    }

    void initTrigger(int trigger_source){
        uint32_t cfgr = this->regs->CFGR1;
        /* Setup trigger on rising edge */
        cfgr |= (0x1) << 10;
        cfgr |= (trigger_source) << 6;
        this->regs->CFGR1 = cfgr;
    }

    timer_handle_t* getTimer(){return &timer;}
};

typedef PeriphADC<PeriphTargetADC_STM32x0> PeriphDefaultADC;

#endif /* _ADC_TARGET_H_ */
