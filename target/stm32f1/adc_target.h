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

#ifdef __cplusplus
    extern "C" {
#endif

#define ADC_MAX_CHANNELS 16

#define VREFINT_CAL (1489)
#define VREFINT_CAL_MV  (3300)

typedef uint8_t adc_channel_t;

gpio_pin_t adc_target_find_pin(int adc_id,int channel);
int adc_target_find_config(gpio_pin_t pin_in, int* adc_id_out, adc_channel_t* adc_channel_out);
dma_handle_t adc_target_find_dma(int adc_id);
int adc_target_find_timer_counter(int adc_id,int* timer_id_out,
    int* counter_id_out, uint8_t* trigger_source, uint8_t* counter_itr, uint8_t* timer_itr);
void __wait_us(int x);
gpio_pin_t adc_target_find_pin(int adc_id,int channel);
void adc_target_set_pin_analog(gpio_pin_t pin);

#ifdef __cplusplus
    }
#endif

class PeriphTargetADC_STM32F1 : public BasePeriph {
protected:
    /** \brief Pointer to ADC registers */
    ADC_TypeDef* regs;
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
    /** \brief Sample time used for channel configuration */
    uint8_t global_sample_time;
    /** \brief Maximum input impedance for current sampling time */
    uint32_t maximum_input_impedance;
public:
    PeriphTargetADC_STM32F1(int adc_id, const adc_init_t* init_data,
        const adc_channel_t* channel_info){

        int tries = 200000;
        int error = ERROR_NONE;
        int i;
        uint32_t cr2;
        int timer_id = 0;
        int counter_id;
        uint8_t trigger_source = 0;
        uint8_t timer_itr;
        uint8_t counter_itr;
        /* TODO: init get clock */

        /* TODO: load this from DB */
        if(adc_id == 1){
            this->regs = ADC1;
            //this->common_regs = ADC12_COMMON;
            RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
        }
#if 0
        /* ADC2 disabled for now as it doesn't support DMA requests */
        else if(adc_id == 2){
            this->regs = ADC2;
            //this->common_regs = ADC12_COMMON;
            RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
        }
#endif
        /* TODO: add support for ADC3 */
        else {
            this->setError(ERROR_INVALID_ID);
            return;
        }

        this->external_clocks = 0;
        this->dma = adc_target_find_dma(adc_id);

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

        /* Wake-up ADC */
        if((this->regs->CR2 & ADC_CR2_ADON) == 0){
            this->regs->CR2 |= ADC_CR2_ADON;

            __wait_us(10);
        }

        /* Calibration */
        this->regs->CR2 |= ADC_CR2_CAL;
        while(tries-- || (this->regs->CR2 & ADC_CR2_CAL) != 0);
        if(this->regs->CR2 & ADC_CR2_CAL){
            /* ADC calibration timeout */
            this->setError(ERROR_TIMEOUT);
            return;
        }

        cr2 = ADC_CR2_DMA | ADC_CR2_ADON;
        /* setup size */
        if(init_data->data_size != 12) {
            /* Only 12-bit size is allowed */
            this->setError(ERROR_BAD_CONFIGURATION);
            return;
        }
        /* Enable VREFINT sensing */
        if(adc_id == 1){
            cr2 |= ADC_CR2_TSVREFE; /* same for ADC34 */
        }
        this->regs->CR1 = ADC_CR1_SCAN;
        this->regs->CR2 = cr2;

        this->num_channels = init_data->num_channels;
        this->active_channels = init_data->num_channels;
        this->configured_channels = (uint8_t*)module_malloc(init_data->num_channels);

        /* Configure channels */
        this->regs->SQR1 = ((init_data->num_channels - 1) & 0xF) << 20;
        this->regs->SQR2 = 0;
        this->regs->SQR3 = 0;
        this->regs->SMPR1 = 0;
        this->regs->SMPR2 = 0;

        /* Select sample rate */
        this->global_sample_time = (init_data->hints & ADC_HINT_FAST)?0x2:0x5;
        for(i = 0;i < init_data->num_channels;++i){
            /* TODO: check channel number */

            gpio_pin_t pin = adc_target_find_pin(adc_id,channel_info[i]);
            if(pin){
                adc_target_set_pin_analog(pin);
            }
            this->configureRegularSequence(i,channel_info[i],this->global_sample_time);

            this->configured_channels[i] = channel_info[i];
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
        else {
            /* SWSTART bit as trigger */
            this->initTrigger(0x7);
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
            this->circular_buffer.item_size = (init_data->num_channels * this->getDataBytes());
        }
        else {
            this->circular_buffer.buffer = 0;
        }
    }

    virtual ~PeriphTargetADC_STM32F1(){
        if(this->sample_rate > 0){
            timer_deinit(&this->timer);
            timer_deinit(&this->timer_counter);
        }

        if(this->dma){
            dma_deinit(this->dma);
        }

        if(this->regs->CR2 & ADC_CR2_ADON){
            this->regs->CR2 &= ~ADC_CR2_ADON;
            while(this->regs->CR2 & ADC_CR2_ADON) continue;
        }

        /* TODO: handle shared clocks and peripheral reset ? */
    }

    static uint32_t adc_target_set_sample_rate(PeriphTargetADC_STM32F1* _this, uint32_t value, uint32_t* prescaler_out){
        return _this->setSampleRate(value,prescaler_out);
    }

    void updateSampleTime(){
        uint32_t smpr;
        /* TODO: dynamic sample time */
        maximum_input_impedance = 0;
        /*if(this->sample_rate > 0){
            uint32_t total_frequency = this->active_channels * this->sample_rate;
            uint32_t config = adc_target_find_sampletime(total_frequency, &maximum_input_impedance);
            smpr = config;
        }
        else {
          uint32_t config = adc_target_find_sampletime(0, &maximum_input_impedance);
          smpr = config;
        }*/
        this->global_sample_time = smpr = (this->sample_rate > 0)?0x2:0x5;

        smpr |= (smpr << 3);
        smpr |= (smpr << 6);
        smpr |= (smpr << 12);
        smpr |= (smpr << 24);

        this->regs->SMPR1 = (smpr & 0x3FFFFFF8);
        this->regs->SMPR2 = (smpr & 0x07FFFFFF);
    }

    int getAttribute(buffer_attribute_t name, void* value_out){
        if(name == BUFFER_ATTRIBUTE_MAX_IMPEDANCE){
            *((uint32_t*)value_out) = maximum_input_impedance;
            return 0;
        }
        return -1;
    }

    uint32_t setChannels(uint32_t channel_mask){
        int i;
        if(this->regs->CR2 & ADC_CR2_ADON){
            this->regs->CR2 &= ~ADC_CR2_ADON;
            while(this->regs->CR2 & ADC_CR2_ADON) continue;
        }

        /* Configure channels */
        this->regs->SQR1 = 0;
        this->regs->SQR2 = 0;
        this->regs->SQR3 = 0;
        this->regs->SMPR1 = 0;
        this->regs->SMPR2 = 0;

        this->active_channels = 0;
        for(i = 0;i < this->num_channels;++i){
            if(channel_mask & (1 << i)){
                /* Single ended channels */
                if(this->configured_channels[i] < 20){
                    this->configureRegularSequence(i,this->configured_channels[i],this->global_sample_time);
                }
                /* Differential channels */
                else {
                    this->configureRegularSequence(i,this->configured_channels[i]-20,this->global_sample_time);
                }
                this->active_channels++;
            }
        }
        this->regs->SQR1 |= ((this->active_channels - 1) & 0xF) << 20;

        this->circular_buffer.item_size = (this->active_channels *  this->getDataBytes());
        updateSampleTime();
        return this->active_channels;
    }

    uint32_t setSampleRate(uint32_t value, uint32_t* prescaler_out){
        if(this->external_clocks == 0){
            this->sample_rate = value;
            *prescaler_out = 1;
            updateSampleTime();
            return timer_change_frequency(&this->timer,value,prescaler_out);
        }
        return value;
    }

    void startContinuous(){
      dma_init_t dma_init_structure;
      int num_bytes = this->getDataBytes();

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

      if((this->regs->CR2 & ADC_CR2_ADON) == 0){
          this->regs->CR2 |= ADC_CR2_ADON;
          while((this->regs->CR2 & ADC_CR2_ADON) == 0) continue;
      }

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
      int16_t* adc_value = (int16_t*)values;
      uint32_t i;
      dma_init_t dma_init_structure;
      dma_init_structure.memory_address = values;
      dma_init_structure.periph_address = (void *)&this->regs->DR;
      dma_init_structure.data_size = this->getChannels();
      dma_init_structure.bytes = this->getDataBytes();
      dma_init_structure.direction = DMA_TO_MEMORY;
      dma_init(this->dma,&dma_init_structure);

      dma_start(this->dma,0);
      if((this->regs->CR2 & ADC_CR2_ADON) == 0){
          this->regs->CR2 |= ADC_CR2_ADON;
          while((this->regs->CR2 & ADC_CR2_ADON) == 0) continue;
      }

      this->regs->CR2 |= ADC_CR2_SWSTART;

      dma_wait_for_complete(this->dma);

      for(i = 0;i < this->num_channels;++i){
          if(this->configured_channels[i] > 20){
              adc_value[i] <<= 1;
          }
      }
    }

    circular_buffer_t* getCircularBuffer(){
        return &this->circular_buffer;
    }

    void configureRegularSequence(int i,
        adc_channel_t channel,uint8_t sample_time){

        /* Force long sample time for VREFINT */
        if(channel == 17 && this->sample_rate == 0){
          sample_time = 0x7;
        }

        if(i <= 5){
            this->regs->SQR3 |= (channel << (i*5));
        }
        else if (i <= 11){
            this->regs->SQR2 |= (channel << ((i-6)*5));
        }
        else if (i <= 15) {
            this->regs->SQR1 |= (channel << ((i - 12)*5));
        }

        if(channel <= 9){
            this->regs->SMPR2 |= ((sample_time & 0x7) << (channel * 3));
        }
        else {
            this->regs->SMPR1 |= ((sample_time & 0x7) << ((channel-10) * 3));
        }
    }

    int getDataBytes(){
        return 2;
    }

    void initTrigger(int trigger_source){
      uint32_t cr2 = this->regs->CR2;
      /* Enable external trigger (rising edge) */
      cr2 |= (0x1) << 20;
      cr2 |= (trigger_source) << 17;
      this->regs->CR2 = cr2;
    }
};

typedef PeriphADC<PeriphTargetADC_STM32F1> PeriphDefaultADC;

#endif /* _ADC_TARGET_H_ */
