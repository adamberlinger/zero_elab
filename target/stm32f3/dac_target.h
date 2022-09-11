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
#ifndef _DAC_TARGET_H_
#define _DAC_TARGET_H_

#include "stm32_common.h"
#include "stm32_dma.h"
#include "timer.h"
#include "periph.h"

#ifdef __cplusplus
    extern "C" {
#endif

typedef struct {
    DAC_TypeDef* regs;
    int channel;
    dma_handle_t dma;
    timer_handle_t timer;
    /*void* dma_buffer;
    uint32_t dma_buffer_size;

    dac_write_callback_t callback;
    void* user_arg;*/
    circular_buffer_t circular_buffer;
} dac_handle_t;

#ifdef __cplusplus
    }
#endif

gpio_pin_t dac_target_find_pin(int dac_id,int channel);
int dac_target_find_timer(int dac_id,uint8_t* trigger_source);
dma_handle_t dac_target_find_dma(int dac_id,int channel);

class PeriphTargetDAC_STM32F3 : public BasePeriph {
protected:
  DAC_TypeDef* regs;
  int channel;
  dma_handle_t dma;
  timer_handle_t timer;
  circular_buffer_t circular_buffer;
public:
  PeriphTargetDAC_STM32F3(int dac_id, const dac_init_t* init_data){

      gpio_pin_t dac_pin;
      int timer_id = 0;
      uint8_t trigger_source = 0;
      uint32_t crreg = 0;

      dac_pin = dac_target_find_pin(dac_id,init_data->channel);
      timer_id = dac_target_find_timer(dac_id,&trigger_source);
      this->dma = dac_target_find_dma(dac_id,init_data->channel);

      if(this->dma == 0){
          this->setError(ERROR_NO_CONFIGURATION);
          return;
      }

      if(dac_id == 1){
          this->regs = DAC1;
          RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
      }
  #ifdef DAC2
      else if(dac_id == 2){
          this->regs = DAC2;
          RCC->APB1ENR |= RCC_APB1ENR_DAC2EN;
      }
  #endif
      else {
          this->setError(ERROR_INVALID_ID);
          return;
      }

      this->channel = init_data->channel;
      stm32_gpio_init(dac_pin,MODE_AN);

      crreg |= DAC_CR_EN1;
      crreg |= (trigger_source & 0x7) << 3;
  #ifdef STM32F303x8
      if(init_data->channel == 2 && dac_id == 1){
          crreg |= DAC_CR_BOFF1;
      }
      else if(init_data->channel == 1 && dac_id == 2){
          crreg |= DAC_CR_BOFF1;
      }
  #endif
      if(init_data->channel == 1){
          this->regs->CR = (this->regs->CR & 0xFFFF0000) | crreg;
      }
      else {
          this->regs->CR = (this->regs->CR & 0x0000FFFF) | (crreg << 16);
      }

      this->setLevel(0x00);

      buffer_init(&this->circular_buffer);

      this->circular_buffer.samplerate_arg = &this->timer;
      this->circular_buffer.samplerate_callback = (buffer_samplerate_callback_t)timer_change_frequency;

      if(init_data->sample_rate > 0){
          timer_init_t timer_init_data;
          timer_init_data.time_type = TIMER_INIT_FREQUENCY;
          timer_init_data.time_value = init_data->sample_rate;
          timer_init_data.usage = TIMER_USAGE_INTERNAL;
          timer_init(&this->timer,timer_id,&timer_init_data);
      }
      this->circular_buffer.buffer = (uint8_t*)module_malloc(init_data->buffer_size);
      if(this->circular_buffer.buffer){
          this->circular_buffer.buffer_size = init_data->buffer_size;
      }
  }

  void setLevel(uint32_t level){
    if(this->channel == 1) {
        this->regs->DHR12R1 = (level & 0xFFF);
    }
#ifdef DAC_CR_EN2
    else if(this->channel == 2){
        this->regs->DHR12R2 = (level & 0xFFF);
    }
#endif
  }

  void startContinuous(){
    dma_init_t dma_init_structure;

    /* Enable DMA & trigger */
    this->regs->CR |= (DAC_CR_TEN1 | DAC_CR_DMAEN1) << ((this->channel - 1) * 16);

    dma_set_callback(this->dma,(dma_signal_callback_t)dac_target_continuous_callback,this);

    dma_init_structure.memory_address = this->circular_buffer.buffer;
    if(this->channel == 1){
        dma_init_structure.periph_address = (void*)&this->regs->DHR12R1;
    }
#ifdef DAC_CR_EN2
    else {
        dma_init_structure.periph_address = (void*)&this->regs->DHR12R2;
    }
#endif
    dma_init_structure.data_size = this->circular_buffer.buffer_size / 2;
    dma_init_structure.bytes = 2;
    dma_init_structure.direction = DMA_TO_PERIPH;
    dma_init(this->dma,&dma_init_structure);

    dma_start(this->dma,1);
    timer_start(&this->timer);
  }

  void stop(){
    timer_stop(&this->timer);
  }

  static void dac_target_continuous_callback(PeriphTargetDAC_STM32F3* _this,dma_event_t dma_event){
      _this->continuousCallback(dma_event);
  }

  void continuousCallback(dma_event_t dma_event){
    uint8_t *data = this->circular_buffer.buffer;
    uint32_t size = (this->circular_buffer.buffer_size >> 1);
    if(this->circular_buffer.process_callback){
        if(dma_event == DMA_EVENT_FULL_COMPLETE){
            data = data + size;
        }
        this->circular_buffer.process_callback(this->circular_buffer.process_arg,data,size);
    }
  }

  virtual ~PeriphTargetDAC_STM32F3(){
    uint32_t crreg = this->regs->CR;
    if(this->channel == 1){
        crreg &= ~(uint32_t)(0xFFFF);
    }
    if(this->channel == 2){
        crreg &= ~(uint32_t)(0xFFFF0000);
    }
    this->regs->CR = crreg;
  }

  circular_buffer_t* getCircularBuffer(){
      return &this->circular_buffer;
  }
};

typedef PeriphDAC<PeriphTargetDAC_STM32F3> PeriphDefaultDAC;

#endif /* _DAC_TARGET_H_ */
