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
#ifndef _TIMER_TARGET_H_
#define _TIMER_TARGET_H_

#include "stm32_common.h"
#include "stm32_dma.h"
#include "circular_buffer.h"

#ifdef __cplusplus
    extern "C" {
#endif

typedef enum {
    TIMER_TYPE_ADVANCED = 0x01,
    TIMER_TYPE_GENERAL = 0x02,
    TIMER_TYPE_BASIC = 0x03,
    TIMER_TYPE_HAS_COMPLEMENTARY = 0x80,
}timer_type_t;

typedef struct _timer_handle {
    TIM_TypeDef* regs;
    timer_type_t type;
    timer_usage_t usage;
    int channels;
    int main_channel;
    int timer_id;
    circular_buffer_t *circular_buffer;
    additional_buffer_t *additional_buffer;
    void *custom_dma_src;
    dma_handle_t dma;
    int8_t periph_prescaler;
    uint8_t config_index;
    struct _timer_handle *help_timer;
} timer_handle_stm32_t;

typedef timer_handle_stm32_t timer_handle_t;

int timer_target_update_prescaler(timer_handle_t* timer_handle,int value);
void timer_target_set_external_clocks(timer_handle_t* timer_handle,gpio_pin_t pin, uint8_t af);

int timer_target_configure_counter(timer_handle_t* timer_handle, int timer_id, uint32_t reset_value,uint8_t itr);
void timer_target_reconfigure_counter(timer_handle_t* timer_handle,uint32_t reset_value);
int timer_target_find_master_timer(int timer_id,int* timer_id_out,uint8_t* itr_out);
int timer_target_find_looped_timer(int timer_id,
        int* timer_id_out, uint8_t* itr_master, uint8_t* itr_slave);

uint8_t timer_target_find_etr_af(int timer_id,gpio_pin_t pin);
void timer_target_configure_slave_gated_mode(timer_handle_t* timer_handle,
    uint8_t itr);
void timer_target_configure_custom_dma(timer_handle_t* timer_handle, void* src_address, additional_buffer_t* buffer);

void timer_target_set_repetitioncounter(timer_handle_t* timer_handle, uint8_t value);
uint8_t timer_target_get_repetitioncounter(timer_handle_t* timer_handle);

typedef struct {
    uint8_t tim_channel;
    gpio_pin_t pin;
    uint8_t af;
}timer_pin_db_t;

typedef struct {
    int timer_id;
    TIM_TypeDef* regs;
    timer_type_t type;
    int channels;
    volatile uint32_t* port_enr;
    volatile uint32_t* port_rst;
    uint32_t port_enr_bit;
    int8_t prescaler;
#ifdef TIM_TARGET_APB_PRESCALERS
    int8_t prescaler;
#endif
}timer_config_db_t;

typedef struct {
    uint8_t master_timer;
    uint8_t slave_timer;
    uint8_t itr_value;
    uint8_t reverse_index;
}timer_slave_db_t;

typedef struct {
    int timer_id;
#ifdef STM32_HASDMAMUX
    uint32_t dmamux_select;
#else
    dma_handle_t dma;
#ifdef DMA_TARGET_SOURCE_SELECT
    uint8_t dma_select;
#endif
#endif
}timer_dma_db_t;

#define DEFINE_TIM_CHANNEL(tim,channel) (uint8_t)(((tim & 0x1F) << 3) | (channel & 0x7))
#define TIMER_PIN_DB_TIMER_ID(tim_channel) (int)((tim_channel >> 3) & 0x1F)
#define TIMER_PIN_DB_CHANNEL(tim_channel) (int)(tim_channel & 0x7)

#ifdef __cplusplus
    }
#endif

#endif /* _TIMER_TARGET_H_ */
