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
#ifndef _API_TIMER_H_
#define _API_TIMER_H_

#include "core.h"

#ifdef __cplusplus
    extern "C" {
#endif

/** \brief Defines units used for timer initialization */
typedef enum {
    /** \brief Time base is defined in frequency in Hz */
    TIMER_INIT_FREQUENCY,
    /** \brief Time base is defined in micro-seconds */
    TIMER_INIT_PERIOD_US
}timer_time_type_t;

/** \brief Defines timer usage */
typedef enum {
    /** \brief Use for generating periodic interrupt */
    TIMER_USAGE_INTERNAL = 0,
    /** \brief Use for generating PWM output with configurable frequency and duty cycle */
    TIMER_USAGE_PWM = 1,
    /** \brief Use for timing other peripherals such as ADC or DAC */
    TIMER_USAGE_TIMING = 2,
    /** \brief Use for measuring input PWM (low and high pulse width) */
    TIMER_USAGE_PWM_INPUT = 3,
    /** \brief Use for generating PWM with variable duty cycle
     *
     * This is useful for emulating analog output without DAC
     */
    TIMER_USAGE_PWM_GENERATOR = 4,
    /** \brief Use for precise frequency (number of edges) measurement */
    TIMER_USAGE_PULSE_COUNTER = 5,
}timer_usage_t;

/** \brief Flags readable by timer API */
typedef enum {
    /** \brief Timer was updated */
    TIMER_FLAG_UPDATE = 0,
    /** \brief Capture event occured */
    TIMER_FLAG_CHANNEL_CAPTURED = 1,
}timer_flag_t;

/**
 * \struct timer_handle_t
 * \brief This structure is platform dependent
 *
 * Please see corrsponding adc_target.h file for implementation.
 * \sa timer_handle_stm32_t
 */
#include "timer_target.h"

typedef struct {
    timer_usage_t usage;
    timer_time_type_t time_type;
    uint32_t time_value;
    uint16_t duty_cycle;
    gpio_pin_t pin;
    uint32_t buffer_size;
}timer_init_t;

uint16_t prescaler_split(uint32_t in,uint16_t* out2);

int timer_init(timer_handle_t* timer_handle, int timer_id, const timer_init_t* init_data);
int timer_deinit(timer_handle_t* timer_handle);
int timer_start(timer_handle_t* timer_handle);
int timer_stop(timer_handle_t* timer_handle);
int timer_change_time(timer_handle_t* timer_handle, timer_time_type_t type, uint32_t value);
uint32_t timer_change_frequency(timer_handle_t* timer_handle, uint32_t value, uint32_t* prescaler_out);
int timer_change_duty_cycle(timer_handle_t* timer_handle, uint16_t duty_cycle);
int timer_get_pwm_input_value(timer_handle_t* timer_handle, uint32_t* freq_out, uint32_t* duty_cycle_out);
int timer_lock(int timer_id);
int timer_is_free(int timer_id);
uint32_t timer_get_resolution(timer_handle_t* timer_handle);
void timer_unlock(int timer_id);
void timer_unlock_all(void);

int timer_get_counter_value(timer_handle_t* timer_handle, uint32_t *value_out);
uint32_t timer_get_clocks(timer_handle_t* timer_handle);
uint32_t timer_get_frequency_divider(timer_handle_t* timer_handle);

int timer_get_flag(timer_handle_t* timer_handle, timer_flag_t flag);
int timer_clear_flag(timer_handle_t* timer_handle, timer_flag_t flag);

inline int timer_target_init(timer_handle_t* timer_handle, int timer_id, const timer_init_t* init_data);
inline int timer_target_deinit(timer_handle_t* timer_handle);
inline int timer_target_start(timer_handle_t* timer_handle);
inline int timer_target_stop(timer_handle_t* timer_handle);
inline int timer_target_change_time(timer_handle_t* timer_handle, timer_time_type_t type, uint32_t value);
inline int timer_target_change_duty_cycle(timer_handle_t* timer_handle, uint16_t duty_cycle);
inline int timer_target_get_pwm_input_value(timer_handle_t* timer_handle, uint32_t* freq_out, uint32_t* duty_cycle_out);
inline uint32_t timer_target_get_resolution(timer_handle_t* timer_handle);

inline int timer_target_get_counter_value(timer_handle_t* timer_handle, uint32_t *value_out);

inline int timer_target_get_flag(timer_handle_t* timer_handle, timer_flag_t flag);
inline int timer_target_clear_flag(timer_handle_t* timer_handle, timer_flag_t flag);
inline uint32_t timer_target_get_clocks(timer_handle_t* timer_handle);
inline uint32_t timer_target_get_frequency_divider(timer_handle_t* timer_handle);

#ifdef __cplusplus
    }
#endif

#endif /* _API_TIMER_H_ */
