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
#include "timer.h"

#include "timer_target_db.h"
#include "error_codes.h"

static int timer_target_find_channel(int timer_id, gpio_pin_t pin,uint8_t *af_out);
static int timer_target_find_config(timer_handle_t* timer_handle, int timer_id);
static dma_handle_t timer_target_find_dma(int timer_id);
static void timer_target_configure_channel(timer_handle_t* timer_handle, int channel, uint32_t bits);
static void timer_target_start_continuous(timer_handle_t* timer_handle);

/* TODO: support advanced timers */

uint32_t timer_target_get_clocks(timer_handle_t* timer_handle){
    uint32_t clocks = get_core_clock();
    /* Negative value = multiplier */
    if(timer_handle->periph_prescaler < 0) clocks *= -timer_handle->periph_prescaler;
    /* Positive value = prescaler */
    else clocks /= timer_handle->periph_prescaler;
    return clocks;
}

#define PWM_INPUT_RESET_DELAY      (2)

/* Init configuration for gated mode */
const static timer_init_t gate_timer_init = {
    .usage = TIMER_USAGE_INTERNAL,
    .time_type = TIMER_INIT_FREQUENCY,
    .time_value = 1,
    .duty_cycle = 1,
    .pin = 0,
    .buffer_size = 0
};

int timer_target_init(timer_handle_t* timer_handle, int timer_id, const timer_init_t* init_data){
    uint32_t base_frequency;
    uint32_t cr1 = 0x0,cr2 = 0x0;
    uint32_t divider;
    uint16_t prescaler,arr;
    timer_handle->main_channel = 0;
    timer_handle->dma = 0;
    timer_handle->timer_id = timer_id;
    timer_handle->help_timer = 0;
    timer_handle->usage = init_data->usage;
    if(!timer_target_find_config(timer_handle, timer_id)){
        return ERROR_NO_CONFIGURATION;
    }

    base_frequency = timer_target_get_clocks(timer_handle);

    /* Compute divider */
    if(init_data->time_type == TIMER_INIT_FREQUENCY){
        divider = base_frequency / init_data->time_value;
    }
    else {
        /* Assuming base_frequency is rounded to MHz */
        divider = (base_frequency / 1000000) * init_data->time_value;
    }

    if(init_data->usage == TIMER_USAGE_PULSE_COUNTER){
        arr = 1;
        prescaler = 1;
    }
    else if(init_data->usage == TIMER_USAGE_PWM_GENERATOR){
        /* Approximating DAC by PWM */
        /* ARR should be fixed value to have fixed precision */
        arr = 256;
        prescaler = (divider / arr);
        if(prescaler == 0) {
            prescaler = 1;
        }
    }
    else {
        /* split divider to prescaler & autoreload */
        prescaler = prescaler_split(divider,&arr);

        /* Prevent underflow */
        if(arr == 0) {
            arr = 1;
        }
    }



    timer_handle->regs->PSC = prescaler - 1;
    timer_handle->regs->ARR = arr - 1;

    if(init_data->usage == TIMER_USAGE_INTERNAL){
        /* TRGO on update event */
        cr2 |= (2 << 4);
#if defined(STM32G0XX) || defined(STM32C0XX)
        /* TRGO 2 */
        if(timer_id == 1){
            cr2 |= (2 << 20);
        }
#endif
    }
    else if(init_data->usage == TIMER_USAGE_PULSE_COUNTER){
        /* Configure ETR as external clocks */
        int gate_timer_id;
        uint8_t itr_slave;
        uint8_t pin_af;
        if(timer_target_find_master_timer(timer_id,&gate_timer_id,&itr_slave)){
            timer_handle->help_timer = (timer_handle_t*)module_malloc(sizeof(timer_handle_t));
            timer_init(timer_handle->help_timer,gate_timer_id,&gate_timer_init);
            timer_handle->help_timer->regs->CR2 &= ~(uint32_t)(0x7 << 4);
            timer_handle->help_timer->regs->CR2 |= (0x4 << 4);
            timer_handle->help_timer->regs->CR1 |= TIM_CR1_OPM;
            /* TODO: check overflow */
            timer_handle->help_timer->regs->ARR += 1;

            timer_handle->help_timer->regs->CCMR1 = (0x7 << 4); /* PWM mode 2 */
            timer_handle->help_timer->regs->CCR1 = 0x1;
            timer_handle->help_timer->regs->CCER = 0x1; /* Enable CH1 */
        }

        if(init_data->pin){
            pin_af = timer_target_find_etr_af(timer_id,init_data->pin);
            stm32_gpio_af(init_data->pin,MODE_IN,pin_af);
        }

        timer_handle->regs->SMCR = TIM_SMCR_ECE | (0x5) | (itr_slave << 4);
        //timer_handle->regs->SMCR = TIM_SMCR_ECE;
        timer_handle->regs->ARR = 0xFFFF;
    }
    else if(init_data->usage == TIMER_USAGE_PWM ||
        init_data->usage == TIMER_USAGE_PWM_GENERATOR){
        /* TODO: missing support for channels  5 & 6 */
        uint8_t af;
        uint32_t ccmr_set;
        uint32_t ccr;

        timer_handle->main_channel = timer_target_find_channel(timer_id, init_data->pin,&af);

        if(timer_handle->main_channel <= 0 || timer_handle->main_channel > timer_handle->channels){
            return ERROR_NO_CONFIGURATION;
        }

        if(init_data->pin){
            stm32_gpio_af(init_data->pin,MODE_OUT_PP,af);
        }

        /* Configure channel */
        ccmr_set = (0x6 << 4); /* PWM mode 1 */
        timer_target_configure_channel(timer_handle, timer_handle->main_channel, ccmr_set);

        ccr = (((uint32_t)arr * init_data->duty_cycle) / 0x10000);
        if(timer_handle->main_channel < 5){
            *(((uint32_t*)&timer_handle->regs->CCR1) + (timer_handle->main_channel-1)) = ccr;

        }

        /* Enable channel */
        timer_handle->regs->CCER |= (1 << ((timer_handle->main_channel - 1) * 4));
        /* Enable complementary channel with same polarity */
        if(timer_handle->type & TIMER_TYPE_HAS_COMPLEMENTARY){
            timer_handle->regs->CCER |= (0x3 << (((timer_handle->main_channel - 1) * 4)+2));
        }

        if(init_data->usage == TIMER_USAGE_PWM_GENERATOR){
            timer_handle->circular_buffer = (circular_buffer_t*)module_malloc(sizeof(circular_buffer_t));
            buffer_init(timer_handle->circular_buffer);
            timer_handle->circular_buffer->control_arg = timer_handle;
            timer_handle->circular_buffer->stop_callback = (buffer_stop_callback_t)timer_stop;
            timer_handle->circular_buffer->start_callback = (buffer_start_callback_t)timer_target_start_continuous;
            timer_handle->circular_buffer->deinit_callback = (buffer_deinit_callback_t)timer_deinit;

            timer_handle->regs->DIER = TIM_DIER_UDE;
            timer_handle->dma = timer_target_find_dma(timer_id);

            if(timer_handle->dma){
                buffer_alloc(timer_handle->circular_buffer, init_data->buffer_size);
            }
        }
    }
    else if(init_data->usage == TIMER_USAGE_PWM_INPUT){
        int secondary_channel;
        uint8_t af = 0;
        uint32_t ccmr_set;
        uint32_t ccer = 0;

        timer_handle->main_channel = timer_target_find_channel(timer_id, init_data->pin,&af);

        /* Only on channel 1 or 2 */
        if(timer_handle->main_channel <= 0 || timer_handle->main_channel > 2){
            return ERROR_NO_CONFIGURATION;
        }

        if(init_data->pin){
            stm32_gpio_af(init_data->pin,MODE_IN,af);
        }

        secondary_channel = (timer_handle->main_channel == 1)?2:1;

        /* Configure main channel (rising edge) */
        ccmr_set = 0x1; /* Input capture */
        timer_target_configure_channel(timer_handle, timer_handle->main_channel, ccmr_set);
        ccer |= ((0x1) << ((timer_handle->main_channel - 1) * 4));

        /* Configure secondary channel (falling edge) */
        ccmr_set = 0x2;
        timer_target_configure_channel(timer_handle, secondary_channel, ccmr_set);
        ccer |= ((0x3) << ((secondary_channel - 1) * 4));

        timer_handle->regs->CCER |= ccer;

        /* Count to max. value */
        timer_handle->regs->ARR = 0xFFFF;

        /* Setup reset slave mode */
        timer_handle->regs->SMCR = (timer_handle->main_channel == 1)?(0x50):(0x60);
        timer_handle->regs->SMCR |= 0x4;

        cr1 |= TIM_CR1_URS;
    }

    timer_handle->regs->CR1 = cr1;
    timer_handle->regs->CR2 = cr2;

    return 0;
}

int timer_target_deinit(timer_handle_t* timer_handle){
    if(timer_handle->help_timer){
        timer_target_deinit(timer_handle->help_timer);
    }

    if(timer_handle->dma){
        dma_deinit(timer_handle->dma);
    }

    (*timer_config_db[timer_handle->config_index].port_rst) |= timer_config_db[timer_handle->config_index].port_enr_bit;
    (*timer_config_db[timer_handle->config_index].port_rst) &= ~timer_config_db[timer_handle->config_index].port_enr_bit;
    (*timer_config_db[timer_handle->config_index].port_enr) &= ~timer_config_db[timer_handle->config_index].port_enr_bit;

    return 0;
}

void timer_target_start_continuous(timer_handle_t* timer_handle){
    dma_init_t dma_init_structure;
    int num_bytes = 2;

    timer_stop(timer_handle);
    dma_stop(timer_handle->dma);

    dma_set_callback(timer_handle->dma,dma_generic_callback,timer_handle->circular_buffer);
    dma_init_structure.memory_address = timer_handle->circular_buffer->buffer;

    dma_init_structure.periph_address = (void *)(((uint32_t*)&timer_handle->regs->CCR1) + (timer_handle->main_channel-1));
    dma_init_structure.data_size = timer_handle->circular_buffer->buffer_size / num_bytes;
    dma_init_structure.bytes = num_bytes;
    dma_init_structure.direction = DMA_TO_PERIPH;
    dma_init(timer_handle->dma,&dma_init_structure);

    dma_start(timer_handle->dma,1);

    timer_start(timer_handle);
}

void timer_target_set_external_clocks(timer_handle_t* timer_handle,gpio_pin_t pin, uint8_t af){
    timer_handle->regs->SMCR |= TIM_SMCR_ECE;
    timer_handle->regs->ARR = 1;
    timer_handle->regs->PSC = 0;

    stm32_gpio_af(pin,MODE_OUT_PP,af);
}

int timer_target_configure_counter(timer_handle_t* timer_handle, int timer_id,
    uint32_t reset_value,uint8_t itr){

    if(!timer_target_find_config(timer_handle, timer_id)){
        return ERROR_NO_CONFIGURATION;
    }

    timer_handle->regs->CR1 = 0;
    /* OC1_REF = TRGO */
    timer_handle->regs->CR2 = (4) << 4;
    timer_handle->regs->PSC = 0;
    timer_handle->regs->SMCR = ((itr & 0x7) << 4) | (0x7);
    timer_handle->regs->ARR = reset_value-1;

    /* Force active on OC1 */
    timer_handle->regs->CCR1 = reset_value;
    timer_handle->regs->CCMR1 = (0x5) << 4;
    timer_handle->regs->CCER = 0x1;

    timer_handle->help_timer = 0;
    timer_handle->dma = 0;

    return ERROR_NONE;
}

void timer_target_reconfigure_counter(timer_handle_t* timer_handle,uint32_t reset_value){
    /* Force active on OC1 */
    timer_handle->regs->CCMR1 = (0x5) << 4;
    timer_handle->regs->ARR = reset_value-1;
}

void timer_target_configure_slave_gated_mode(timer_handle_t* timer_handle, uint8_t itr){
    timer_handle->regs->SMCR = (0 << 15) | ((itr & 0x7) << 4) | (0x5);
}

void timer_target_custom_dma_start(timer_handle_t* timer_handle){
    if(timer_handle->dma){
        dma_init_t dma_init_structure;
        circular_buffer_t* parent_buffer = (circular_buffer_t*)timer_handle->additional_buffer->parent;
        dma_stop(timer_handle->dma);

        dma_init_structure.memory_address = timer_handle->additional_buffer->buffer;
        timer_handle->additional_buffer->buffer_size = parent_buffer->buffer_size / parent_buffer->item_size;

        dma_init_structure.periph_address = timer_handle->custom_dma_src;
        dma_init_structure.data_size = timer_handle->additional_buffer->buffer_size;
        dma_init_structure.bytes = 1;
        dma_init_structure.direction = DMA_TO_MEMORY;
        dma_init(timer_handle->dma,&dma_init_structure);

        dma_start(timer_handle->dma,1);
    }
}

void timer_target_custom_dma_stop(timer_handle_t* timer_handle){
    if(timer_handle->dma){
        dma_stop(timer_handle->dma);
    }
}

void timer_target_configure_custom_dma(timer_handle_t* timer_handle, void* src_address, additional_buffer_t* buffer){
    if(timer_handle->dma == 0){
        timer_handle->dma = timer_target_find_dma(timer_handle->timer_id);
    }

    if(timer_handle->dma != 0){
        timer_handle->custom_dma_src = src_address;

        buffer->control_arg = timer_handle;
        buffer->start_callback=(buffer_start_callback_t)timer_target_custom_dma_start;
        buffer->stop_callback=(buffer_start_callback_t)timer_target_custom_dma_stop;
        /*
        No need for deinitialization callback, since the DMA is linked
        to ADC timer which is deinitialized by adc_deinit
        */

        timer_handle->additional_buffer = buffer;

        timer_handle->regs->DIER |= TIM_DIER_UDE;
    }
    else {
        buffer->start_callback = 0;
        buffer->stop_callback = 0;
    }
}

int timer_target_get_flag(timer_handle_t* timer_handle, timer_flag_t flag){
    if(flag == TIMER_FLAG_UPDATE){
        return (timer_handle->regs->SR & TIM_SR_UIF) > 0;
    }
    else if(flag == TIMER_FLAG_CHANNEL_CAPTURED){
        uint32_t flag_bit = (1 << timer_handle->main_channel);
        return (timer_handle->regs->SR & flag_bit) > 0;
    }
    else {
        return 0;
    }
}

int timer_target_clear_flag(timer_handle_t* timer_handle, timer_flag_t flag){
    if(flag == TIMER_FLAG_UPDATE){
        timer_handle->regs->SR &= ~TIM_SR_UIF;
    }
    else if(flag == TIMER_FLAG_CHANNEL_CAPTURED){
        uint32_t flag_bit = (1 << timer_handle->main_channel);
        timer_handle->regs->SR &= ~flag_bit;
    }
    return ERROR_NONE;
}


int timer_target_update_prescaler(timer_handle_t* timer_handle,int value){
    uint32_t psc = timer_handle->regs->PSC + 1;
    uint32_t inc = psc >> 3;
    if(inc == 0) inc = 1;
    if(value < 0){
        psc = psc - inc;
    }
    else {
        /* TODO: define max PSC value */
        if(timer_handle->regs->PSC < 0x8000){
            psc = psc + inc;
        }
    }
    if(psc == 0) psc = 1;
    timer_handle->regs->PSC = psc-1;
    timer_handle->regs->EGR |= 0x1;
    return ERROR_NONE;
}

uint32_t timer_target_get_resolution(timer_handle_t* timer_handle){
    uint32_t clocks = timer_target_get_clocks(timer_handle);
    return clocks / (timer_handle->regs->PSC+1);
}

uint32_t timer_target_get_frequency_divider(timer_handle_t* timer_handle){
    return (timer_handle->regs->PSC+1) * (timer_handle->regs->ARR+1);
}

int timer_target_get_pwm_input_value(timer_handle_t* timer_handle, uint32_t* freq_out, uint32_t* duty_cycle_out){
    uint32_t v1 = timer_handle->regs->CCR1 + PWM_INPUT_RESET_DELAY;
    uint32_t v2 = timer_handle->regs->CCR2 + PWM_INPUT_RESET_DELAY;

    if(timer_handle->main_channel == 1){
        *freq_out = v1;
        *duty_cycle_out = v2;
    }
    else {
        *freq_out = v2;
        *duty_cycle_out = v1;
    }

    return ERROR_NONE;
}

static void timer_target_configure_channel(timer_handle_t* timer_handle, int channel, uint32_t ccmr_set){
    uint32_t ccmr_clr = 0xFF00FF;
    volatile uint32_t* ccmr;
    if((channel & 0x1) == 0){
        ccmr_clr = ccmr_clr << 8;
        ccmr_set = ccmr_set << 8;
    }

    if(channel <= 2){
        ccmr = &timer_handle->regs->CCMR1;
    }
#ifdef STM32F3XX
    else if(channel > 4 && channel <= 6) {
        ccmr = &timer_handle->regs->CCMR3;
    }
#endif
    else {
        ccmr = &timer_handle->regs->CCMR2;
    }

    *ccmr = ((*ccmr) & ~ccmr_clr) | ccmr_set;
}

int timer_target_change_time(timer_handle_t* timer_handle, timer_time_type_t type, uint32_t value){

    uint32_t divider;
    uint32_t base_frequency = timer_target_get_clocks(timer_handle);
    uint16_t prescaler,arr;

    /* Compute divider */
    if(type == TIMER_INIT_FREQUENCY){
        divider = base_frequency / value;
    }
    else {
        /* Assuming base_frequency is rounded to MHz */
        divider = (base_frequency / 1000000) * value;
    }

    /* split divider to prescaler & autoreload */
    prescaler = prescaler_split(divider,&arr);

    timer_handle->regs->PSC = prescaler-1;
    timer_handle->regs->ARR = arr-1;

    /* generate update event (reset timer)
        otherwise it might be blocked for long time */
    timer_handle->regs->EGR = 0x1;

    return ERROR_NONE;
}

int timer_target_change_duty_cycle(timer_handle_t* timer_handle, uint16_t value){
    uint32_t ccr = (((uint32_t)(timer_handle->regs->ARR+1) * value) / 1000);

    if(timer_handle->main_channel < 5){
        *(((uint32_t*)&timer_handle->regs->CCR1) + (timer_handle->main_channel-1)) = ccr;
    }
    return ERROR_NONE;
}

int timer_target_find_channel(int timer_id, gpio_pin_t pin,uint8_t *af_out){
    int i;
    for(i = 0; i < TIM_PIN_DB_SIZE;++i){
        if(TIMER_PIN_DB_TIMER_ID(timer_pin_db[i].tim_channel) == timer_id &&
            timer_pin_db[i].pin == pin){
            *af_out = timer_pin_db[i].af;
            return TIMER_PIN_DB_CHANNEL(timer_pin_db[i].tim_channel);
        }
    }
    return 0;
}

int timer_target_find_looped_timer(int timer_id,
        int* timer_id_out, uint8_t* itr_master, uint8_t* itr_slave){
    int i;
    for(i = 0; i < TIM_SLAVE_DB_SIZE;++i){
        if(timer_slave_db[i].master_timer == timer_id){
            int ri = timer_slave_db[i].reverse_index;
            if(ri > 0 || (timer_slave_db[0].reverse_index == i && timer_slave_db[0].slave_timer == timer_id)){
                if(timer_is_free(timer_slave_db[i].slave_timer)){
                    (*timer_id_out) = timer_slave_db[i].slave_timer;
                    (*itr_master) = timer_slave_db[ri].itr_value;
                    (*itr_slave) = timer_slave_db[i].itr_value;
                    return 1;
                }
            }
        }
    }
    return 0;
}

int timer_target_find_master_timer(int timer_id,int* timer_id_out,uint8_t* itr_out){
    int i;
    for(i = 0; i < TIM_SLAVE_DB_SIZE;++i){
        if(timer_slave_db[i].slave_timer == timer_id && timer_is_free(timer_slave_db[i].master_timer)){
            (*timer_id_out) = timer_slave_db[i].master_timer;
            (*itr_out) = timer_slave_db[i].itr_value;
            return 1;
        }
    }
    return 0;
}

uint8_t timer_target_find_etr_af(int timer_id,gpio_pin_t pin){
    int i;
    for(i = 0;i < TIM_ETR_DB_SIZE;++i){
        if(TIMER_PIN_DB_TIMER_ID(timer_etr_db[i].tim_channel) == timer_id &&
            timer_etr_db[i].pin == pin){
            return timer_etr_db[i].af;
        }
    }
    return 0;
}

static int timer_target_find_config(timer_handle_t* timer_handle, int timer_id){
    int i;
    for(i = 0; i < TIM_CONFIG_DB_SIZE;++i){
        if(timer_config_db[i].timer_id == timer_id){
            timer_handle->regs = timer_config_db[i].regs;
            timer_handle->type = timer_config_db[i].type;
            timer_handle->channels = timer_config_db[i].channels;
#ifdef TIM_TARGET_APB_PRESCALERS
            timer_handle->periph_prescaler = timer_config_db[i].prescaler;
#else
            timer_handle->periph_prescaler = 1;
#endif
            timer_handle->config_index = (uint8_t)i;

            /* Enable timer clock */
            (*timer_config_db[i].port_enr) |= timer_config_db[i].port_enr_bit;
            return 1;
        }
    }
    return 0;
}

dma_handle_t timer_target_find_dma(int timer_id){
    int i;
    for(i = 0; i < TIM_DMA_DB_SIZE;++i){
        if(timer_dma_db[i].timer_id == timer_id){
            #ifdef STM32_HASDMAMUX
            dma_handle_t result = stm32_find_dma();
            stm32_dma_config_mux(result, timer_dma_db[i].dmamux_select);
            return result;
            #else
            return timer_dma_db[i].dma;
            #endif
        }
    }
    return (dma_handle_t)0;
}

int timer_target_start(timer_handle_t* timer_handle){
    if(timer_handle->usage == TIMER_USAGE_PULSE_COUNTER && timer_handle->help_timer){
        timer_handle->regs->EGR = TIM_EGR_UG;
        timer_handle->regs->SR &= ~TIM_SR_UIF;
        timer_handle->regs->CR1 |= TIM_CR1_CEN;
        timer_handle->help_timer->regs->SR &= ~TIM_SR_UIF;
        timer_handle->help_timer->regs->CR1 |= TIM_CR1_CEN;
    }
    else {
        timer_handle->regs->CR1 |= TIM_CR1_CEN;
    }
#if !defined(STM32L0XX)
    if(timer_handle->type & TIMER_TYPE_HAS_COMPLEMENTARY){
        timer_handle->regs->BDTR |= TIM_BDTR_MOE;
    }
#endif
    return 0;
}

int timer_target_get_counter_value(timer_handle_t* timer_handle, uint32_t *value_out){
    if(timer_handle->usage == TIMER_USAGE_PULSE_COUNTER && timer_handle->help_timer){
        if(timer_handle->regs->SR & TIM_SR_UIF){
            timer_handle->regs->SR &= ~TIM_SR_UIF;
            (*value_out) += 0x10000;
        }

        if(timer_handle->help_timer->regs->SR & TIM_SR_UIF){
            (*value_out) += timer_handle->regs->CNT;
            return 1;
        }
    }
    return 0;
}

int timer_target_stop(timer_handle_t* timer_handle){
    timer_handle->regs->CR1 &= ~TIM_CR1_CEN;
    return 0;
}
