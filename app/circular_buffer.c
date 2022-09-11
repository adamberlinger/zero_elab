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
/** \file circular_buffer.c */
#include "circular_buffer.h"

void buffer_init(circular_buffer_t* circular_buffer){
    circular_buffer->buffer = 0;
    circular_buffer->buffer_size = 0;
    circular_buffer->allocated_size = 0;
    circular_buffer->process_callback = 0;
    circular_buffer->start_callback = 0;
    circular_buffer->stop_callback = 0;
    circular_buffer->get_attribute_callback = 0;
    circular_buffer->set_channels_callback = 0;
    circular_buffer->samplerate_callback = 0;
    circular_buffer->additional_buffer = 0;
}

void buffer_deinit(circular_buffer_t* circular_buffer){
    additional_buffer_t* current = circular_buffer->additional_buffer;
    while(current != 0){
        if(current->deinit_callback){
            current->deinit_callback(current->control_arg);
        }
        current = current->next;
    }
    if(circular_buffer->deinit_callback){
        circular_buffer->deinit_callback(circular_buffer->control_arg);
    }
}

int buffer_alloc(circular_buffer_t* circular_buffer, int size){
    circular_buffer->buffer = (uint8_t*)module_malloc(size);
    if(circular_buffer->buffer){
        circular_buffer->buffer_size = size;
        circular_buffer->allocated_size = size;
    }
    else {
        return ERROR_OUT_OF_MEMORY;
    }
    return ERROR_NONE;
}

void buffer_resize(circular_buffer_t* circular_buffer,int newSize){
    if(circular_buffer->allocated_size >= newSize){
        circular_buffer->buffer_size = newSize;
    }
}

void buffer_start_rw(circular_buffer_t* circular_buffer){
    additional_buffer_t* current = circular_buffer->additional_buffer;
    while(current != 0){
        if(current->start_callback){
            current->start_callback(current->control_arg);
        }
        current = current->next;
    }

    if(circular_buffer->start_callback != 0){
        circular_buffer->start_callback(circular_buffer->control_arg);
    }
}

void buffer_stop_rw(circular_buffer_t* circular_buffer){
    additional_buffer_t* current = circular_buffer->additional_buffer;
    if(circular_buffer->stop_callback != 0){
        circular_buffer->stop_callback(circular_buffer->control_arg);
    }

    while(current != 0){
        if(current->stop_callback){
            current->stop_callback(current->control_arg);
        }
        current = current->next;
    }
}

void buffer_stop_precise_rw(circular_buffer_t* circular_buffer, uint32_t sample_number){
    if(circular_buffer->stop_precise_callback != 0){
        circular_buffer->stop_precise_callback(circular_buffer->control_arg, sample_number);
    }
}

int buffer_get_stop_state(circular_buffer_t* circular_buffer,uint32_t *index_out){
    if(circular_buffer->get_stop_state_callback != 0){
        return circular_buffer->get_stop_state_callback(circular_buffer->control_arg,index_out);
    }
    return 0;
}

int buffer_get_attribute(circular_buffer_t* circular_buffer, buffer_attribute_t name, void* value_out){
    if(circular_buffer->get_attribute_callback != 0 && value_out != NULL){
        return circular_buffer->get_attribute_callback(circular_buffer->control_arg,name,value_out);
    }
    return -1;
}

uint32_t buffer_change_samplerate(circular_buffer_t* circular_buffer,uint32_t value,uint32_t* prescaler_out){
    if(circular_buffer->samplerate_callback != 0){
        return circular_buffer->samplerate_callback(circular_buffer->samplerate_arg,value,prescaler_out);
    }
    return 0;
}

uint32_t buffer_set_channels(circular_buffer_t* circular_buffer,uint32_t num_channels){
    if(circular_buffer->set_channels_callback != 0){
        return circular_buffer->set_channels_callback(circular_buffer->control_arg, num_channels);
    }
    return 1;
}

additional_buffer_t* buffer_additional_alloc(circular_buffer_t* circular_buffer, int size){
    additional_buffer_t** last_node = &circular_buffer->additional_buffer;
    while(*last_node != 0){
        last_node = &(*last_node)->next;
    }

    (*last_node) = (additional_buffer_t*)module_malloc(sizeof(additional_buffer_t));
    memset((*last_node),0,sizeof(additional_buffer_t));
    (*last_node)->buffer = (uint8_t*)module_malloc(size);
    if((*last_node)->buffer){
        memset((*last_node)->buffer,0,size);
        (*last_node)->buffer_size = size;
        (*last_node)->allocated_size = size;
        (*last_node)->parent = circular_buffer;
    }
    return (*last_node);
}

int buffer_get_count(circular_buffer_t* circular_buffer){
    int i = 1;
    additional_buffer_t* current = circular_buffer->additional_buffer;
    while(current){
        i++;
        current = current->next;
    }
    return i;
}
