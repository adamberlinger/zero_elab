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
/** \file circular_buffer.h */
#ifndef _CYCLIC_BUFFER_H_
#define _CYCLIC_BUFFER_H_

#include "core.h"

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * \brief Callback type for processing data
 * \param user_arg User argument specified in
 * \ref circular_buffer_t.process_arg
 * \param data Pointer to data that should be processed
 * \param data_length Size of data in bytes
 */
typedef void (*buffer_process_callback_t)(void* user_arg,uint8_t* data,uint32_t data_length);

/**
 * \brief Callback type to start data stream
 * \param user_arg User argument specified in
 * \ref circular_buffer_t.control_arg
 */
typedef void (*buffer_start_callback_t)(void* user_arg);
/**
 * \brief Callback type to stop data stream
 * \param user_arg User argument specified in
 * \ref circular_buffer_t.control_arg
 */
typedef void (*buffer_stop_callback_t)(void* user_arg);

/**
 * \brief Callback type to deinitialize data stream
 * \param user_arg User argument specified in
 * \ref circular_buffer_t.control_arg
 */
typedef void (*buffer_deinit_callback_t)(void* user_arg);
/**
 * \brief Callback type to start data stream precisely see
 * \ref buffer_stop_precise_rw for more info
 * \param user_arg User argument specified in
 * \ref circular_buffer_t.control_arg
 * \param sample_number Index of sample where to stop
 */
typedef void (*buffer_stop_precise_callback_t)(void* user_arg, uint32_t sample_number);
/**
 * \brief Callback type to get stop state see
 * \ref buffer_get_stop_state for more info
 * \param user_arg User argument specified in
 * \ref circular_buffer_t.control_arg
 * \param index_out Index where data stream stopped
 * \return Non-zero value if data stream stopped
 */
typedef int (*buffer_get_stop_state_callback_t)(void* user_arg,uint32_t *index_out);
/**
 * \brief Callback type to set channels see
 * \ref buffer_set_channels for more info
 * \param user_arg User argument specified in
 * \ref circular_buffer_t.control_arg
 * \param channels_mask Bit field, specifing which channel is on
 * \return Number of channels enabled
 */
typedef uint32_t (*buffer_set_channels_callback_t)(void* user_arg,uint32_t channels_mask);
/**
 * \brief Callback type to set the sample rate
 * \ref buffer_change_samplerate for more info
 * \param user_arg User argument specified in
 * \ref circular_buffer_t.samplerate_arg
 * \param value Sample rate in Hz
 * \param prescaler_out Prescaler value configured by timer (if any)
 * \return Real setup frequency, before divided by \ref prescaler
 */
typedef uint32_t (*buffer_samplerate_callback_t)(void* user_arg, uint32_t value, uint32_t *prescaler_out);


/**
 * \brief Specifier of buffer type used for communication with PC application
 */
typedef enum {
    /** \brief Data represented as number (e.g. analog channel) */
    BUFFER_TYPE_NUMBER = 0,
    /** \brief Data represented as bits (e.g. digital channels) */
    BUFFER_TYPE_BITS = 1,
}buffer_type_t;

typedef enum {
    BUFFER_ATTRIBUTE_MAX_IMPEDANCE,
}buffer_attribute_t;
/**
 * \brief Callback type to get custom attributes
 * \ref buffer_get_attribute for more info
 * \param user_arg User argument specified in
 * \ref circular_buffer_t.control_arg
 * \param name Attribute the function should read
 * \return 0 - attribute returned in value_out, other - attribute not supported
 */
typedef int (*buffer_get_attribute_callback_t)(void* user_arg, buffer_attribute_t name, void* value_out);

/**
 * \brief Macro for creating buffer header
 * \param channels Number of channels in buffer
 * \param sample_size Size of sample in bytes
 * \param type Type specified as buffer_type_t
 */
#define BUFFER_FORMAT(channels,sample_size,type)    (uint8_t)((channels & 0xF) << 4) \
    | ((sample_size & 0x3) << 2) | (type & 0x3)

/**
 * \brief API for additional circular buffer
 *
 * This is used for synchronous digital channels.
 * This is useful when data from another synchronous
 * stream are stored in separate circular buffer.
 */
typedef struct _additional_buffer {
    /** \brief Pointer to data memory */
    uint8_t *buffer;
    /** \brief Actuall buffer size in bytes */
    uint32_t buffer_size;
    /** \brief Maximum size of buffer in bytes */
    uint32_t allocated_size;
    /** \brief Pointer to next buffer in linked list */
    struct _additional_buffer *next;

    /** \brief Argument passed to start_callback and stop_callback */
    void *control_arg;
    /** \brief Callback for starting data stream */
    buffer_start_callback_t start_callback;
    /** \brief Callback for stopping data stream immediately */
    buffer_stop_callback_t stop_callback;
    /** \brief Callback to deinitialize data stream */
    buffer_deinit_callback_t deinit_callback;
    /**
     * \brief Pointer to parent circular_buffer_t structure
     *
     * This needs to be void* because circular_buffer_t is defined later
     */
    void* parent;
} additional_buffer_t;

/**
 * \brief API for interfacing data streams
 *
 * Circular buffer is a advanced API for interfacing
 * algorithms such as oscilloscope or DDS generator
 * with data streams.
 *
 * Circular buffer can act as an output or an input.
 * There is no flag specifing the flow direction, but
 * the behaviour is different.
 *
 * In both cases, the circular buffer generates process callback
 * (process_callback)
 * that notifies the algorithm that new data are available (for input),
 * or that new data should be generated (for output).
 */
typedef struct {
    /** \brief Pointer to data memory */
    uint8_t* buffer;
    /** \brief Actuall buffer size in bytes */
    uint32_t buffer_size;
    /** \brief Maximum size of buffer in bytes */
    uint32_t allocated_size;
    /** \brief Size of one sample in bytes */
    uint32_t item_size;
    /** \brief Linked list to additional buffers (if any) */
    additional_buffer_t* additional_buffer;

    /** \brief Argument passed to process_callback */
    void *process_arg;
    /** \brief Callback called when new data are available/requested */
    buffer_process_callback_t process_callback;

    /** \brief Argument passed to other callbacks (not process_callback) */
    void *control_arg;
    /** \brief Callback for starting data stream */
    buffer_start_callback_t start_callback;
    /** \brief Callback for stopping data stream immediately */
    buffer_stop_callback_t stop_callback;
    /** \brief Callback for stopping data stream at precise sample */
    buffer_stop_precise_callback_t stop_precise_callback;
    /** \brief Callback to get status after calling stop_precise_callback */
    buffer_get_stop_state_callback_t get_stop_state_callback;
    /** \brief Callback to set number of channels */
    buffer_set_channels_callback_t set_channels_callback;
    /** \brief Callback to deinitialize data stream */
    buffer_deinit_callback_t deinit_callback;
    /** \brief Callback to read attributes */
    buffer_get_attribute_callback_t get_attribute_callback;

    /** \brief Arguemnt passed to samplerate_callback */
    void *samplerate_arg;
    /** \brief Callback to change sample rate of data stream */
    buffer_samplerate_callback_t samplerate_callback;
}circular_buffer_t;

/**
 * \brief Initializes all circular_buffer_t fields with default values
 * \param circular_buffer Pointer to circular buffer
 */
void buffer_init(circular_buffer_t* circular_buffer);
/**
 * \brief Deinitialize the circular_buffer and all linked additional buffers
 * \param circular_buffer Pointer to circular buffer
 */
void buffer_deinit(circular_buffer_t* circular_buffer);
/**
 * \brief Allocates memory for circular buffer
 * \param circular_buffer Pointer to circular buffer
 * \param size Size of the buffer in bytes
 */
int buffer_alloc(circular_buffer_t* circular_buffer, int size);
/**
 * \brief Allocates memory for additional buffer
 * \param circular_buffer Pointer to circular buffer
 * \param size Size of the additional buffer in bytes
 * \return Pointer to newly allocated buffer
 */
additional_buffer_t* buffer_additional_alloc(circular_buffer_t* circular_buffer, int size);
/**
 * \brief Gets number of additional buffers
 * \param circular_buffer Pointer to circular buffer
 * \return Number of additional buffers
 */
int buffer_get_count(circular_buffer_t* circular_buffer);
/**
 * \brief Changes size of the buffer
 * \param circular_buffer Pointer to circular buffer
 * \param newSize New size of the buffer in bytes
 *
 * The newSize value must be less or equal to allocated size of the buffer.
 * This is usually done in following sequence:
 * - Stop the buffer by calling buffer_stop_rw() (if it was running)
 * - Call buffer_resize()
 * - Start the buffer again by calling buffer_start_rw(). The callback reads
 *   the newly set value in the circular_buffer_t structure.
 */
void buffer_resize(circular_buffer_t* circular_buffer, int newSize);
/**
 * \brief Starts the data stream by calling start callbacks
 * \param circular_buffer Pointer to circular buffer
 *
 * This function calls \ref circular_buffer_t.start_callback
 * and \ref additional_buffer_t.start_callback callbacks in the following order:
 * - call \ref additional_buffer_t.start_callback callbacks in linked order
 * - call \ref circular_buffer_t.start_callback callback
 *
 * Callback values are checked and if it is set to NULL it is not called.
 */
void buffer_start_rw(circular_buffer_t* circular_buffer);
/**
 * \brief Stops the data stream by calling stop callbacks
 * \param circular_buffer Pointer to circular buffer
 *
 * This function calls \ref circular_buffer_t.start_callback
 * and \ref additional_buffer_t.stop_callback callbacks in the following order:
 * - call \ref additional_buffer_t.stop_callback callbacks in linked order
 * - call \ref circular_buffer_t.stop_callback callback
 *
 * Callback values are checked and if it is set to NULL it is not called.
 */
void buffer_stop_rw(circular_buffer_t* circular_buffer);
/**
 * \brief Stops the data stream at precise sample by calling
 * \ref circular_buffer_t.stop_precise_callback
 * \param circular_buffer Pointer to circular buffer
 * \param sample_number Index of sample in circular buffer where to stop
 *
 * This function calls \ref circular_buffer_t.stop_precise_callback.
 * Callback value is checked and if it is set to NULL it is not called.
 *
 * The precise stop should be implemented in hardware e.g. using timers,
 * because software implementation isn't usually reliable.
 * There is no stop_precise_callback in additional_buffer_t, since
 * this concept assumes that additional buffers are stopped synchronously
 * by hardware.
 */
void buffer_stop_precise_rw(circular_buffer_t* circular_buffer,uint32_t sample_number);
/**
 * \brief Gets status after calling buffer_stop_precise_rw() function
 * \ref circular_buffer_t.stop_precise_callback
 * \param circular_buffer Pointer to circular buffer
 * \param index_out Index of sample where buffer actually stopped
 * \return Non-zero value if buffer has stopped
 *
 * This function calls \ref circular_buffer_t.get_stop_state_callback.
 * Callback value is checked and if it is set to NULL it is not called.
 *
 * The returned index (index_out) can be different from sample_number
 * passed to buffer_stop_precise_rw() function. This happens, when the hardware
 * is not fast enough to stop the data stream on time.
 */
int buffer_get_stop_state(circular_buffer_t* circular_buffer,uint32_t* index_out);

int buffer_get_attribute(circular_buffer_t* circular_buffer, buffer_attribute_t name, void* value_out);
/**
 * \brief Changes data stream sample rate by calling
 * \ref circular_buffer_t.samplerate_callback
 * \param circular_buffer Pointer to circular buffer
 * \param value Sampling frequency in Hz
 * \param prescaler_out Prescaler value configured by timer (if any)
 * \return Real setup frequency, before divided by \ref prescaler
 *
 * This function calls \ref circular_buffer_t.samplerate_callback.
 * Callback value is checked and if it is set to NULL it is not called.
 */
uint32_t buffer_change_samplerate(circular_buffer_t* circular_buffer,uint32_t value,uint32_t* prescaler_out);
/**
 * \brief Sets active channels
 * \ref circular_buffer_t.samplerate_callback
 * \param circular_buffer Pointer to circular buffer
 * \param channel_mask Bit field, specifing which channel is on
 * \return Number of active channels
 *
 * This function calls \ref circular_buffer_t.set_channels_callback.
 * The returned value doesn't have to correspond to number of setup bits
 * in channel_mask parameter, beacuse the number of channels is limited.
 */
uint32_t buffer_set_channels(circular_buffer_t* circular_buffer,uint32_t channel_mask);

#ifdef __cplusplus
    }
#endif

#endif /* _CYCLIC_BUFFER_H_ */
