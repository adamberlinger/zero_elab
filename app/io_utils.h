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
 * \file io_utils.h
 * \brief Utilities for communicating with PC application
 */
#ifndef _APP_PRINT_UTILS_
#define _APP_PRINT_UTILS_

#include <stdint.h>
#include "comm.h"

#ifdef __cplusplus
    extern "C" {
#endif

/** \brief Size of command in bytes */
#define COMMAND_DATA_SIZE   (7)

/** \brief Converts 16-bit to 8-bit array literals in little-endian format */
#define UINT16_TO_U8ARRAY_LE(value) (value & 0xFF), (value & 0xFF00) >> 8

/** \brief Converts 32-bit to 8-bit array literals in little-endian format */
#define UINT32_TO_U8ARRAY_LE(value) (value & 0xFF), (value & 0xFF00) >> 8, \
    (value & 0xFF0000) >> 16, (value & 0xFF000000) >> 24

/**
 * \brief Structure representing command
 */
typedef struct {
    /** \brief Prameter of the command */
    uint32_t value;
    /** \brief Command ID */
    uint8_t command_id;
    /** \brief Channel */
    uint8_t channel;

    /** \brief Number of data read by interface */
    uint8_t data_readed;
    /** \brief Raw data before parsing command */
    uint8_t raw_data[COMMAND_DATA_SIZE];
}command_t;

/**
 * \brief Sends data including standard header
 * \param comm Communication interface
 * \param channel Communication channel
 * \param data Pointer to data
 * \param length Size of data in bytes
 */
void send_binary_data(comm_t* comm,uint8_t channel,char* data,uint16_t length);
/**
 * \brief Sends standard header
 * \param comm Communication interface
 * \param channel Communication channel
 * \param length Size of data in bytes
 *
 * This is useful when data to be sent are not stored in one linear buffer.
 * E.g. when sending data from circular buffer it needs to be sent in two chunks.
 */
void send_binary_data_header(comm_t* comm,uint8_t channel,uint16_t length);
/**
 * \brief Non-blocking function for receiving command
 * \param comm Communication interface
 * \param command Readed command
 * \return Non-zero value if command was received
 */
int receive_command(comm_t* comm,command_t* command);
/**
 * \brief Converts samplerate from pseudo-float 16-bit value
 * to 32-bit integer value
 * \return Value in Hz
 * \deprecated This should be removed since command now Contains
 * 32-bit parameter value
 */
uint32_t convert_samplerate(uint16_t input);
/**
 * \brief Sends the command
 * \param comm Communication interface
 * \param channel Communication channel
 * \param command_id Command number
 * \param value Value
 */
void send_command(comm_t* comm,uint8_t channel, uint8_t command_id, uint32_t value);

#ifdef __cplusplus
    }
#endif

#endif /* _APP_PRINT_UTILS_ */
