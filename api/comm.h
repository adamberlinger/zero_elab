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
 * \file comm.h
 * \brief API for implementing communication interface
 */
#ifndef _API_COMM_H_
#define _API_COMM_H_

#include <stdint.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * \brief Callback type for receiving data from interface
 * \param handle User argument specified in
 * \ref comm_t.hw_handle
 * \param buffer Data to be read
 * \param size Size of data to be read
 * \return Actuall number of data that were read
 *
 * The function should work in non-blocking mode, if there are no data available
 * it just returns zero. Higher layer function like comm_read() can make
 * repetitive calls to make the read operation blocking.
 */
typedef uint32_t (*comm_read_callback_t)(void* handle,char* buffer,uint32_t size);
/**
 * \brief Callback type for transmitting data from interface
 * \param handle User argument specified in
 * \ref comm_t.hw_handle
 * \param buffer Data to be written
 * \param size Size of data to be written
 * \return Actuall number of data that were written
 *
 * The function should work in non-blocking mode, if there can be any data
 * written it just returns zero. Higher layer function like comm_write()
 * can make repetitive calls to make the write operation blocking.
 */
typedef uint32_t (*comm_write_callback_t)(void* handle,const char* buffer,uint32_t size);
/**
 * \brief Callback type for flushing data from interface
 * \param handle User argument specified in
 * \ref comm_t.hw_handle
 *
 * This function should make sure all data written to the interface
 * were actually written. This can be e.g. useful for USB Virtual COM port (CDC)
 * implementation, since the data are passed to higher level
 * (e.g. PC application) from the driver after receiving either:
 * - non-full packet (packet size is not maximum allowed)
 * - receiving empty packet
 *
 * If transmitted data are multiply of maximum packet size,
 * the data are not passed to PC application without explicit flush
 */
typedef uint32_t (*comm_flush_callback_t)(void* handle);

/**
 * \brief Structure for representing bidirectional communication interface
 */
typedef struct {
    /** \brief Pointer to implementation specific data */
    void* hw_handle;
    /** \brief Read operation callback */
    comm_read_callback_t read_callback;
    /** \brief Write operation callback */
    comm_write_callback_t write_callback;
    /** \brief Flush operation callback */
    comm_flush_callback_t flush_callback;
}comm_t;

/**
 * \brief Writes data to interface
 * \param comm Pointer to communication interface
 * \param data Pointer to data to be written
 * \param size Size of data to be written
 * \return Number of data written. Returns zero if
 * \ref comm_t.write_callback is NULL.
 */
uint32_t comm_write(comm_t* comm,const char* data,uint32_t size);
/**
 * \brief Reads data from interface
 * \param comm Pointer to communication interface
 * \param data Pointer where should be the read data stored
 * \param size Size of data to be read
 * \param blocking If non-zero, the function blocks till all data are read
 * \return Number of data read. Returns zero if
 * \ref comm_t.read_callback is NULL.
 */
uint32_t comm_read(comm_t* comm,char* data,uint32_t size, int blocking);
/**
 * \brief Flushes the interface
 * \param comm Pointer to communication interface
 */
void comm_flush(comm_t* comm);

#ifdef __cplusplus
    }
#endif

#endif /* _API_COMM_H_ */
