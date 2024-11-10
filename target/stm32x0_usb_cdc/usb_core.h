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
#ifndef _USB_CORE_H_
#define _USB_CORE_H_

#include "usb_conf.h"
#include <stdint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#ifdef USB_SRAM_32BIT
  typedef uint32_t usb_sram_t;
  #define USB_SRAM_MULT 2
#else
  typedef uint16_t usb_sram_t;
  #define USB_SRAM_MULT 1
#endif

#define USB_NUM_BUFFERS          (4)
#define USB_NUM_TRANSMIT_BUFFERS (2)

#define USB_EPx(ep) (*(((uint16_t*)&USB->EP0R) + (ep)*2))

#ifdef USB_SRAM_32ACCESS
typedef struct {
    /* E.g. on C0, registers allow only 32-bit access */
    volatile uint32_t tx_conf;
    volatile uint32_t rx_conf;
}buffer_table_row_t;
#elif defined(STM32F1XX)
typedef struct {
    volatile uint32_t addr_tx;
    volatile uint32_t count_tx;
    volatile uint32_t addr_rx;
    volatile uint32_t count_rx;
}buffer_table_row_t;
#else
typedef struct {
    volatile uint16_t addr_tx;
    volatile uint16_t count_tx;
    volatile uint16_t addr_rx;
    volatile uint16_t count_rx;
}buffer_table_row_t;
#endif

typedef struct {
  const uint8_t* data;
  volatile uint32_t size;
  volatile uint32_t transfered;
}usb_transfer_t;

typedef enum {
  USB_CONTROL_IDLE,
  USB_CONTROL_DATA_RX,
  USB_CONTROL_DATA_TX,
}usb_control_state_t;

typedef struct {
  uint8_t request_type;
  uint8_t request_id;
  uint16_t value;
  uint16_t index;
  uint16_t length;
}usb_request_t;

typedef struct {
  uint32_t address;
  uint32_t ms_ticks;
  uint8_t address_pending;

  usb_control_state_t control_state;
  usb_request_t request;

  usb_transfer_t transfers[USB_NUM_TRANSMIT_BUFFERS];
}usb_state_t;

int usb_core_init(void);
void usb_core_reset(void);
void usb_core_ep_tx(uint32_t ep);
void usb_core_ep_rx(uint32_t ep);
void usb_interrupt(void);

void usb_core_transmit0(const uint8_t* data,uint16_t size);
void usb_core_transmit1(const uint8_t* data,uint16_t size);
void usb_core_status0(int success);
void usb_core_handle_request(const usb_request_t *request, const uint8_t* request_data);
uint8_t usb_core_is_ep1_free(void);

void mem_copy16_tousb(const void* source, void* dest, uint16_t size);
void mem_copy16_fromusb(const void* source, void* dest, uint16_t size);
void mem_copy16to8(const void* source, void* dest, uint16_t size);
void mem_copy8to16(const void* source, void* dest, uint16_t size);

#ifdef USB_SRAM_32ACCESS
#define USB_SET_ADDR_TX(x, val)  (x).tx_conf = (((x).tx_conf & 0xFFFF0000) | (val))
#define USB_SET_ADDR_RX(x, val)  (x).rx_conf = (((x).rx_conf & 0xFFFF0000) | (val))
#define USB_SET_COUNT_TX(x, val)  (x).tx_conf = (((x).tx_conf & 0xFFFF) | ((val) << 16))
#define USB_SET_COUNT_RX(x, val)  (x).rx_conf = (((x).rx_conf & 0xFFFF) | ((val) << 16))

#define USB_GET_ADDR_TX(x)  (uint16_t)((x).tx_conf & 0xFFFF)
#define USB_GET_ADDR_RX(x)  (uint16_t)((x).rx_conf & 0xFFFF)
#define USB_GET_COUNT_TX(x)  (uint16_t)(((x).tx_conf & 0xFFFF0000) >> 16)
#define USB_GET_COUNT_RX(x)   (uint16_t)(((x).rx_conf & 0xFFFF0000) >> 16)
#else
#define USB_SET_ADDR_TX(x, val)  (x).addr_tx = (val)
#define USB_SET_ADDR_RX(x, val)  (x).addr_rx = (val)
#define USB_SET_COUNT_TX(x, val)  (x).count_tx = (val)
#define USB_SET_COUNT_RX(x, val)  (x).count_rx = (val)

#define USB_GET_ADDR_TX(x)  ((x).addr_tx)
#define USB_GET_ADDR_RX(x)  ((x).addr_rx)
#define USB_GET_COUNT_TX(x)  ((x).count_tx)
#define USB_GET_COUNT_RX(x)  ((x).count_rx)
#endif

#ifdef __cplusplus
    }
#endif

#endif /* _USB_CORE_H_ */
