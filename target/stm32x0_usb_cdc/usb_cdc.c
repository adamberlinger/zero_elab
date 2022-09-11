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
#include "usb_cdc.h"

#define DEFAULT_BAUDRATE         (115200U)

#ifndef CDC_RX_BUFFER_SIZE
#define CDC_RX_BUFFER_SIZE      0
#endif

/* Force 16-bit alignment */
uint16_t line_coding[4] = {
  (DEFAULT_BAUDRATE & 0xFFFF),
  ((DEFAULT_BAUDRATE >> 16) & 0xFFFF),
  0, /* One stop bit and no parity */
  8, /* 8 data bits */
};

#if CDC_RX_BUFFER_SIZE > 0
  static volatile uint8_t rx_buffer[CDC_RX_BUFFER_SIZE];
  static volatile uint32_t rx_read_ptr = 0;
  static volatile uint32_t rx_write_ptr = 0;
#else
  static volatile uint8_t *rx_read_ptr = 0;
  static volatile uint8_t rx_read_ptr_size = 0;
#endif


void usb_cdc_request(const usb_request_t *request, const uint8_t *request_data){
  /* Get line coding */
  if(request->request_id == 0x21){
    usb_core_transmit0((uint8_t*)line_coding,7);
  }
  /* Set line coding */
  else if(request->request_id == 0x20){
    mem_copy16_fromusb(request_data, line_coding, 7);
    usb_core_status0(1);
  }
  /* Set control line state */
  else if(request->request_id == 0x22){
    usb_core_status0(1);
  }
  else {
    usb_core_status0(0);
  }
}

void usb_cdc_enable(){
  /* TODO: Re-init buffers ? */
}

void usb_cdc_data_received(uint16_t* data, uint16_t length){
#if CDC_RX_BUFFER_SIZE > 0
  uint32_t i = 0;
  for(i = 0;i < (length-1);i+=2){
    uint16_t data16 = data[i>>1];
    rx_buffer[rx_write_ptr] = (data16 & 0xFF);
    rx_write_ptr++;
    if(rx_write_ptr == CDC_RX_BUFFER_SIZE) rx_write_ptr = 0;
    rx_buffer[rx_write_ptr] = (data16 >> 8) & 0xFF;
    rx_write_ptr++;
    if(rx_write_ptr == CDC_RX_BUFFER_SIZE) rx_write_ptr = 0;
  }

  if(i == (length-1)){
    rx_buffer[rx_write_ptr] = data[i>>1] & 0xFF;
    rx_write_ptr++;
    if(rx_write_ptr == CDC_RX_BUFFER_SIZE) rx_write_ptr = 0;
  }
#else
  rx_read_ptr = (uint8_t*)data;
  rx_read_ptr_size = length;
#endif
}

uint8_t usb_cdc_rx_ready(){
#if CDC_RX_BUFFER_SIZE > 0
  uint32_t free_size;
  uint32_t rp = rx_read_ptr;
  uint32_t wp = rx_write_ptr;
  if(wp < rp){
    free_size = rp - wp;
  }
  else {
    free_size = rp + CDC_RX_BUFFER_SIZE - wp;
  }

  return (free_size > 64);
#else
  return rx_read_ptr_size == 0;
#endif
}

uint8_t usb_cdc_getc(){
#if CDC_RX_BUFFER_SIZE > 0
  while(rx_read_ptr == rx_write_ptr);
  uint8_t result = rx_buffer[rx_read_ptr];
  if((rx_read_ptr+1) == CDC_RX_BUFFER_SIZE) rx_read_ptr = 0;
  else rx_read_ptr++;
  return result;
#else
  while(rx_read_ptr_size == 0);
  uint8_t result;
  mem_copy16to8((void*)rx_read_ptr, &result,1);
  rx_read_ptr++;
  rx_read_ptr_size--;
  return result;
#endif
}

uint32_t usb_cdc_read(uint8_t* data, uint32_t length){
#if CDC_RX_BUFFER_SIZE > 0
  if(rx_read_ptr != rx_write_ptr){
    uint32_t i = 0;
    while(rx_read_ptr != rx_write_ptr && i < length){
      data[i] = rx_buffer[rx_read_ptr];
      if((rx_read_ptr+1) == CDC_RX_BUFFER_SIZE) rx_read_ptr = 0;
      else rx_read_ptr++;
    }
    return i;
  }
#else
  if(rx_read_ptr_size > 0){
    uint32_t copy_size = (length > rx_read_ptr_size)?rx_read_ptr_size:length;
    mem_copy16to8((void*)rx_read_ptr, data, copy_size);
#if USB_SRAM_MULT > 1
    rx_read_ptr += (copy_size & 0xFFFFFFFE) * USB_SRAM_MULT + (copy_size & 0x1);
#else
    rx_read_ptr += copy_size;
#endif
    rx_read_ptr_size -= copy_size;
    return copy_size;
  }
#endif
  return 0;
}

void usb_cdc_write(const uint8_t* data, uint16_t length){
  if(length == 0) return;
  while(!usb_core_is_ep1_free());
  usb_core_transmit1((uint8_t*)data, length);
  /* TODO: only check if all data copied to USB SRAM */
  while(!usb_core_is_ep1_free());
}
