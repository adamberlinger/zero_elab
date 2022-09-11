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
#include "usb_desc.h"

#define U8_TO_U16(a,b)          (((uint16_t)a) | ((uint16_t)b << 8))

/*
 * This is nasty trick to keep descriptors 16-bit aligned on all compilers.
 * But there is usually no need to modify these
 */

const uint16_t device_descriptor[9] = {
  U8_TO_U16(18,                   /* Size */
  0x01),                 /* Device descriptor */
  U8_TO_U16(0x00, 0x02),           /* USB 2.0 */
  U8_TO_U16(0x02,                 /* Device class: CDC */
  0x02),                 /* Device subclass: CDC */
  U8_TO_U16(0x00,                 /* Device protocol */
  64),                   /* Max. packet size */
  U8_TO_U16(0x83, 0x04),           /* VID: 0x0483 */
  U8_TO_U16(0x40, 0x57),           /* PID: 0x5740 */
  U8_TO_U16(0x00, 0x01),           /* Device version 1.0 */
  U8_TO_U16(0x00,                 /* Manufacturer string */
  0x00),                 /* Product string */
  U8_TO_U16(0x00,                 /* Serial number */
  0x01),                 /* Number of configurations */
};

const uint16_t config_descriptor[SIZE_TO_U16(CONFIG_DESCRIPTOR_SIZE)] = {
  U8_TO_U16(9,                    /* Size */
  0x02),                 /* Configuration descriptor */
  CONFIG_DESCRIPTOR_SIZE,         /* Total length */
  U8_TO_U16(0x02,                 /* Number of interfaces */
  0x01),                 /* Configuration value */
  U8_TO_U16(0x00,                 /* Configuration index */
  0x80),
  U8_TO_U16(50,                   /* 100mA consumption */

  /*Interface Descriptor */
  0x09),   /* bLength: Interface Descriptor size */
  U8_TO_U16(0x04,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00),   /* bInterfaceNumber: Number of Interface */
  U8_TO_U16(0x00,   /* bAlternateSetting: Alternate setting */
  0x01),   /* bNumEndpoints: One endpoints used */
  U8_TO_U16(0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02),   /* bInterfaceSubClass: Abstract Control Model */
  U8_TO_U16(0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00),   /* iInterface: */

  /*Header Functional Descriptor*/
  U8_TO_U16(0x05,   /* bLength: Endpoint Descriptor size */
  0x24),   /* bDescriptorType: CS_INTERFACE */
  U8_TO_U16(0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10),   /* bcdCDC: spec release number */
  U8_TO_U16(0x01,

  /*Call Management Functional Descriptor*/
  0x05),   /* bFunctionLength */
  U8_TO_U16(0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01),   /* bDescriptorSubtype: Call Management Func Desc */
  U8_TO_U16(0x00,   /* bmCapabilities: D0+D1 */
  0x01),   /* bDataInterface: 1 */

  /*ACM Functional Descriptor*/
  U8_TO_U16(0x04,   /* bFunctionLength */
  0x24),   /* bDescriptorType: CS_INTERFACE */
  U8_TO_U16(0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02),   /* bmCapabilities */

  /*Union Functional Descriptor*/
  U8_TO_U16(0x05,   /* bFunctionLength */
  0x24),   /* bDescriptorType: CS_INTERFACE */
  U8_TO_U16(0x06,   /* bDescriptorSubtype: Union func desc */
  0x00),   /* bMasterInterface: Communication class interface */
  U8_TO_U16(0x01,   /* bSlaveInterface0: Data Class Interface */

  /*Endpoint 2 Descriptor*/
  0x07),                           /* bLength: Endpoint Descriptor size */
  U8_TO_U16(0x05,   /* bDescriptorType: Endpoint */
  0x83),                     /* bEndpointAddress */
  U8_TO_U16(0x03,                           /* bmAttributes: Interrupt */
  8),                           /* wMaxPacketSize: */
  U8_TO_U16(0,
  0x10),                           /* bInterval: */

  U8_TO_U16(0x09,
  0x04),                 /* Interface descriptor */
  U8_TO_U16(0x01,                 /* Interface number */
  0x00),                 /* Alt. settings */
  U8_TO_U16(0x02,                 /* Num. endpoints */
  0x0A),                 /* Class: CDC */
  U8_TO_U16(0x00,                 /* Subclass */
  0x00),                 /* Protocol */
  U8_TO_U16(0x00,                 /* Interface string */

  /* Endpoint OUT */
  0x07),
  U8_TO_U16(0x05,                 /* Endpoint descriptor */
  0x02),                 /* EP2 */
  U8_TO_U16(0x02,                 /* Bulk transfer */
  64),
  U8_TO_U16(0,                /* 64 byte packets */
  0x0),                  /* Interval ignored */

  /* Endpoint IN */
  U8_TO_U16(0x07,
  0x05),                 /* Endpoint descriptor */
  U8_TO_U16(0x81,                 /* EP1 */
  0x02),                 /* Bulk transfer */
  U8_TO_U16(64, 0),                /* 64 byte packets */
  U8_TO_U16(0x0, 0)                 /* Interval ignored */
};
