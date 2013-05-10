/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs LLC.
 * Copyright (c) 2013 Magnus Lundin.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file libmaple/include/libmaple/usb_midi_device.h
 * @brief USB MIDI support, device configuration
 *
 * IMPORTANT: this API is unstable, and may change without notice.
 */

#ifndef _LIBMAPLE_USB_MIDI_DEVICE_H_
#define _LIBMAPLE_USB_MIDI_DEVICE_H_

#include <libmaple/libmaple_types.h>
#include <libmaple/midi_specs.h>
#include <libmaple/gpio.h>
#include <libmaple/usb.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
    uint8  byte[4];
    uint32 data;
    MIDI_EVENT_PACKET_t packet;
} USB_MIDI_Event_Packet;

/*
 * USB MIDI Requests
 */

/*
 * USB Device configuration
 */

/* FIXME move to Wirish */

#define LEAFLABS_ID_VENDOR                0x1EAF
#define MAPLE_ID_PRODUCT                  0x0014

#define MAX_POWER (100 >> 1)
 
/*
 * Endpoint configuration
 */

#define USB_MIDI_CTRL_ENDP            0
#define USB_MIDI_CTRL_RX_ADDR         0x40
#define USB_MIDI_CTRL_TX_ADDR         0x80
#define USB_MIDI_CTRL_EPSIZE          0x40

#define USB_MIDI_TX_ENDP              1
#define USB_MIDI_TX_ADDR              0xC0
#define USB_MIDI_TX_EPSIZE            0x40

#define USB_MIDI_RX_ENDP              2
#define USB_MIDI_RX_ADDR              0x100
#define USB_MIDI_RX_EPSIZE            0x40

/*
 * MIDI interface
 */

void usb_midi_enable(gpio_dev*, uint8);
void usb_midi_disable(gpio_dev*, uint8);

void   usb_midi_putc(char ch);
uint32 usb_midi_tx(const uint32* buf, uint32 len);
uint32 usb_midi_tx_buffered(const uint32* buf, uint32 len);
uint32 usb_midi_rx(uint32* buf, uint32 len);
uint32 usb_midi_peek(uint32* buf, uint32 len);

uint32 usb_midi_data_available(void); /* in RX buffer */
uint16 usb_midi_get_pending(void);
uint8  usb_midi_is_transmitting(void);

#ifdef __cplusplus
}
#endif

#endif
