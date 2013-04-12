/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
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
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
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
 * @brief USB MIDI device
 */

#include <wirish/usb_midi.h>

#include <string.h>
#include <stdint.h>

#include <libmaple/nvic.h>
#include <libmaple/usb_midi_device.h>
#include <libmaple/usb.h>

#include <wirish/wirish.h>

/*
 * USBMidi interface
 */

#define USB_TIMEOUT 50

USBMidi::USBMidi(void) {
#if !BOARD_HAVE_MIDIUSB
    ASSERT(0);
#endif
}

void USBMidi::begin(void) {
#if BOARD_HAVE_MIDIUSB
    usb_midi_enable(BOARD_USB_DISC_DEV, BOARD_USB_DISC_BIT);
#endif
}

void USBMidi::end(void) {
#if BOARD_HAVE_MIDIUSB
    usb_midi_disable(BOARD_USB_DISC_DEV, BOARD_USB_DISC_BIT);
#endif
}

void USBMidi::write(uint8 ch) {
    this->write(&ch, 1);
}

void USBMidi::write(const char *str) {
    this->write(str, strlen(str));
}

void USBMidi::write(const void *buf, uint32 len) {
    if (!this->isConnected() || !buf) {
        return;
    }

    uint32 txed = 0;
    uint32 old_txed = 0;
    uint32 start = millis();

    uint32 sent = 0;

    while (txed < len && (millis() - start < USB_TIMEOUT)) {
//        sent = usb_midi_tx_buffered((const uint8*)buf + txed, len - txed);
        sent = usb_midi_tx((const uint8*)buf + txed, len - txed);
        txed += sent;
        if (old_txed != txed) {
            start = millis();
        }
        old_txed = txed;
    }


    if (sent == USB_MIDI_TX_EPSIZE) {
        while (usb_midi_is_transmitting() != 0) {
        }
        /* flush out to avoid having the pc wait for more data */
        usb_midi_tx(NULL, 0);
    }
}

uint32 USBMidi::available(void) {
    return usb_midi_data_available();
}

uint32 USBMidi::read(void *buf, uint32 len) {
    if (!buf) {
        return 0;
    }

    uint32 rxed = 0;
    while (rxed < len) {
        rxed += usb_midi_rx((uint8*)buf + rxed, len - rxed);
    }

    return rxed;
}

/* Blocks forever until 1 byte is received */
uint8 USBMidi::read(void) {
    uint8 b;
    this->read(&b, 1);
    return b;
}

uint8 USBMidi::pending(void) {
    return usb_midi_get_pending();
}

uint8 USBMidi::isConnected(void) {
    return usb_is_connected(USBLIB) && usb_is_configured(USBLIB);
}

#if BOARD_HAVE_MIDIUSB
USBMidi MidiUSB;
#endif
