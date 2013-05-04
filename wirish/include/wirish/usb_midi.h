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
 * @brief Wirish USB MIDI port (MidiUSB).
 */

#ifndef _WIRISH_USB_MIDI_H_
#define _WIRISH_USB_MIDI_H_

#include <wirish/Print.h>
#include <wirish/boards.h>

#define BOARD_HAVE_MIDIUSB (BOARD_HAVE_USB &&        \
                         defined(USB_TYPE) &&        \
                       (USB_TYPE == USB_MIDI))

/**
 * @brief Virtual serial terminal.
 */
class USBMidi : public Print {
public:
    USBMidi(void);

    void begin(void);
    void end(void);

    uint32 available(void);

    uint32 read(void *buf, uint32 len);
    uint8  read(void);

    void write(uint8);
    void write(const char *str);
    void write(const void*, uint32);

    uint8 isConnected();
    uint8 pending();
};

#if BOARD_HAVE_MIDIUSB
extern USBMidi MidiUSB;
#endif

#endif

