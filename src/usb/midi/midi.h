#ifndef _USB_MIDI_H_
#define _USB_MIDI_H_

#include "types.h"
#include "uhc.h"




// read and spawn events (non-blocking)
extern void midi_read(void);

// write to MIDI device `device` (slot index; see uhi_midi.h UHI_MIDI_MAX_DEV)
extern bool midi_write(u8 device, const u8* data, u32 bytes);
extern void midi_write_packet(u8 device, u8 cable_number, u8 *pack);

// MIDI device was plugged or unplugged
extern void midi_change(uhc_device_t* dev, u8 plug);

// device state (slot index; see uhi_midi.h UHI_MIDI_MAX_DEV)
extern bool midi_dev_connected(u8 dev);
extern const char* midi_dev_name(u8 dev);

// fetch/cache the USB product-name string for a connected device slot. Does a
// blocking control transfer -- call only from main-loop context, never from
// midi_change() or a UHC enumeration callback (see midi.c for why).
extern void midi_dev_fetch_name(u8 dev);


#endif
