/*
  uhi_midi.h
  aleph-avr32

  usb host interface for midi driver

 */

#ifndef _USB_UHI_MIDI_H_
#define _USB_UHI_MIDI_H_

#include "conf_usb_host.h"
#include "usb_protocol.h"
#include "uhi.h"

//! Maximum number of simultaneously connected USB MIDI devices. Each device
//! costs 2 data pipes (bulk IN + OUT); the UC3B has 7 shared data pipes, so with
//! a grid (2 pipes) already attached 2 MIDI devices (4 pipes) is the practical
//! ceiling. See MIDI_MULTIDEVICE_PLAN.md.
#define UHI_MIDI_MAX_DEV 2

//! Sentinel returned by uhi_midi_slot_by_add() when no slot matches.
#define UHI_MIDI_NO_SLOT 0xff

//! Global define which contains standard UHI API for UHC
//! It must be added in USB_HOST_UHI define from conf_usb_host.h file.
#define UHI_MIDI { \
	.install = uhi_midi_install, \
	.enable = uhi_midi_enable, \
	.uninstall = uhi_midi_uninstall, \
	.sof_notify = NULL, \
}


// install
extern uhc_enum_status_t uhi_midi_install(uhc_device_t* dev);
// uninstall
extern void uhi_midi_uninstall(uhc_device_t* dev);
// enable
extern void uhi_midi_enable(uhc_device_t* dev);
// input transfer from device slot `idx`
extern bool uhi_midi_in_run(uint8_t idx, uint8_t * buf, iram_size_t buf_size,
		uhd_callback_trans_t callback);
// output transfer to device slot `idx`
extern bool uhi_midi_out_run(uint8_t idx, uint8_t * buf, iram_size_t buf_size,
		uhd_callback_trans_t callback);
// map a USB address (as delivered to transfer callbacks, which carry no device
// handle) back to a device slot index, or UHI_MIDI_NO_SLOT if none matches
extern uint8_t uhi_midi_slot_by_add(usb_add_t add);
// true if device slot `idx` currently holds a connected device
extern bool uhi_midi_slot_connected(uint8_t idx);
// device handle for slot `idx`, or NULL if unconnected/out of range
extern uhc_device_t* uhi_midi_slot_dev(uint8_t idx);


// get virtual cable id
/* extern u8 midi_get_cable_in(void); */
/* extern u8 midi_get_cable_out(void); */

#endif // h guard
