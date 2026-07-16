#ifndef _USB_H_
#define _USB_H_

#include "compiler.h"
#include "uhc.h"

// usb mode change callback
void usb_mode_change(bool b_host_mode);
 
// usb Vbus change callback
void usb_vbus_change(bool b_vbus_present);

// usb vbus error callback
void usb_vbus_error(void);

// usb connection callback
// Nonzero while a USB device enumeration is in progress (set at the connect
// report, cleared by usb_enum). Bulk traffic to already-connected devices does
// not reliably survive a concurrent enumeration on this host (a pending grid
// bulk-IN during a heavy composite enumeration kills the grid's init), so the
// module keeps monome traffic off the bus while this is set.
extern volatile uint8_t usb_enumeration_active;

void usb_connection(uhc_device_t *dev, bool b_present);

// usb wakeup callback
void usb_wakeup(void);

// usb start-of-frame callback
void usb_sof(void);

// usb end-of-enumeration callback
void usb_enum(uhc_device_t *dev, uhc_enum_status_t status);


#endif // header guard
