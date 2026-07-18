#include "conf_board.h"
#include "compiler.h"
#include "gpio.h"
#include "print_funcs.h"
#include "conf_usb_host.h" // USB_HOST_HUB_SUPPORT + hub-aware uhc_device_t layout
#include "usb.h"


/*
///// TODO! 
most of these things don't really need to be implemented
the ASF usb host stack needs them to be defined though.

we might be able to use vbus_error for overcurrent notifications.


   */

// usb mode change callback
void usb_mode_change(bool b_host_mode) {
  // print_dbg("\r\n mode change (ignore) ");
}
 
// usb Vbus change callback
void usb_vbus_change(bool b_vbus_present) {
  // print_dbg("\r\n usb vbus change, new status: ");
  // print_dbg_ulong(b_vbus_present);
}

// usb vbus error callback
void usb_vbus_error(void) {
  // print_dbg("\r\n ******************* usb vbus error");

}

volatile uint8_t usb_enumeration_active;

// usb connection callback
void usb_connection(uhc_device_t *dev, bool b_present) {
    // Enumeration of this device starts now (uhc_connection_tree). The flag
    // clears in usb_enum() when enumeration finishes (success or give-up).
    if (b_present) usb_enumeration_active = 1;
}

// usb wakeup callback
void usb_wakeup(void) {
    // print_dbg("\r\n usb wakeup");
}

// usb start-of-frame callback
void usb_sof(void) {
     // print_dbg("\r\n usb sof");
}

// usb end-of-enumeration callback
void usb_enum(uhc_device_t *dev, uhc_enum_status_t status) {
  usb_enumeration_active = 0;
#ifdef USB_HOST_HUB_SUPPORT
  // A device behind a hub just finished enumerating (success or final failure).
  // pipe 0 is free again -> resume that hub's status poll so the NEXT device
  // (e.g. a keyboard plugged after the grid) gets detected.
  if (dev->hub != NULL) {
    uhi_hub_poll_resume(dev->hub);
  }
#endif
}
