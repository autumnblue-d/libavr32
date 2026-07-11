#ifndef _USB_UHI_HUB_H_
#define _USB_UHI_HUB_H_

// USB host hub (UHI) driver for the ASF UHC stack.
//
// SKELETON — derived by hand-porting the algorithm in TinyUSB
// (src/host/hub.c, src/host/hub.h) onto the Atmel ASF UHC/UHD API.
// TinyUSB cannot be linked on AVR32; this re-expresses its logic against
// uhd_setup_request()/uhd_ep_run()/uhc_connection_tree().
//
// Registration: add UHI_HUB to USB_HOST_UHI in conf_usb_host.h, and add this
// .c file to module/config.mk CSRCS. See USB_HUB_PORT_PLAN.md Phase 1.

#include "conf_usb_host.h"
#include "usb_protocol.h"
#include "uhc.h"
#include "uhd.h"

// Max simultaneous hubs and max downstream ports we will service. Keep small —
// the AVR32 USBB has only ~7 pipes total (one shared control pipe). One hub
// with a few ports is the realistic target. See plan "Hardware ceiling".
#ifndef UHI_HUB_MAX
#  define UHI_HUB_MAX 1
#endif
#ifndef UHI_HUB_MAX_PORTS
#  define UHI_HUB_MAX_PORTS 4
#endif

// USB hub class code (bInterfaceClass). Mirrors TUSB_CLASS_HUB.
#define HUB_CLASS 0x09

// Hub class requests (hub.h: HUB_REQUEST_*)
#define HUB_REQ_GET_STATUS     0
#define HUB_REQ_CLEAR_FEATURE  1
#define HUB_REQ_SET_FEATURE    3
#define HUB_REQ_GET_DESCRIPTOR 6

// Port features (hub.h: HUB_FEATURE_PORT_*)
#define HUB_FEAT_PORT_RESET             4
#define HUB_FEAT_PORT_POWER             8
#define HUB_FEAT_PORT_SUSPEND           2
#define HUB_FEAT_PORT_CONNECTION_CHANGE   16
#define HUB_FEAT_PORT_ENABLE_CHANGE       17
#define HUB_FEAT_PORT_SUSPEND_CHANGE      18
#define HUB_FEAT_PORT_OVER_CURRENT_CHANGE 19
#define HUB_FEAT_PORT_RESET_CHANGE        20

// bmRequestType values (recipient/type/dir) for hub vs port requests.
//   D7 dir (1=IN), D6..5 type (01=class), D4..0 recipient (0=device,3=other)
#define HUB_REQTYPE_DEV_IN   0xA0
#define HUB_REQTYPE_PORT_IN  0xA3
#define HUB_REQTYPE_PORT_OUT 0x23

//! Standard UHI API entry, added to USB_HOST_UHI in conf_usb_host.h
#define UHI_HUB { \
	.install    = uhi_hub_install, \
	.enable     = uhi_hub_enable, \
	.uninstall  = uhi_hub_uninstall, \
	.sof_notify = NULL, \
}

extern uhc_enum_status_t uhi_hub_install(uhc_device_t* dev);
extern void uhi_hub_enable(uhc_device_t* dev);
extern void uhi_hub_uninstall(uhc_device_t* dev);

// Called by uhc.c during enumeration of a device that sits BEHIND a hub
// (uhc.c references these inside #ifdef USB_HOST_HUB_SUPPORT, ~lines 229/249).
// They must drive the hub's downstream port, then invoke the UHC callback.
extern void uhi_hub_send_reset(uhc_device_t* dev, uhd_callback_reset_t callback);
extern void uhi_hub_suspend(uhc_device_t* dev);

// Resume a hub's downstream status poll after a device behind it has finished
// enumerating (called from usb_enum). The poll is paused during a downstream
// enumeration so its GET_STATUS control transfers don't race the enumeration on
// the shared pipe 0; resuming here lets the next device be detected.
extern void uhi_hub_poll_resume(uhc_device_t* hub_dev);

#endif // _USB_UHI_HUB_H_
