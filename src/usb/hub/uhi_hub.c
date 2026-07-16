/*
  uhi_hub.c

  USB host HUB class driver for the Atmel ASF UHC stack.

  Hand-ported from TinyUSB src/host/hub.c (MIT, Ha Thach). TinyUSB does not run
  on AVR32; this re-expresses its hub algorithm against the ASF UHC/UHD API in
  libavr32, so a grid + keyboard (+ a MIDI controller) can run through a powered
  hub. See USB_HUB_PORT_PLAN.md.

  KEY CONSTRAINT: ASF's control-transfer completion callback
  (uhd_callback_setup_end_t) has no user_data — and the `add` it receives is
  pipe 0's *current* configured address, which is unreliable once devices share
  the pipe. Since there is only one hub (UHI_HUB_MAX == 1), callbacks recover the
  hub via get_hub_by_addr(), which returns the single active hub regardless of
  `add`; the transfer data (ctrl_buf) is still the hub's because uhd serializes
  control transfers.
*/

#include <string.h>
#include "compiler.h" // uint8_t, le16_to_cpu, COMPILER_WORD_ALIGNED, usb base types
#include "usb_protocol.h"
#include "uhd.h"
#include "uhc.h"
#include "conf_usb_host.h"
#include "uhi_hub.h"

#ifdef USB_TOPO_DEBUG
#include "usb_dbg.h"
#endif

// Inert unless USB_HOST_HUB_SUPPORT is defined: the body references uhc_device_t
// fields (->hub, ->hub_port) that only exist under that flag, so the file can
// live in the build (config.mk) without breaking a hub-disabled compile.
#ifdef USB_HOST_HUB_SUPPORT

//--------------------------------------------------------------------+
// Per-hub state  (TinyUSB: hub_interface_t + hub_epbuf_t, hub.c:43-62)
//--------------------------------------------------------------------+

// ASF enumerates ONE device at a time (global uhc_dev_enum), so a hub services
// one port-change to completion before re-arming its status poll.
typedef enum {
	HUB_ST_IDLE = 0,
	HUB_ST_GET_PORT_STATUS,
	HUB_ST_RESETTING,
} hub_state_t;

typedef struct {
	uhc_device_t* dev;        // the hub device itself (NULL = slot free)
	usb_ep_t      ep_in;      // interrupt-IN status-change endpoint
	uint8_t       nb_ports;   // from hub descriptor bNbrPorts
	uint8_t       pwr_good_2ms;
	uint8_t       cur_port;   // port currently being serviced
	hub_state_t   state;

	// downstream enumeration hand-off (uhi_hub_send_reset handshake)
	uhd_callback_reset_t reset_cb; // enumeration continuation to invoke when done
	uint8_t reset_port;            // port currently being reset
	uint8_t reset_polls;           // GET_STATUS poll count (bounds the wait)

	// Ports holding a device we already rejected as unsupported (low-speed behind
	// a full-speed hub). Bit (port-1). Cleared when that port reports disconnect.
	// Stops the reset->reject->reset loop: uhc_hub_port_change's duplicate-connect
	// guard tears down the stale device and re-enumerates, so without this bit a
	// rejected device would be re-enumerated (and re-rejected) every round.
	uint8_t rejected;

	// Word-aligned for USBB DMA (matches COMPILER_WORD_ALIGNED in the other UHI
	// drivers). status_change: bitmap, bit0=hub, bit n=port n.
	COMPILER_WORD_ALIGNED uint8_t status_change[4];
	COMPILER_WORD_ALIGNED uint8_t ctrl_buf[16]; // hub descriptor / port status
} uhi_hub_t;

static uhi_hub_t hubs[UHI_HUB_MAX];

//--------------------------------------------------------------------+
// Downstream device speed (low-speed-behind-hub fix)
//--------------------------------------------------------------------+

// Speed the hub reported for the most-recently-reset downstream port. ASF
// enumerates one device at a time (global uhc_dev_enum), so one value suffices.
static uhd_speed_t hub_reset_speed = UHD_SPEED_FULL;

uhd_speed_t uhi_hub_get_reset_speed(void) { return hub_reset_speed; }

// forward decl: defined below, near the other get_hub_by_* helpers
static uhi_hub_t* get_hub_by_dev(uhc_device_t* dev);

// Mark `dev`'s downstream port as an unsupported low-speed device. Marking the
// port stops uhc.c from re-enumerating it in a loop (see `rejected` above); the
// bit is cleared when the device disconnects.
void uhi_hub_reject_ls(uhc_device_t* dev) {
	uhi_hub_t* hub = get_hub_by_dev(dev->hub);
	if (hub != NULL && dev->hub_port >= 1 && dev->hub_port <= UHI_HUB_MAX_PORTS) {
		hub->rejected |= (uint8_t)(1u << (dev->hub_port - 1));
	}
}

// Recover the hub for a control-transfer completion callback. uhd passes
// `uhd_get_configured_address(0)` (pipe 0's current address), which can be some
// other device's address once transfers interleave on the shared pipe. With
// UHI_HUB_MAX == 1 there is only one hub, so return it regardless of `add`; the
// transfer data (ctrl_buf) is still the hub's since uhd serializes transfers.
// (If UHI_HUB_MAX were >1 this would need a request-carried hub reference.)
static uhi_hub_t* get_hub_by_addr(usb_add_t add) {
	(void)add;
	for (uint8_t i = 0; i < UHI_HUB_MAX; i++) {
		if (hubs[i].dev != NULL) {
			return &hubs[i];
		}
	}
	return NULL;
}

static uhi_hub_t* get_hub_by_dev(uhc_device_t* dev) {
	for (uint8_t i = 0; i < UHI_HUB_MAX; i++) {
		if (hubs[i].dev == dev) return &hubs[i];
	}
	return NULL;
}

//--------------------------------------------------------------------+
// Control-request helpers
//   (TinyUSB: hub_port_set_feature / _clear_feature / _get_status, hub.c:95-200)
//--------------------------------------------------------------------+
static bool hub_ctrl(usb_add_t add, uint8_t reqtype, uint8_t req,
                     uint16_t value, uint16_t index, uint8_t* buf, uint16_t len,
                     uhd_callback_setup_end_t end_cb) {
	usb_setup_req_t r;
	r.bmRequestType = reqtype;
	r.bRequest      = req;
	r.wValue        = value;
	r.wIndex        = index;
	r.wLength       = len;
	return uhd_setup_request(add, &r, buf, len, NULL, end_cb);
}

// SET_FEATURE(PORT_POWER) etc. on a port (TinyUSB hub_port_set_feature)
static bool port_set_feature(usb_add_t add, uint8_t port, uint8_t feat,
                             uhd_callback_setup_end_t cb) {
	return hub_ctrl(add, HUB_REQTYPE_PORT_OUT, HUB_REQ_SET_FEATURE,
	                feat, port, NULL, 0, cb);
}

// CLEAR_FEATURE(C_PORT_*) on a port (TinyUSB hub_port_clear_feature)
static bool port_clear_feature(usb_add_t add, uint8_t port, uint8_t feat,
                               uhd_callback_setup_end_t cb) {
	return hub_ctrl(add, HUB_REQTYPE_PORT_OUT, HUB_REQ_CLEAR_FEATURE,
	                feat, port, NULL, 0, cb);
}

// GET_STATUS(port) -> 4 bytes into hub->ctrl_buf (TinyUSB hub_port_get_status)
static bool port_get_status(uhi_hub_t* hub, uint8_t port,
                            uhd_callback_setup_end_t cb) {
	return hub_ctrl(hub->dev->address, HUB_REQTYPE_PORT_IN, HUB_REQ_GET_STATUS,
	                0, port, hub->ctrl_buf, 4, cb);
}

//--------------------------------------------------------------------+
// Forward decls for the async chain
//--------------------------------------------------------------------+
static void hub_start_status_poll(uhi_hub_t* hub);
static void on_status_change(usb_add_t add, usb_ep_t ep,
                             uhd_trans_status_t status, iram_size_t n);
static void on_port_status(usb_add_t add, uhd_trans_status_t status, uint16_t n);
static void on_conn_change_cleared(usb_add_t add, uhd_trans_status_t status,
                                    uint16_t n);
static void on_hubdesc(usb_add_t add, uhd_trans_status_t status, uint16_t n);
static void on_port_power(usb_add_t add, uhd_trans_status_t status, uint16_t n);
static void on_other_change_cleared(usb_add_t add, uhd_trans_status_t status, uint16_t n);
static void on_reset_feature_set(usb_add_t add, uhd_trans_status_t status, uint16_t n);
static void on_reset_poll(usb_add_t add, uhd_trans_status_t status, uint16_t n);
static void on_reset_cleared(usb_add_t add, uhd_trans_status_t status, uint16_t n);

//--------------------------------------------------------------------+
// UHI: install  (TinyUSB: hub_open, hub.c:221-242)
//--------------------------------------------------------------------+
uhc_enum_status_t uhi_hub_install(uhc_device_t* dev) {
	uint16_t conf_lgt = le16_to_cpu(dev->conf_desc->wTotalLength);
	usb_iface_desc_t* iface = (usb_iface_desc_t*)dev->conf_desc;
	bool supported = false;

	// Do NOT claim a hub slot until a HUB-class interface is confirmed. uhc.c
	// calls every UHI install() on every device; returning anything other than
	// SUCCESS/UNSUPPORTED for a non-hub device would abort that device's
	// enumeration. Non-hub devices must fall through to UNSUPPORTED.
	while (conf_lgt) {
		switch (iface->bDescriptorType) {
		case USB_DT_INTERFACE:
			supported = (iface->bInterfaceClass == HUB_CLASS);
			break;
		case USB_DT_ENDPOINT:
			if (!supported) break;
			{
				// Confirmed hub-class interface: now (and only now) claim a slot.
				uhi_hub_t* hub = get_hub_by_dev(NULL);
				if (hub == NULL) return UHC_ENUM_SOFTWARE_LIMIT;
				if (!uhd_ep_alloc(dev->address, (usb_ep_desc_t*)iface)) {
					return UHC_ENUM_HARDWARE_LIMIT;
				}
				// status-change endpoint is interrupt IN
				hub->ep_in = ((usb_ep_desc_t*)iface)->bEndpointAddress;
				memset(hub->status_change, 0, sizeof(hub->status_change));
				hub->dev   = dev;
				hub->state = HUB_ST_IDLE;
				return UHC_ENUM_SUCCESS;
			}
		default:
			break;
		}
		conf_lgt -= iface->bLength;
		iface = (usb_iface_desc_t*)((uint8_t*)iface + iface->bLength);
	}
	return UHC_ENUM_UNSUPPORTED;
}

//--------------------------------------------------------------------+
// UHI: enable  (TinyUSB: hub_set_config -> config_set_port_power, hub.c:273-342)
//   Fetch hub descriptor, then power each port, then arm the status poll.
//--------------------------------------------------------------------+
void uhi_hub_enable(uhc_device_t* dev) {
	uhi_hub_t* hub = get_hub_by_dev(dev);
	if (hub == NULL) return;

	// GET_DESCRIPTOR (hub class) -> ctrl_buf. wValue=0 per TinyUSB hub_set_config.
	hub_ctrl(dev->address, HUB_REQTYPE_DEV_IN, HUB_REQ_GET_DESCRIPTOR,
	         0, 0, hub->ctrl_buf, 9 /*sizeof hub_desc_cs_t*/, on_hubdesc);
}

// hub descriptor arrived (TinyUSB config_set_port_power, hub.c:304-321)
static void on_hubdesc(usb_add_t add, uhd_trans_status_t status, uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL || status != UHD_TRANS_NOERROR) return;

	// hub_desc_cs_t layout: [2]=bNbrPorts, [5]=bPwrOn2PwrGood
	hub->nb_ports     = hub->ctrl_buf[2];
	hub->pwr_good_2ms = hub->ctrl_buf[5];
	if (hub->nb_ports > UHI_HUB_MAX_PORTS) hub->nb_ports = UHI_HUB_MAX_PORTS;

	// Power port 1; on_port_power walks the rest (TinyUSB config_port_power_*).
	hub->cur_port = 1;
	port_set_feature(add, 1, HUB_FEAT_PORT_POWER, on_port_power);
}

// each PORT_POWER set completes -> power next, or finish (hub.c:323-342)
static void on_port_power(usb_add_t add, uhd_trans_status_t status, uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL || status != UHD_TRANS_NOERROR) return;

	if (hub->cur_port >= hub->nb_ports) {
		// All ports powered. TODO: honour pwr_good_2ms delay (2ms * value)
		// before trusting connection status — schedule via a SOF/timer.
		hub_start_status_poll(hub);
	} else {
		hub->cur_port++;
		port_set_feature(add, hub->cur_port, HUB_FEAT_PORT_POWER, on_port_power);
	}
}

//--------------------------------------------------------------------+
// Status-change interrupt poll  (TinyUSB: hub_edpt_status_xfer + hub_xfer_cb,
//                                hub.c:254-395)
//--------------------------------------------------------------------+
static void hub_start_status_poll(uhi_hub_t* hub) {
	// interrupt IN, 1 byte enough for <=7 ports (bit0 hub + bits 1..n ports)
#ifdef USB_TOPO_DEBUG
	// "poll!" once per failure streak: the status poll could not be re-armed
	// (uhd request slots exhausted / pipe gone) — hub servicing is now dead.
	static bool poll_run_failed;
	bool ok = uhd_ep_run(hub->dev->address, hub->ep_in, true,
	                     hub->status_change, 1, 0, on_status_change);
	if (!ok && !poll_run_failed) usb_dbg_push("poll!");
	poll_run_failed = !ok;
#else
	uhd_ep_run(hub->dev->address, hub->ep_in, true,
	           hub->status_change, 1, 0, on_status_change);
#endif
}

// interrupt transfer completed -> decode which port changed (hub_xfer_cb)
static void on_status_change(usb_add_t add, usb_ep_t ep,
                             uhd_trans_status_t status, iram_size_t n) {
	(void)ep;
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;

#ifdef USB_TOPO_DEBUG
	// Log only the FIRST error of a streak — if the error completes
	// synchronously, the re-arm below loops at IRQ rate and per-error pushes
	// would flood the ring. "sc t <status>" then silence + frozen module =
	// tight error/re-arm loop with that status. On recovery, "sc ok <count>"
	// reports how many errors the streak had.
	static uint16_t sc_err_streak;
	if (status != UHD_TRANS_NOERROR) {
		if (sc_err_streak == 0) usb_dbg_log_val("sc t", status);
		if (sc_err_streak < 0xFFFF) sc_err_streak++;
	}
	else if (sc_err_streak != 0) {
		usb_dbg_log_val("sc ok", sc_err_streak);
		sc_err_streak = 0;
	}
#endif
	if (status != UHD_TRANS_NOERROR) {
		hub_start_status_poll(hub); // re-arm and bail
		return;
	}

	uint8_t change = hub->status_change[0];
	if (change == 0) {
		hub_start_status_poll(hub); // spurious; re-arm (TinyUSB processed=false)
		return;
	}

	// bit0 = hub-level change (over-current / power). TODO handle if needed.
	// bits 1..n = port change. Service the first set port; re-poll after.
	for (uint8_t port = 1; port <= hub->nb_ports; port++) {
		if (change & (1u << port)) {
			hub->cur_port = port;
			hub->state    = HUB_ST_GET_PORT_STATUS;
			port_get_status(hub, port, on_port_status);
			return; // one port per pass (TinyUSB breaks here)
		}
	}
	hub_start_status_poll(hub);
}

// GET_STATUS(port) arrived (TinyUSB process_new_status STATE_CLEAR_CHANGE,
//                           hub.c:426-449)
static void on_port_status(usb_add_t add, uhd_trans_status_t status, uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	if (status != UHD_TRANS_NOERROR) {
		hub_start_status_poll(hub); // re-arm rather than dead-stall
		return;
	}

	// port status response: [0..1]=status bits, [2..3]=change bits.
	// change word low byte (ctrl_buf[2]): connection[0] enable[1] suspend[2]
	// over_current[3] reset[4]. EVERY set change bit must be cleared or the hub
	// re-asserts the status-change interrupt forever.
	uint8_t chg = hub->ctrl_buf[2];

	if (chg & 0x01) {
		// Connection change: ack, then attach/detach in on_conn_change_cleared.
		port_clear_feature(add, hub->cur_port, HUB_FEAT_PORT_CONNECTION_CHANGE,
		                   on_conn_change_cleared);
	} else if (chg & 0x02) {
		port_clear_feature(add, hub->cur_port, HUB_FEAT_PORT_ENABLE_CHANGE,
		                   on_other_change_cleared);
	} else if (chg & 0x04) {
		port_clear_feature(add, hub->cur_port, HUB_FEAT_PORT_SUSPEND_CHANGE,
		                   on_other_change_cleared);
	} else if (chg & 0x08) {
		port_clear_feature(add, hub->cur_port, HUB_FEAT_PORT_OVER_CURRENT_CHANGE,
		                   on_other_change_cleared);
	} else if (chg & 0x10) {
		port_clear_feature(add, hub->cur_port, HUB_FEAT_PORT_RESET_CHANGE,
		                   on_other_change_cleared);
	} else {
		// No change bits set for this port; just look for the next change.
		hub_start_status_poll(hub);
	}
}

// A non-connection port change was acked -> re-poll to drain the next change
// (the hub reports one bit at a time; this loop empties the pending set).
static void on_other_change_cleared(usb_add_t add, uhd_trans_status_t status,
                                    uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	hub_start_status_poll(hub);
}

// connection-change acked -> attach or detach downstream device
//   (TinyUSB process_new_status STATE_CHECK_CONN, hub.c:451-464; the
//    HCD_EVENT_DEVICE_ATTACH there becomes a uhc_hub_port_change() call here)
static void on_conn_change_cleared(usb_add_t add, uhd_trans_status_t status,
                                   uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL || status != UHD_TRANS_NOERROR) return;

	bool connected = hub->ctrl_buf[0] & 0x01; // current status.connection
	uint8_t port_bit = (hub->cur_port >= 1 && hub->cur_port <= UHI_HUB_MAX_PORTS)
	                       ? (uint8_t)(1u << (hub->cur_port - 1))
	                       : 0;

	if (connected && (hub->rejected & port_bit)) {
		// This port holds a device we already rejected (unsupported low-speed
		// device behind a full-speed hub). Re-enumerating it just resets ->
		// rejects -> resets forever and leaks a uhc_device_t each round, so
		// leave it alone. A reset can re-assert the connection-change bit, which
		// is exactly what used to drive that loop. Ignore until it disconnects.
		hub_start_status_poll(hub);
		return;
	}
	if (!connected) {
		// Device gone: allow this port to enumerate again next time.
		hub->rejected &= (uint8_t)~port_bit;
	}

	// Hand off to the UHC entry point (uhc.c), which owns the device list and
	// enumeration. On connect it allocates + links a uhc_device_t and enumerates
	// it (calling back into uhi_hub_send_reset() for the port reset); on
	// disconnect it finds the device on (hub, port) and tears it down.
	uhc_hub_port_change(hub->dev, hub->cur_port, connected);

	// Re-arm the status poll ONLY on disconnect. On CONNECT we must NOT re-arm:
	// the downstream device now enumerates via control transfers on the SHARED
	// pipe 0, and a concurrent hub GET_STATUS poll would race that pipe and
	// corrupt the enumeration. The poll resumes from usb_enum (uhi_hub_poll_resume)
	// once the device finishes enumerating and pipe 0 is free again.
	if (!connected) {
		hub_start_status_poll(hub);
	}
}

//--------------------------------------------------------------------+
// Called BY uhc.c while enumerating a device behind this hub (uhc.c ~229/249)
//--------------------------------------------------------------------+

// Reset the hub port `dev` is attached to, then signal completion.
// (TinyUSB hub_port_reset = SET_FEATURE(PORT_RESET); hub.h:184)
//
// UHC's enumeration cannot proceed until `callback` fires:
//   SET_FEATURE(PORT_RESET) -> poll GET_STATUS(port) until C_PORT_RESET ->
//   CLEAR_FEATURE(C_PORT_RESET) -> callback(). The GET_STATUS polling is
//   self-clocking: each control transfer takes ~1ms and a hub completes reset in
//   ~10-20ms, so a handful of polls covers it without an explicit timer.
void uhi_hub_send_reset(uhc_device_t* dev, uhd_callback_reset_t callback) {
	uhi_hub_t* hub = get_hub_by_dev(dev->hub);
	if (hub == NULL) { if (callback) callback(); return; }

	hub->reset_cb    = callback;
	hub->reset_port  = dev->hub_port;
	hub->reset_polls = 0;
	hub->state       = HUB_ST_RESETTING;
	port_set_feature(hub->dev->address, hub->reset_port, HUB_FEAT_PORT_RESET,
	                 on_reset_feature_set);
}

// PORT_RESET asserted -> start polling for reset completion.
static void on_reset_feature_set(usb_add_t add, uhd_trans_status_t status,
                                 uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	if (status != UHD_TRANS_NOERROR) { if (hub->reset_cb) hub->reset_cb(); return; }
	port_get_status(hub, hub->reset_port, on_reset_poll);
}

// GET_STATUS(port) during reset -> done when change.reset (bit 4) is set.
static void on_reset_poll(usb_add_t add, uhd_trans_status_t status, uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	if (status != UHD_TRANS_NOERROR) { if (hub->reset_cb) hub->reset_cb(); return; }

	// port status response: change word is bytes [2..3]; reset = bit 4 -> 0x10.
	if (hub->ctrl_buf[2] & 0x10) {
		// Reset complete; port is enabled. The hub reports the attached device's
		// speed in the STATUS word (bytes [0..1]): bit 9 = low-speed, bit 10 =
		// high-speed, neither = full-speed. Capture it so uhc.c can set the
		// device's speed from what the hub saw, instead of uhd_get_speed() (the
		// hub's own full-speed link). Valid now: the port is enabled post-reset.
		uint16_t pstat = hub->ctrl_buf[0] | ((uint16_t)hub->ctrl_buf[1] << 8);
		if (pstat & (1u << 9))       hub_reset_speed = UHD_SPEED_LOW;
		else if (pstat & (1u << 10)) hub_reset_speed = UHD_SPEED_HIGH;
		else                         hub_reset_speed = UHD_SPEED_FULL;
		// Ack the change, then resume enum.
		port_clear_feature(hub->dev->address, hub->reset_port,
		                   HUB_FEAT_PORT_RESET_CHANGE, on_reset_cleared);
	} else if (++hub->reset_polls < 50) {
		port_get_status(hub, hub->reset_port, on_reset_poll); // not done; poll again
	} else {
		// Timed out waiting for reset; resume so enumeration fails cleanly.
#ifdef USB_TOPO_DEBUG
		usb_dbg_log_val("rstTO p", hub->reset_port);
#endif
		if (hub->reset_cb) hub->reset_cb();
	}
}

// C_PORT_RESET cleared -> hand control back to UHC's enumeration state machine.
static void on_reset_cleared(usb_add_t add, uhd_trans_status_t status,
                             uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	uhd_callback_reset_t cb = hub->reset_cb;
	hub->reset_cb = NULL;
	hub->state    = HUB_ST_IDLE;
	if (cb) cb(); // -> uhc_enumeration_stepN (GET_DESCRIPTOR / SET_ADDRESS ...)
}

// Suspend the hub port `dev` is on. (TinyUSB: SET_FEATURE PORT_SUSPEND)
void uhi_hub_suspend(uhc_device_t* dev) {
	uhi_hub_t* hub = get_hub_by_dev(dev->hub);
	if (hub == NULL) return;
	port_set_feature(hub->dev->address, dev->hub_port, HUB_FEAT_PORT_SUSPEND, NULL);
}

//--------------------------------------------------------------------+
// UHI: uninstall  (TinyUSB: hub_close, hub.c:244-252)
//--------------------------------------------------------------------+
void uhi_hub_uninstall(uhc_device_t* dev) {
	uhi_hub_t* hub = get_hub_by_dev(dev);
	if (hub == NULL) return;
	// TODO: tear down any still-attached downstream devices on this hub first
	// (uhc_hub_port_change(hub, port, false)) to avoid orphaned devices.
	memset(hub, 0, sizeof(*hub)); // frees the slot (dev = NULL)
}

// Resume the downstream status poll after a device behind `hub_dev` finished
// enumerating (called from usb_enum). pipe 0 is now free, so it's safe to poll
// again — this is what lets a SECOND device (e.g. a keyboard after the grid) be
// detected. Serializes cleanly: enumerate -> resume -> detect next.
void uhi_hub_poll_resume(uhc_device_t* hub_dev) {
	uhi_hub_t* hub = get_hub_by_dev(hub_dev);
	if (hub != NULL) {
		hub_start_status_poll(hub);
	}
}

#endif // USB_HOST_HUB_SUPPORT
