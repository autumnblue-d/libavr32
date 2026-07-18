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

// Deferred actions run from the 1 ms SOF tick (uhi_hub_sof). This is how the
// driver waits (bPwrOn2PwrGood settle) and retries transient failures (a full
// uhd request FIFO, a flaky transfer) instead of dead-stalling forever.
typedef enum {
	HUB_SOF_NONE = 0,
	HUB_SOF_START_POLL,   // (re)arm the status-change interrupt poll
	HUB_SOF_RETRY_ENABLE, // re-run the enable sequence (descriptor + power)
} hub_sof_action_t;

typedef struct {
	uhc_device_t* dev;        // the hub device itself (NULL = slot free)
	usb_ep_t      ep_in;      // interrupt-IN status-change endpoint
	uint8_t       nb_ports;   // watched ports: min(bNbrPorts, UHI_HUB_MAX_PORTS)
	uint8_t       pwr_ports;  // bNbrPorts as reported: ALL of them get PORT_POWER
	uint8_t       pwr_good_2ms;
	uint8_t       cur_port;   // port currently being serviced
	hub_state_t   state;

	uint8_t       poll_armed;     // a status-change transfer is pending
	uint8_t       enable_retries; // bounded retry budget for the enable sequence

	// SOF-driven deferred action (see hub_sof_action_t / uhi_hub_sof)
	uint16_t      sof_delay;  // countdown in ms; 0 = idle
	uint8_t       sof_action; // hub_sof_action_t

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

// CAUTION: get_hub_by_dev(NULL) returns a FREE slot — uhi_hub_install relies
// on this to allocate. Never call it with a possibly-NULL device pointer
// expecting a "not found" result.
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
static void on_hub_status(usb_add_t add, uhd_trans_status_t status, uint16_t n);
static void on_hub_oc_cleared(usb_add_t add, uhd_trans_status_t status, uint16_t n);
static void hub_enable_start(uhi_hub_t* hub);
static void hub_enable_retry(uhi_hub_t* hub);

//--------------------------------------------------------------------+
// SOF deferred-action scheduling (executed by uhi_hub_sof, bottom of file)
//--------------------------------------------------------------------+

// Schedule `action` to run `ms` milliseconds from now (from uhi_hub_sof).
// One slot per hub: a later schedule replaces an earlier pending one.
static void hub_sof_schedule(uhi_hub_t* hub, uint8_t action, uint16_t ms) {
	hub->sof_action = action;
	hub->sof_delay  = ms ? ms : 1;
}

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
		if (iface->bLength == 0 || iface->bLength > conf_lgt) {
			// Malformed descriptor: a zero bLength would loop here forever,
			// an oversized one would underflow conf_lgt into a wild walk.
			return UHC_ENUM_UNSUPPORTED;
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
	hub->enable_retries = 0;
	hub_enable_start(hub);
}

// GET_DESCRIPTOR (hub class) -> ctrl_buf. wValue carries the descriptor type
// in the high byte (USB 2.0 11.24.2.5) — some hubs STALL a zero wValue.
static void hub_enable_start(uhi_hub_t* hub) {
	if (!hub_ctrl(hub->dev->address, HUB_REQTYPE_DEV_IN, HUB_REQ_GET_DESCRIPTOR,
	              (uint16_t)(HUB_DT_HUB << 8), 0, hub->ctrl_buf,
	              9 /*sizeof hub_desc_cs_t*/, on_hubdesc)) {
		hub_enable_retry(hub);
	}
}

// Transient failure in the enable sequence: retry the whole sequence a few
// times from the SOF tick. After the budget, give up — the hub stays
// unpolled (the pre-existing behavior, but no longer on the first hiccup).
static void hub_enable_retry(uhi_hub_t* hub) {
	if (++hub->enable_retries <= 3) {
		hub_sof_schedule(hub, HUB_SOF_RETRY_ENABLE, 10);
	}
}

// hub descriptor arrived (TinyUSB config_set_port_power, hub.c:304-321)
static void on_hubdesc(usb_add_t add, uhd_trans_status_t status, uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	if (status != UHD_TRANS_NOERROR || n < 6) {
		hub_enable_retry(hub);
		return;
	}

	// hub_desc_cs_t layout: [1]=bDescriptorType, [2]=bNbrPorts,
	// [5]=bPwrOn2PwrGood. A wrong type or zero ports means the hub answered
	// with garbage — retry rather than driving a nonsensical port walk.
	if (hub->ctrl_buf[1] != HUB_DT_HUB || hub->ctrl_buf[2] == 0) {
		hub_enable_retry(hub);
		return;
	}
	hub->pwr_good_2ms = hub->ctrl_buf[5];
	hub->pwr_ports    = hub->ctrl_buf[2]; // power ALL reported ports...
	hub->nb_ports     = hub->pwr_ports;   // ...but watch at most 7 (bitmap)
	if (hub->nb_ports > UHI_HUB_MAX_PORTS) hub->nb_ports = UHI_HUB_MAX_PORTS;

	// Power port 1; on_port_power walks the rest (TinyUSB config_port_power_*).
	hub->cur_port = 1;
	if (!port_set_feature(add, 1, HUB_FEAT_PORT_POWER, on_port_power)) {
		hub_enable_retry(hub);
	}
}

// each PORT_POWER set completes -> power next, or finish (hub.c:323-342)
static void on_port_power(usb_add_t add, uhd_trans_status_t status, uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	if (status != UHD_TRANS_NOERROR) {
		hub_enable_retry(hub);
		return;
	}

	if (hub->cur_port >= hub->pwr_ports) {
		// All ports powered. Honour bPwrOn2PwrGood (2 ms units, +2 ms margin)
		// before trusting connection status / arming the change poll.
		hub_sof_schedule(hub, HUB_SOF_START_POLL,
		                 (uint16_t)hub->pwr_good_2ms * 2 + 2);
	} else {
		hub->cur_port++;
		if (!port_set_feature(add, hub->cur_port, HUB_FEAT_PORT_POWER,
		                      on_port_power)) {
			hub_enable_retry(hub);
		}
	}
}

//--------------------------------------------------------------------+
// Status-change interrupt poll  (TinyUSB: hub_edpt_status_xfer + hub_xfer_cb,
//                                hub.c:254-395)
//--------------------------------------------------------------------+
static void hub_start_status_poll(uhi_hub_t* hub) {
	// interrupt IN, 1 byte enough for <=7 ports (bit0 hub + bits 1..n ports)
	if (hub->poll_armed) {
		return; // already pending; double-arming would just be refused
	}
	if (uhd_ep_run(hub->dev->address, hub->ep_in, true,
	               hub->status_change, 1, 0, on_status_change)) {
		hub->poll_armed = 1;
	}
	else {
		// uhd couldn't take the transfer (request slots busy): retry from the
		// SOF tick instead of dropping hub servicing forever.
		hub_sof_schedule(hub, HUB_SOF_START_POLL, 2);
	}
}

// interrupt transfer completed -> decode which port changed (hub_xfer_cb)
static void on_status_change(usb_add_t add, usb_ep_t ep,
                             uhd_trans_status_t status, iram_size_t n) {
	(void)ep;
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	hub->poll_armed = 0;

	if (status != UHD_TRANS_NOERROR) {
		hub_start_status_poll(hub); // re-arm and bail
		return;
	}

	uint8_t change = hub->status_change[0];
	if (n == 0 || change == 0) {
		// n == 0: zero-length completion, status_change[0] is stale — do not
		// reprocess old bits. change == 0: spurious (TinyUSB processed=false).
		hub_start_status_poll(hub);
		return;
	}

	// bit0 = hub-level change (local power / over-current): fetch hub status,
	// clear the change bit, and re-power ports after an over-current trip —
	// a latched hub change would otherwise re-assert the interrupt forever.
	if (change & 0x01) {
		if (!hub_ctrl(hub->dev->address, HUB_REQTYPE_DEV_IN, HUB_REQ_GET_STATUS,
		              0, 0, hub->ctrl_buf, 4, on_hub_status)) {
			hub_sof_schedule(hub, HUB_SOF_START_POLL, 2);
		}
		return;
	}
	// bits 1..n = port change. Service the first set port; re-poll after.
	for (uint8_t port = 1; port <= hub->nb_ports; port++) {
		if (change & (1u << port)) {
			hub->cur_port = port;
			hub->state    = HUB_ST_GET_PORT_STATUS;
			if (!port_get_status(hub, port, on_port_status)) {
				// change stays latched; the retried poll will re-report it
				hub_sof_schedule(hub, HUB_SOF_START_POLL, 2);
			}
			return; // one port per pass (TinyUSB breaks here)
		}
	}
	hub_start_status_poll(hub);
}

// GET_STATUS(hub) arrived: change word low byte in ctrl_buf[2]:
// bit0 = C_HUB_LOCAL_POWER, bit1 = C_HUB_OVER_CURRENT (USB 2.0 11.24.2.6).
static void on_hub_status(usb_add_t add, uhd_trans_status_t status, uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	if (status != UHD_TRANS_NOERROR) {
		hub_start_status_poll(hub);
		return;
	}

	uint8_t chg = hub->ctrl_buf[2];
	if (chg & 0x02) {
		// Over-current trip: the hub has cut port power. Ack the change, then
		// re-run the PORT_POWER walk (which ends by re-arming the poll).
		if (!hub_ctrl(add, HUB_REQTYPE_DEV_OUT, HUB_REQ_CLEAR_FEATURE,
		              HUB_FEAT_C_HUB_OVER_CURRENT, 0, NULL, 0,
		              on_hub_oc_cleared)) {
			hub_sof_schedule(hub, HUB_SOF_START_POLL, 2);
		}
	} else if (chg & 0x01) {
		if (!hub_ctrl(add, HUB_REQTYPE_DEV_OUT, HUB_REQ_CLEAR_FEATURE,
		              HUB_FEAT_C_HUB_LOCAL_POWER, 0, NULL, 0,
		              on_other_change_cleared)) {
			hub_sof_schedule(hub, HUB_SOF_START_POLL, 2);
		}
	} else {
		// No hub change bit set (spurious); just resume watching.
		hub_start_status_poll(hub);
	}
}

// C_HUB_OVER_CURRENT acked -> ports lost power; run the power walk again.
// `status` is deliberately ignored: re-powering is harmless if the ack
// failed, and a still-latched change simply gets reported again.
static void on_hub_oc_cleared(usb_add_t add, uhd_trans_status_t status,
                              uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	hub->enable_retries = 0; // fresh budget for the re-power walk
	hub->cur_port       = 1;
	if (!port_set_feature(hub->dev->address, 1, HUB_FEAT_PORT_POWER,
	                      on_port_power)) {
		hub_enable_retry(hub);
	}
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
	uint8_t feat;
	uhd_callback_setup_end_t cb = on_other_change_cleared;

	if (chg & 0x01) {
		// Connection change: ack, then attach/detach in on_conn_change_cleared.
		feat = HUB_FEAT_PORT_CONNECTION_CHANGE;
		cb   = on_conn_change_cleared;
	} else if (chg & 0x02) {
		feat = HUB_FEAT_PORT_ENABLE_CHANGE;
	} else if (chg & 0x04) {
		feat = HUB_FEAT_PORT_SUSPEND_CHANGE;
	} else if (chg & 0x08) {
		feat = HUB_FEAT_PORT_OVER_CURRENT_CHANGE;
	} else if (chg & 0x10) {
		feat = HUB_FEAT_PORT_RESET_CHANGE;
	} else {
		// No change bits set for this port; just look for the next change.
		hub_start_status_poll(hub);
		return;
	}
	if (!port_clear_feature(add, hub->cur_port, feat, cb)) {
		// change stays latched; the retried poll will re-report it
		hub_sof_schedule(hub, HUB_SOF_START_POLL, 2);
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
	if (hub == NULL) return;
	if (status != UHD_TRANS_NOERROR) {
		// Transient transfer error: keep the hub serviced. If the clear did
		// not take, the latched change will simply be reported again.
		hub_start_status_poll(hub);
		return;
	}

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

// Reset handshake failed (success is handled by on_reset_cleared): clear the
// handshake state, then let enumeration proceed to fail cleanly via the
// stashed callback.
static void hub_reset_fail(uhi_hub_t* hub) {
	uhd_callback_reset_t cb = hub->reset_cb;
	hub->reset_cb = NULL;
	hub->state    = HUB_ST_IDLE;
	if (cb) cb();
}

// Reset the hub port `dev` is attached to, then signal completion.
// (TinyUSB hub_port_reset = SET_FEATURE(PORT_RESET); hub.h:184)
//
// UHC's enumeration cannot proceed until `callback` fires:
//   SET_FEATURE(PORT_RESET) -> poll GET_STATUS(port) until C_PORT_RESET ->
//   CLEAR_FEATURE(C_PORT_RESET) -> callback(). The GET_STATUS poll count is
//   bounded (50); a hub completes reset in ~10-20ms and each poll costs at
//   least one control-transfer round-trip. TRSTRCY recovery after the reset
//   is provided by uhc.c's SOF timeouts (steps 2/4), not here.
void uhi_hub_send_reset(uhc_device_t* dev, uhd_callback_reset_t callback) {
	uhi_hub_t* hub = get_hub_by_dev(dev->hub);
	if (hub == NULL) { if (callback) callback(); return; }

	hub->reset_cb    = callback;
	hub->reset_port  = dev->hub_port;
	hub->reset_polls = 0;
	hub->state       = HUB_ST_RESETTING;
	if (!port_set_feature(hub->dev->address, hub->reset_port,
	                      HUB_FEAT_PORT_RESET, on_reset_feature_set)) {
		hub_reset_fail(hub);
	}
}

// PORT_RESET asserted -> start polling for reset completion.
static void on_reset_feature_set(usb_add_t add, uhd_trans_status_t status,
                                 uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	if (status != UHD_TRANS_NOERROR ||
	    !port_get_status(hub, hub->reset_port, on_reset_poll)) {
		hub_reset_fail(hub);
	}
}

// GET_STATUS(port) during reset -> done when change.reset (bit 4) is set.
static void on_reset_poll(usb_add_t add, uhd_trans_status_t status, uint16_t n) {
	uhi_hub_t* hub = get_hub_by_addr(add);
	if (hub == NULL) return;
	if (status != UHD_TRANS_NOERROR) { hub_reset_fail(hub); return; }

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
		if (!port_clear_feature(hub->dev->address, hub->reset_port,
		                        HUB_FEAT_PORT_RESET_CHANGE, on_reset_cleared)) {
			hub_reset_fail(hub);
		}
	} else if (++hub->reset_polls < 50) {
		// not done; poll again
		if (!port_get_status(hub, hub->reset_port, on_reset_poll)) {
			hub_reset_fail(hub);
		}
	} else {
		// Timed out waiting for reset; resume so enumeration fails cleanly.
		hub_reset_fail(hub);
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
	// Downstream devices are torn down by uhc_connection_tree's root-unplug
	// sweep in uhc.c before this runs; the slot only needs clearing here.
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

// Set while any device enumeration is in flight (src/usb.c). Deferred poll
// arms must not race the enumeration's control traffic on shared pipe 0.
extern volatile uint8_t usb_enumeration_active;

// 1 ms SOF tick (UHI sof_notify hook): runs the deferred actions scheduled
// by hub_sof_schedule() — the bPwrOn2PwrGood settle time, status-poll re-arm
// retries, and bounded enable-sequence retries.
void uhi_hub_sof(bool b_micro) {
	if (b_micro) return;
	for (uint8_t i = 0; i < UHI_HUB_MAX; i++) {
		uhi_hub_t* hub = &hubs[i];
		if (hub->dev == NULL || hub->sof_delay == 0) continue;
		if (--hub->sof_delay != 0) continue;

		uint8_t action = hub->sof_action;
		hub->sof_action = HUB_SOF_NONE;
		switch (action) {
		case HUB_SOF_START_POLL:
			if (usb_enumeration_active) {
				// pipe 0 is busy enumerating a device; hold the re-arm
				// (uhi_hub_poll_resume also fires when the enum finishes)
				hub_sof_schedule(hub, HUB_SOF_START_POLL, 2);
				break;
			}
			hub_start_status_poll(hub);
			break;
		case HUB_SOF_RETRY_ENABLE:
			hub_enable_start(hub);
			break;
		default:
			break;
		}
	}
}

#endif // USB_HOST_HUB_SUPPORT
