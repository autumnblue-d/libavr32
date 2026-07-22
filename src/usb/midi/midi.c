/*
  midi.c
  libavr32

  usb MIDI functions.
*/

#include <stdlib.h>
#include <string.h>

// asf
#include "interrupt.h" // cpu_irq_save/restore for the TX-queue kick
#include "print_funcs.h"

// libavr32
#include "conf_usb_host.h"
#include "events.h"
#include "events.h"
#include "midi.h"
#include "uhi_midi.h"


//------------------------------------
//------ defines

// RX buffer size.
// the buffer will start filling up if events come in faster than the polling rate
// the more full the buffer, the longer we'll spend in the usb read ISR parsing it...
// so it is a tradeoff.
#define MIDI_RX_EVENT_BUF_SIZE 16
// TX ring depth (4-byte USB-MIDI events). Absorbs bursts (e.g. Kria firing
// several tracks + note-offs on one clock step) so notes aren't dropped while
// the previous packet is still on the wire. 32 * 4 B = 128 B.
#define MIDI_TX_EVENT_BUF_SIZE 32


//------------------------------
//----- types

// usb midi event type
// See http://www.usb.org/developers/docs/devclass_docs/midi10.pdf
// Section 4 (page 16)
typedef union {
  struct { u8 header; u8 msg[3]; };
  u32 raw;
} usb_midi_event_t;

// System Real-Time messages (Clock/Start/Continue/Stop, status >= 0xF8) get
// their own ring, drained ahead of the normal ring by midi_tx_start(). This
// keeps transport/clock bytes from (a) being dropped when a note burst has
// filled the normal ring and (b) sitting behind that burst — the spec expects
// realtime bytes to reach the wire with minimal jitter. A realtime message
// still can't preempt a transfer already on the wire, but it goes out on the
// very next one. 8 * 4 B = 32 B; ample since these drain ~1 event/USB frame.
#define MIDI_RT_EVENT_BUF_SIZE 8

// Cached USB product-name string per device slot, truncated to fit one OLED
// line (~21 chars at 6px/char on a 128px-wide line).
#define MIDI_DEV_NAME_LEN 21

//------------------------------------
//------ static variables
//

// Per-device I/O state. One entry per connected USB MIDI device; the slot index
// matches the driver's device slot (uhi_midi.c), so a transfer callback maps the
// USB address it is handed back to a slot via uhi_midi_slot_by_add(). Buffers
// come first and the array is word-aligned so each rx/tx/rt buffer stays word
// aligned per the uhd_ep_run() docs.
typedef struct {
  usb_midi_event_t rxBuf[MIDI_RX_EVENT_BUF_SIZE];
  usb_midi_event_t txBuf[MIDI_TX_EVENT_BUF_SIZE];
  usb_midi_event_t rtBuf[MIDI_RT_EVENT_BUF_SIZE];

  bool connected;
  char name[MIDI_DEV_NAME_LEN];

  volatile bool rxBusy;
  u32 rxBytes;

  volatile bool txBusy;
  volatile u8 txGetIdx;
  volatile u8 txPutIdx;
  volatile u8 txInFlight; // events in the current (batched) transfer

  volatile u8 rtGetIdx;
  volatile u8 rtPutIdx;
  volatile u8 rtInFlight; // realtime events in the current transfer
  volatile bool rtActive; // is the in-flight transfer from rtBuf?
} midi_dev_state_t;

COMPILER_WORD_ALIGNED static midi_dev_state_t midi_devs[UHI_MIDI_MAX_DEV];

static void midi_tx_start(u8 dev); // starts a batched transfer; caller holds txBusy

// current packet data
static event_t ev = { .type = kEventMidiPacket, .data = 0x00000000 };

//------------------------------------
//----- static functions

// parse device `dev`'s rx buffer and spawn appropriate events
static void midi_parse_event(u8 dev) {
  midi_dev_state_t* d = &midi_devs[dev];
  int i;
  int eventCount = d->rxBytes >> 2; // assume full events; partials are dropped

  union { u32 data; s32 sdata; } buf;

  usb_midi_event_t* rxEvent = &(d->rxBuf[0]);

  for (i = 0; i < eventCount; i++) {
    // shift the 3-byte MIDI message into bits 8..31 (as midi_packet_parse
    // expects) and pack per-event metadata into the otherwise-unused low byte:
    //   low nibble  = virtual cable (MIDISPORT port A = 0, port B = 1, ...)
    //   high nibble = device slot (which physical MIDI device it arrived on)
    // both are <= 15. midi_packet_parse only reads bits 8..31.
    buf.data = (rxEvent->raw << 8) | (((u32)dev & 0x0f) << 4)
               | (rxEvent->header >> 4);
    ev.data = buf.sdata;
    event_post(&ev);

    ++rxEvent;
  }
}

// callback for the non-blocking asynchronous read. The transfer callback carries
// only the USB address, so map it back to a device slot.
static void midi_rx_done(usb_add_t add,
                         usb_ep_t ep,
                         uhd_trans_status_t stat,
                         iram_size_t nb) {
  (void)ep;
  uint8_t slot = uhi_midi_slot_by_add(add);
  if (slot == UHI_MIDI_NO_SLOT) {
    return; // device left between issuing the read and its completion
  }
  midi_dev_state_t* d = &midi_devs[slot];
  if (nb > 0) {
    if (stat == UHD_TRANS_NOERROR) {
      d->rxBytes = nb;
      midi_parse_event(slot);
    }
  }

  d->rxBusy = false;
}

// callback for the non-blocking asynchronous write. Runs in the USB
// transfer-complete ISR. The packet at txGetIdx just finished (NOERROR) or
// errored/timed out; either way advance past it and, if the ring still holds
// queued packets, immediately start the next so a burst drains back-to-back
// without the producer ever blocking.
static void midi_tx_done(usb_add_t add,
                         usb_ep_t ep,
                         uhd_trans_status_t stat,
                         iram_size_t nb) {
  (void)ep;
  (void)nb;
  uint8_t slot = uhi_midi_slot_by_add(add);
  if (slot == UHI_MIDI_NO_SLOT) {
    return; // device left mid-transfer; its state was already reset
  }
  midi_dev_state_t* d = &midi_devs[slot];
  if (stat != UHD_TRANS_NOERROR) {
    print_dbg("\r\n midi tx error (in callback). status: 0x");
    print_dbg_hex((u32)stat);
  }

  if (d->rtActive) {
    d->rtGetIdx = (d->rtGetIdx + d->rtInFlight) % MIDI_RT_EVENT_BUF_SIZE;
    d->rtInFlight = 0;
  } else {
    d->txGetIdx = (d->txGetIdx + d->txInFlight) % MIDI_TX_EVENT_BUF_SIZE;
    d->txInFlight = 0;
  }
  if (d->rtGetIdx != d->rtPutIdx || d->txGetIdx != d->txPutIdx) {
    midi_tx_start(slot); // more queued: chain the next batch (txBusy stays true)
  } else {
    d->txBusy = false; // both rings drained
  }
}

// Start a bulk transfer of the largest CONTIGUOUS run of queued events from
// txGetIdx (not wrapping past the buffer end — the wrapped remainder rides the
// next transfer). Batching many 4-byte events into one USB transfer is what
// lets a dense burst drain fast enough: one event per transfer (~1 USB frame
// each) can't keep up with Kria's bursts, so the ring overflowed and dropped
// note-ons. Caller must already hold txBusy and ensure the ring is non-empty.
static void midi_tx_start(u8 dev) {
  midi_dev_state_t* d = &midi_devs[dev];
  // Realtime ring first: transport/clock bytes jump ahead of queued notes.
  if (d->rtGetIdx != d->rtPutIdx) {
    u8 run = (d->rtPutIdx > d->rtGetIdx) ? (u8)(d->rtPutIdx - d->rtGetIdx)
                                         : (u8)(MIDI_RT_EVENT_BUF_SIZE - d->rtGetIdx);
    d->rtInFlight = run;
    d->rtActive = true;
    uhi_midi_out_run(dev, (uint8_t*)&d->rtBuf[d->rtGetIdx],
                     run * sizeof(usb_midi_event_t), &midi_tx_done);
    return;
  }
  d->rtActive = false;
  u8 run = (d->txPutIdx >= d->txGetIdx) ? (u8)(d->txPutIdx - d->txGetIdx)
                                        : (u8)(MIDI_TX_EVENT_BUF_SIZE - d->txGetIdx);
  d->txInFlight = run;
  uhi_midi_out_run(dev, (uint8_t*)&d->txBuf[d->txGetIdx],
                   run * sizeof(usb_midi_event_t), &midi_tx_done);
}


//-----------------------------------------
//----- extern functions

// read and spawn events (non-blocking); poll every connected device
extern void midi_read(void) {
  for (u8 i = 0; i < UHI_MIDI_MAX_DEV; i++) {
    midi_dev_state_t* d = &midi_devs[i];
    if (!d->connected) {
      continue;
    }
    if (d->rxBusy == false) {
      d->rxBusy = true;
      d->rxBytes = 0;
      if (!uhi_midi_in_run(i, (u8*)d->rxBuf, sizeof d->rxBuf, &midi_rx_done)) {
        // The read did not start (e.g. the device just left), so no callback
        // will fire to clear rxBusy — clear it here so the next poll retries.
        d->rxBusy = false;
        print_dbg("\r\n midi rx endpoint error");
      }
    }
  }
  return;
}

// write to MIDI device `device`
extern bool midi_write(u8 device, const u8* data, u32 bytes) {
  if (device >= UHI_MIDI_MAX_DEV || !midi_devs[device].connected) {
    return false;
  }
  midi_dev_state_t* dev = &midi_devs[device];
  // NB: this function is not currently used across the module code
  // base therefore the precise nature of the incoming buffer layout
  // is not well defined.
  //
  // here it is assumed that the midi data is packed (not padded to 3
  // bytes) but that running status is not used...
  //
  // TODO: testing...
	//
	// FIXME: if txBuf is not large enough to hold all of data the extra
	// msgs in data are dropped

  u8 events = 1;
  usb_midi_event_t* tx = &(dev->txBuf[0]);
  const usb_midi_event_t* txEnd = &(dev->txBuf[MIDI_RX_EVENT_BUF_SIZE]);

  u8* d = (u8*)data;
  const u8* dEnd = data + bytes;

  u8 status, com, ch;

  if (dev->txBusy == false) {
    print_dbg("\r\n midi_write: no buffers available");
    return false;
  }

  while (tx < txEnd && d < dEnd) {
    // clean the tx buffer
    tx->raw = 0x00000000;

    // grab the status byte
    status = *d; d++;
    com = status >> 4;
    ch = status & 0x0f;

    if (com < 0x8) {
      // bad status byte, just bail
      print_dbg("\r\n midi_write: bad status in data, skipping write");
      return false;
    }

    // format usb midi header, high nib of 1st byte = virtual cable
    // low nib = 4b com code, duplicated from status byte
    tx->header = 0x10 | com;
    tx->msg[0] = status;

    // based on message type copy the correct number of bytes into the
    // events
    if (com < 0xf) {
      // channel mode message
      tx->msg[1] = *d; d++;
      if (com == 0xc || com == 0xd) {
        // program change, aftertouch => 1 byte, nothing more
      }
      else {
        tx->msg[2] = *d; d++;
      }
    }
    else if (ch < 0x8) {
      // system common message
      if (ch == 0x0 || ch == 0x7) {
        // sysex start, end
        print_dbg("\r\n midi_write: sysex not supported, skipping write");
        return false;
      }
      else if (ch == 0x2) {
        // song position pointer
        tx->msg[1] = *d; d++;
        tx->msg[2] = *d; d++;
      }
      else if (ch == 0x1 || ch == 0x3) {
        // midi time code quarter frame, song select
        tx->msg[1] = *d; d++;
      }
      else {
        // undefined or tune request
        // ...nothing to do
      }
    }
    else {
      // system realtime message
      // ...status byte only, nothing to do
    }

    tx++; events++;
  }

  dev->txBusy = true;

  if (!uhi_midi_out_run(device, (uint8_t*)dev->txBuf, events * sizeof(usb_midi_event_t), &midi_tx_done)) {
    // hm, every uhd enpoint run always returns unspecified error...
    //  print_dbg("\r\n midi tx endpoint error");
  }

  return true;
}
// Queue a 4-byte USB-MIDI packet for transmission (non-blocking).
//
// Writes the next TX ring slot and, only if no transfer is in flight, kicks one
// off; otherwise the midi_tx_done ISR chains this packet after the in-flight
// one. This replaces the previous implementation, which handed a *local stack*
// buffer to the asynchronous DMA and spin-waited AFTER writing it — so a rapid
// second call (e.g. Kria firing several tracks on one clock step) clobbered the
// first packet's in-flight buffer, corrupting/dropping notes.
extern void midi_write_packet(u8 device, u8 cable_number, u8 *pack) {
  if (device >= UHI_MIDI_MAX_DEV || !midi_devs[device].connected) {
    return;
  }
  midi_dev_state_t* dev = &midi_devs[device];
  // System Real-Time (status >= 0xF8) uses the priority ring; everything else
  // the normal ring. A full realtime ring is near-impossible in practice (it
  // drains a message per USB frame), but if it happens we still drop rather
  // than clobber an in-flight buffer.
  bool is_realtime = pack[0] >= 0xF8;
  usb_midi_event_t* e;
  if (is_realtime) {
    u8 next = (dev->rtPutIdx + 1) % MIDI_RT_EVENT_BUF_SIZE;
    if (next == dev->rtGetIdx) {
      print_dbg("\r\n midi tx realtime queue full, dropping packet");
      return;
    }
    e = &dev->rtBuf[dev->rtPutIdx];
    dev->rtPutIdx = next;
  } else {
    u8 next = (dev->txPutIdx + 1) % MIDI_TX_EVENT_BUF_SIZE;
    if (next == dev->txGetIdx) {
      // Ring full: drop rather than block or corrupt an in-flight one.
      print_dbg("\r\n midi tx queue full, dropping packet");
      return;
    }
    e = &dev->txBuf[dev->txPutIdx];
    dev->txPutIdx = next;
  }
  // USB-MIDI event: header = (cable << 4) | CIN. For channel-voice messages
  // (0x8-0xE) and system-real-time (0xF8-0xFF, single byte) the status high
  // nibble already equals the correct CIN. The 3-byte / 2-byte system-common
  // messages (0xF2 SPP; 0xF1 MTC / 0xF3 song-select) are the exception -- their
  // nibble is 0xF, so give them their spec CIN or a strict host drops the data
  // bytes.
  u8 cin;
  if (pack[0] == 0xF2)
    cin = 0x3;  // song position pointer: 3-byte system common
  else if (pack[0] == 0xF1 || pack[0] == 0xF3)
    cin = 0x2;  // MTC quarter-frame / song select: 2-byte system common
  else
    cin = pack[0] >> 4;
  e->header = ((cable_number << 4) & 0xf0) | cin;
  e->msg[0] = pack[0];
  e->msg[1] = pack[1];
  e->msg[2] = pack[2];

  // Start a transfer only if the engine is idle. Test-and-set txBusy with
  // interrupts masked so we don't race midi_tx_done (ISR) clearing it. When we
  // win the claim no transfer is in flight, so reading txGetIdx here is safe.
  irqflags_t flags = cpu_irq_save();
  bool start = !dev->txBusy;
  if (start) {
    dev->txBusy = true;
  }
  cpu_irq_restore(flags);
  if (start) {
    midi_tx_start(device); // batches all currently-queued events into one transfer
  }
}

// MIDI device was plugged or unplugged. The driver clears its slot only after
// this returns on unplug (see uhi_midi_uninstall), so uhi_midi_slot_by_add()
// resolves the slot in both directions.
extern void midi_change(uhc_device_t* dev, u8 plug) {
  event_t e;

  uint8_t slot = uhi_midi_slot_by_add(dev->address);
  if (slot == UHI_MIDI_NO_SLOT) {
    return;
  }
  midi_dev_state_t* d = &midi_devs[slot];

  // Reset this device's rings either way (connect = fresh start; disconnect =
  // drop anything queued for the departed device).
  d->rxBusy = false;
  d->txBusy = false;
  d->txGetIdx = d->txPutIdx = 0;
  d->txInFlight = 0;
  d->rtGetIdx = d->rtPutIdx = 0;
  d->rtInFlight = 0;
  d->rtActive = false;

  if (plug) {
    d->connected = true;
    d->name[0] = '\0';  // fetched later, off the main loop -- see midi_dev_fetch_name()
    e.type = kEventMidiConnect;
  } else {
    d->connected = false;
    d->name[0] = '\0';
    e.type = kEventMidiDisconnect;
  }

  // carry the device slot so the main loop can tell which device changed
  e.data = slot;

  // posting an event so the main loop can respond
  event_post(&e);
}

// Fetch and cache the USB product-name string for device slot `dev`. Does a
// blocking control transfer (uhc_dev_get_string_product()), so this must only
// be called from ordinary main-loop context (e.g. the ScreenRefresh handler),
// never from midi_change()/uhi_midi_enable() -- those run synchronously inside
// the UHC enumeration's own interrupt-driven setup-request callback chain, and
// a nested blocking transfer there deadlocks waiting for an interrupt that
// can't fire until the current callback returns.
void midi_dev_fetch_name(u8 dev) {
  if (dev >= UHI_MIDI_MAX_DEV || !midi_devs[dev].connected) return;

  uhc_device_t* d = uhi_midi_slot_dev(dev);
  if (d == NULL) return;

  char* product = uhc_dev_get_string_product(d);
  if (product) {
    strncpy(midi_devs[dev].name, product, MIDI_DEV_NAME_LEN - 1);
    midi_devs[dev].name[MIDI_DEV_NAME_LEN - 1] = '\0';
    free(product);
  } else {
    strcpy(midi_devs[dev].name, "MIDI");
  }
}

bool midi_dev_connected(u8 dev) {
  if (dev >= UHI_MIDI_MAX_DEV) return false;
  return midi_devs[dev].connected;
}

const char* midi_dev_name(u8 dev) {
  if (dev >= UHI_MIDI_MAX_DEV) return "";
  return midi_devs[dev].name;
}

