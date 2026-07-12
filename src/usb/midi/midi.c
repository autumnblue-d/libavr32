/*
  midi.c
  libavr32

  usb MIDI functions.
*/

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

//------------------------------------
//------ static variables
// 

static bool midi_connected = false;
// buffers must be word aligned per docs on uhd_ep_run()
COMPILER_WORD_ALIGNED static usb_midi_event_t rxBuf[MIDI_RX_EVENT_BUF_SIZE];
static volatile bool rxBusy = false;
static u32 rxBytes = 0;

// try using an output buffer and adding the extra nib we saw on input ...
COMPILER_WORD_ALIGNED static usb_midi_event_t txBuf[MIDI_TX_EVENT_BUF_SIZE];
static volatile bool txBusy = false;
static volatile u8 txGetIdx = 0;
static volatile u8 txPutIdx = 0;
static volatile u8 txInFlight = 0; // events in the current (batched) transfer

// System Real-Time messages (Clock/Start/Continue/Stop, status >= 0xF8) get
// their own ring, drained ahead of the normal ring by midi_tx_start(). This
// keeps transport/clock bytes from (a) being dropped when a note burst has
// filled the normal ring and (b) sitting behind that burst — the spec expects
// realtime bytes to reach the wire with minimal jitter. A realtime message
// still can't preempt a transfer already on the wire, but it goes out on the
// very next one. 8 * 4 B = 32 B; ample since these drain ~1 event/USB frame.
#define MIDI_RT_EVENT_BUF_SIZE 8
COMPILER_WORD_ALIGNED static usb_midi_event_t rtBuf[MIDI_RT_EVENT_BUF_SIZE];
static volatile u8 rtGetIdx = 0;
static volatile u8 rtPutIdx = 0;
static volatile u8 rtInFlight = 0; // realtime events in the current transfer
static volatile bool rtActive = false; // is the in-flight transfer from rtBuf?

static void midi_tx_start(void); // starts a batched transfer; caller holds txBusy

// current packet data
static event_t ev = { .type = kEventMidiPacket, .data = 0x00000000 };

//------------------------------------
//----- static functions

// parse the buffer and spawn appropriate events
static void midi_parse_event(void) {
  int i;
  int eventCount = rxBytes >> 2; // assume we receive full events, partials are dropped

  union { u32 data; s32 sdata; } buf;

  usb_midi_event_t* rxEvent = &(rxBuf[0]);

  for (i = 0; i < eventCount; i++) {
    // shift the 3-byte MIDI message into bits 8..31 (as midi_packet_parse
    // expects) and stash the virtual cable number (header high nibble) in the
    // otherwise-unused low byte so downstream code can tell which port it came
    // from (e.g. MIDISPORT port A = cable 0, port B = cable 1).
    buf.data = (rxEvent->raw << 8) | (rxEvent->header >> 4);
    ev.data = buf.sdata;
    event_post(&ev);

    ++rxEvent;
  }
}

// callback for the non-blocking asynchronous read.
static void midi_rx_done(usb_add_t add,
                         usb_ep_t ep,
                         uhd_trans_status_t stat,
                         iram_size_t nb) {
  if (nb > 0) {
    if (stat == UHD_TRANS_NOERROR) {
      rxBytes = nb;
      midi_parse_event();
    }
  }

  rxBusy = false;
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
  (void)add;
  (void)ep;
  (void)nb;
  if (stat != UHD_TRANS_NOERROR) {
    print_dbg("\r\n midi tx error (in callback). status: 0x");
    print_dbg_hex((u32)stat);
  }

  if (rtActive) {
    rtGetIdx = (rtGetIdx + rtInFlight) % MIDI_RT_EVENT_BUF_SIZE;
    rtInFlight = 0;
  } else {
    txGetIdx = (txGetIdx + txInFlight) % MIDI_TX_EVENT_BUF_SIZE;
    txInFlight = 0;
  }
  if (rtGetIdx != rtPutIdx || txGetIdx != txPutIdx) {
    midi_tx_start(); // more queued: chain the next batch (txBusy stays true)
  } else {
    txBusy = false; // both rings drained
  }
}

// Start a bulk transfer of the largest CONTIGUOUS run of queued events from
// txGetIdx (not wrapping past the buffer end — the wrapped remainder rides the
// next transfer). Batching many 4-byte events into one USB transfer is what
// lets a dense burst drain fast enough: one event per transfer (~1 USB frame
// each) can't keep up with Kria's bursts, so the ring overflowed and dropped
// note-ons. Caller must already hold txBusy and ensure the ring is non-empty.
static void midi_tx_start(void) {
  // Realtime ring first: transport/clock bytes jump ahead of queued notes.
  if (rtGetIdx != rtPutIdx) {
    u8 run = (rtPutIdx > rtGetIdx) ? (u8)(rtPutIdx - rtGetIdx)
                                   : (u8)(MIDI_RT_EVENT_BUF_SIZE - rtGetIdx);
    rtInFlight = run;
    rtActive = true;
    uhi_midi_out_run((uint8_t*)&rtBuf[rtGetIdx],
                     run * sizeof(usb_midi_event_t), &midi_tx_done);
    return;
  }
  rtActive = false;
  u8 run = (txPutIdx >= txGetIdx) ? (u8)(txPutIdx - txGetIdx)
                                  : (u8)(MIDI_TX_EVENT_BUF_SIZE - txGetIdx);
  txInFlight = run;
  uhi_midi_out_run((uint8_t*)&txBuf[txGetIdx],
                   run * sizeof(usb_midi_event_t), &midi_tx_done);
}


//-----------------------------------------
//----- extern functions

// read and spawn events (non-blocking)
extern void midi_read(void) {
  if(!midi_connected) {
    return;
  }
  if (rxBusy == false) {
    rxBusy = true;
    rxBytes = 0;
    if (!uhi_midi_in_run((u8*)rxBuf, sizeof rxBuf, &midi_rx_done)) {
      // hm, every uhd enpoint run always returns error...
      // ...because most of the time a rx job is already running, by only
      // running the endpoint read after midi_rx_done has set rxBusy to false
      // the errors here stop.
      print_dbg("\r\n midi rx endpoint error");
    }
  }
  return;
}

// write to MIDI device
extern bool midi_write(const u8* data, u32 bytes) {
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
  usb_midi_event_t* tx = &(txBuf[0]);
  const usb_midi_event_t* txEnd = &(txBuf[MIDI_RX_EVENT_BUF_SIZE]);

  u8* d = (u8*)data;
  const u8* dEnd = data + bytes;

  u8 status, com, ch;

  if (txBusy == false) {
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

  txBusy = true;

  if (!uhi_midi_out_run((uint8_t*)txBuf, events * sizeof(usb_midi_event_t), &midi_tx_done)) {
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
extern void midi_write_packet(u8 cable_number, u8 *pack) {
  if (!midi_connected) {
    return;
  }
  // System Real-Time (status >= 0xF8) uses the priority ring; everything else
  // the normal ring. A full realtime ring is near-impossible in practice (it
  // drains a message per USB frame), but if it happens we still drop rather
  // than clobber an in-flight buffer.
  bool is_realtime = pack[0] >= 0xF8;
  usb_midi_event_t* e;
  if (is_realtime) {
    u8 next = (rtPutIdx + 1) % MIDI_RT_EVENT_BUF_SIZE;
    if (next == rtGetIdx) {
      print_dbg("\r\n midi tx realtime queue full, dropping packet");
      return;
    }
    e = &rtBuf[rtPutIdx];
    rtPutIdx = next;
  } else {
    u8 next = (txPutIdx + 1) % MIDI_TX_EVENT_BUF_SIZE;
    if (next == txGetIdx) {
      // Ring full: drop rather than block or corrupt an in-flight one.
      print_dbg("\r\n midi tx queue full, dropping packet");
      return;
    }
    e = &txBuf[txPutIdx];
    txPutIdx = next;
  }
  // USB-MIDI event: header = (cable << 4) | CIN, CIN = status high nibble.
  e->header = ((cable_number << 4) & 0xf0) | (pack[0] >> 4);
  e->msg[0] = pack[0];
  e->msg[1] = pack[1];
  e->msg[2] = pack[2];

  // Start a transfer only if the engine is idle. Test-and-set txBusy with
  // interrupts masked so we don't race midi_tx_done (ISR) clearing it. When we
  // win the claim no transfer is in flight, so reading txGetIdx here is safe.
  irqflags_t flags = cpu_irq_save();
  bool start = !txBusy;
  if (start) {
    txBusy = true;
  }
  cpu_irq_restore(flags);
  if (start) {
    midi_tx_start(); // batches all currently-queued events into one transfer
  }
}

// MIDI device was plugged or unplugged
extern void midi_change(uhc_device_t* dev, u8 plug) {
  event_t e;

  if (plug) {
    midi_connected = true;
    rxBusy = false;
    txBusy = false;
    txGetIdx = txPutIdx = 0; // reset the TX ring
    txInFlight = 0;
    rtGetIdx = rtPutIdx = 0; // reset the realtime ring
    rtInFlight = 0;
    rtActive = false;
    e.type = kEventMidiConnect;
  } else {
    midi_connected = false;
    txBusy = false;
    txGetIdx = txPutIdx = 0; // drop anything queued for the departed device
    txInFlight = 0;
    rtGetIdx = rtPutIdx = 0;
    rtInFlight = 0;
    rtActive = false;
    e.type = kEventMidiDisconnect;
  }

  // posting an event so the main loop can respond
  event_post(&e); 
}

