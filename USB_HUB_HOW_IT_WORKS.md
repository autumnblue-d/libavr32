# USB Hub Support — What Makes It Work

This is a fork of **libavr32** carrying USB host **hub / multi-device** support
(`UHI_HUB`). With it, a monome grid + a USB keyboard + a MIDI controller run
simultaneously through a **powered** hub. All of it is gated by
`#define USB_HOST_HUB_SUPPORT` in `conf/conf_usb_host.h`; with the flag off the
library is byte-identical to stock libavr32.

Origin: Atmel's ASF stack shipped hub *scaffolding* that was **never compiled**
(the proof was a literal syntax error, `uhi_msc_dev_sel = &uhi_msc_dev[];`, in a
shipped file). This fork finishes and debugs that path and adds the missing hub
class driver.

---

## How to build / enable

This library does not build standalone — it is consumed as a git submodule by a
firmware project (e.g. **teletype** or **aleph**). The steps below assume
teletype.

### 1. Prerequisites

- **Ragel** 6.9 (state-machine compiler): `brew install ragel` / `apt install ragel`
- **AVR32 toolchain.** Easiest via Docker image `dewb/monome-build` (no local
  toolchain setup).

### 2. Point your project's submodule at this fork

In the consuming repo:

```bash
git submodule set-url libavr32 https://github.com/autumnblue-d/libavr32.git
git submodule update --init --remote libavr32
# or check out the specific branch/commit you want inside libavr32/
```

### 3. The hub flag (already enabled here)

`conf/conf_usb_host.h` already defines it and registers the hub driver:

```c
#define USB_HOST_HUB_SUPPORT
// UHI_HUB first so a hub device is claimed by the hub driver before others try.
#define USB_HOST_UHI  UHI_HUB, UHI_FTDI, UHI_HID, UHI_MSC, UHI_MIDI, UHI_MCDC
```

Comment out `USB_HOST_HUB_SUPPORT` to build the original single-device firmware.

### 4. Add the hub driver to your project's build

The `.c` and its include dir must be listed by the consumer. For teletype's
`module/config.mk`:

```make
CSRCS = \
    ...
    ../libavr32/src/usb/hub/uhi_hub.c   \
    ...
INC_PATH = \
    ...
    ../src/usb/hub                      \
    ...
```

(`conf/conf_usb_host.h` and the `uhi_hub.{c,h}` sources live in this repo, so no
other consumer-side source edits are needed.)

### 5. Build

Native (with the AVR32 toolchain on PATH):

```bash
cd module
make clean          # object files are shared with host builds — always clean when switching
make                # -> teletype.elf
```

Via Docker (recommended). On **Apple Silicon** the image is `linux/amd64`, so
pass `--platform linux/amd64` and give the command as a single string (the
entrypoint is `bash -c`):

```bash
docker run --rm --platform linux/amd64 -v "$(pwd)":/target dewb/monome-build \
  'cd module && make clean && make'
```

### 6. Sanity check

- **Flag off** → clean build, firmware byte-identical to stock.
- **Flag on** → compiles and links clean, ~+1.4 KB of hub code.
- On hardware: plug a **powered** hub with a grid + keyboard (or grid + MIDI);
  both work at once. Plug one device at a time (see limitations).

---

## What makes it work, layer by layer

### 1. The hub is just another USB device with a driver (`src/usb/hub/uhi_hub.c`)

ASF's host stack (UHC) dispatches every device to a list of class drivers (UHI).
This fork adds a hub-class driver (`UHI_HUB`), hand-ported from TinyUSB's
`hub.c` onto ASF's callback API. It's registered *first* in `USB_HOST_UHI` so a
hub is claimed by the hub driver before anything else looks at it.

The hub driver is an async state machine over control transfers:

- **enable** → GET_DESCRIPTOR to learn `bNbrPorts` → SET_FEATURE(PORT_POWER) on
  each port → arm the interrupt-IN "status change" poll.
- **status-change interrupt fires** → GET_STATUS on the changed port →
  CLEAR_FEATURE to ack the change → on a *connection* change, call
  `uhc_hub_port_change()`.

Because ASF's control-completion callbacks carry **no `user_data`** (and the
`add` they receive is unreliable once devices share pipe 0), the driver recovers
its state via `get_hub_by_addr()` — correct because `UHI_HUB_MAX == 1`, there's
only ever one hub.

### 2. The single control pipe is time-multiplexed (`asf/.../usbb/usbb_host.c`)

The hardware crux. The AT32UC3B has **one** control pipe (pipe 0) that must
serve every device's address. Atmel left three `#error TODO` markers here. The
multiplexing logic *underneath* them turned out to be functional:

- `uhd_setup_request()` already serializes control transfers in a FIFO.
- `uhd_ctrl_phase_setup()` already re-points pipe 0 at the head request's address
  (`uhd_configure_address(0, ...)`) before each transfer.

So the fix was mostly removing the `#error`s — plus two genuine bugs that would
corrupt transfers:

- `ep_ctrl_size` left **uninitialized** in the hub branch of
  `uhd_ctrl_phase_data_out()` → control-OUT corruption. Fixed by querying
  `uhd_get_pipe_size(0)` unconditionally.
- `uhd_ep_free()` with `endp==0xFF` (free-all on disconnect) would tear down the
  **shared pipe 0**, killing control transfers for every *remaining* device. Now
  pipe 0 is protected in hub mode — only an in-flight request for the leaving
  device is aborted.

### 3. Downstream devices get created and addressed (`asf/.../uhc/uhc.c`, `uhc.h`)

ASF had a `free(dev)` on disconnect but **no matching `malloc`** and no code to
build a non-root device — the downstream-device lifecycle was missing. Added:

- **`uhc_hub_port_change(hub, port, plug)`** — the entry point the hub driver
  calls. On connect it `malloc`s a `uhc_device_t`, links it into the (linear,
  NULL-terminated) device list, sets `hub`/`hub_port`, and kicks standard
  enumeration via `uhc_connection_tree(true, nd)`. On disconnect it finds the
  device by `(hub, port)` and unlinks/frees it. Lives in `uhc.c` because the
  device list and `uhc_connection_tree` are `static` there.
- A **rewritten free-address search**. The original `while (usb_addr_free++)`
  tested `0` (false) on the first iteration and never ran — so *every* device got
  address 1, colliding hub with downstream. Now scans 1–127 for the first unused
  address.
- A **struct tag** on `uhc_device_t`. The self-referential `prev`/`next`/`hub`
  pointers used the typedef name mid-definition, which doesn't exist yet — it
  never compiled. Tagging it `struct uhc_device` fixes that.
- **NULL-safe unlink** in `uhc_connection_tree` — the original assumed a circular
  list and dereferenced NULL at the tail.
- `SOFTWARE_LIMIT` made **non-fatal** in `uhc_enumeration_step14`. Since UHC calls
  *every* driver's `install()` on *every* device, a full single-instance driver
  (e.g. CDC once the grid is attached) would return `SOFTWARE_LIMIT` and abort a
  *different-class* device's enumeration (the keyboard). Now it's treated like
  "not mine," same as `UNSUPPORTED`.

### 4. Port reset handshake + the serialization gate

Enumeration of a downstream device can't proceed until its hub port is reset.
`uhi_hub_send_reset()` implements the real handshake: SET_FEATURE(PORT_RESET) →
poll GET_STATUS until the `C_PORT_RESET` change bit → CLEAR_FEATURE → fire the
enumeration continuation callback. The poll is self-clocking (each control
transfer ~1ms, reset completes in ~10–20ms).

The subtle bit that ties it together: **the hub's status poll is paused during a
downstream enumeration.** After a connection, `on_conn_change_cleared` does *not*
re-arm the poll — the new device is now doing control transfers on the shared
pipe 0, and a concurrent GET_STATUS would race it. Instead, `src/usb.c`'s
`usb_enum()` callback (fired at end-of-enumeration) calls
**`uhi_hub_poll_resume()`** once pipe 0 is free again. That's what lets a *second*
device (keyboard plugged after the grid) be detected: enumerate → resume →
detect next.

### 5. Loose ends cleaned up

- MSC (`asf/.../msc/host/uhi_msc.c`) kept single-instance — you never mount two
  USB sticks through a hub — and the `&uhi_msc_dev[]` syntax error fixed to point
  at the singleton.
- FTDI (`src/usb/ftdi/uhi_ftdi.c`) `#error` removed: the grid stays
  single-instance, fine since one grid + a different-class device is the target.

## Why the combo works with just this

The clever leverage: **grid = serial (FTDI/CDC), keyboard = HID, MIDI controller
= MIDI** are all *different classes*. Since UHC dispatches to one driver per class
and each existing singleton driver claims exactly one device, mixed-class combos
(grid + keyboard + MIDI) need **only** the new hub layer — no per-driver
multi-instancing. That's why the change is as small as it is. Two devices of the
*same* class (two keyboards) would need further per-driver work that isn't done.

## Control-pipe sizing: small-EP0 devices (nanoKONTROL2 fix)

Some devices have a control endpoint smaller than 64 bytes — the **Korg
nanoKONTROL2** uses `bMaxPacketSize0 = 8` (confirmed on a Mac via `ioreg`;
full-speed). The hub build had pinned the shared control pipe 0 at a **fixed 64
bytes** and never resized it, so the nanoKONTROL2's 8-byte control-IN packets
were read as *short packets* (= end-of-transfer). The multi-packet
config-descriptor read stopped early, `payload_trans < wTotalLength`, and
`uhc.c` aborted enumeration — the device **failed to enumerate even connected
directly** (this is always a hub build, so even a direct connection takes the
fixed-64 path).

Fix, in `asf/avr32/drivers/usbb/usbb_host.c`:

- **`uhd_ep0_alloc()`** configures pipe 0 to the caller's `ep_size` (the device's
  real `bMaxPacketSize0`) instead of a hardcoded 64, and — when the shared pipe
  is already up — **resizes** it if the size differs instead of short-circuiting.
  This restores ASF's normal per-device EP0 re-allocation that the hub code had
  defeated. *(Hardware-verified: the nanoKONTROL2 enumerates.)*
- **`uhd_ctrl_phase_setup()`** resizes pipe 0 to the *target* device's control
  size before every control transfer, using a small per-address size table
  populated by `uhd_ep0_alloc()`. This lets devices with different EP0 sizes
  (e.g. a 64-byte-EP0 grid and the 8-byte nanoKONTROL2) share pipe 0 behind a
  hub; the resize is a no-op when the size already matches. *(Built; the
  mixed-device case is the remaining thing to exercise on hardware.)*

The strict `payload_trans == wTotalLength` check in `uhc.c` is deliberately left
intact — a valid conformance guard, and the sizing fix removes the *cause* of
the short read so it now passes on its own.

## USB MIDI out: non-blocking TX ring queue

Separate from the hub work, `src/usb/midi/midi.c` gained a proper transmit path
(this snapshot carries it). The old `midi_write_packet()` handed a **local stack
buffer** to the asynchronous USB DMA and spin-waited *after* filling it, so a
rapid second note (e.g. a grid sequencer firing several tracks on one step)
clobbered the previous packet's still-in-flight buffer — corrupting/dropping
notes. It's now a **single-producer/ISR-consumer ring**: `midi_write_packet()`
enqueues and kicks a transfer only when idle (test-and-set under `cpu_irq_save`);
`midi_tx_done` (the transfer-complete ISR) chains the next. The drain is
**batched** — one bulk transfer carries the largest contiguous run of queued
events instead of one 4-byte event per transfer — so dense bursts keep up
(~16–32× the throughput). Note-lifecycle correctness (per-voice note-off before
retrigger/steal, minimum gate length) lives in the *consuming* firmware, not
here.

## USB HID: keyboard fixes

Two keyboard fixes (ported from Daanyal Baber's work, `#79`/`#80`) live in
`src/usb/hid/`:

- **`hid.c` — `u64` dirty bitfield.** `HID_FRAME_MAX_BYTES` is 64 but the
  changed-byte bitfield was a `u32`, so a report byte ≥ 32 did `1 << byte` on a
  32-bit int (undefined; can't set bits 32–63) → change detection broke for a
  range of keyboards. Now `u64` with `1ULL << byte`.
- **`uhi_hid.c` — re-arm the pipe on error.** `uhi_hid_report_reception` used to
  `return` early on a transfer error, skipping `uhi_hid_start_trans_report()` —
  so the interrupt-IN pipe was never re-armed and the keyboard went dead after a
  glitch/reconnect. It now parses only a good frame but **always** re-arms.

## Known limitations

- Disconnect isn't detected while another device is mid-enumeration (poll is
  paused).
- Failed/looping enumerations can leak a `uhc_device_t` (address climbs);
  power-cycle resets it.
- 3 simultaneous devices is at the UC3B pipe ceiling — grid+keyboard+MIDI fits,
  more may not.
- Simultaneous power-on of two devices can race the global `uhc_dev_enum`; plug
  one at a time.
- Use a **powered** hub — the module can't supply bus power for several
  downstream devices.

## In short

The feature works because Atmel's control-pipe multiplexing was secretly
functional under its `#error` markers, and this fork supplied the three
genuinely-missing pieces — a **hub class driver**, a **downstream-device
malloc/link/enumerate path**, and a **poll-pause/resume gate** that serializes
everything onto the single shared control pipe — while fixing ~9 latent bugs in
never-compiled code along the way.
