# USB Hub Bring-up — Retrospective

If I started the USB host hub work over, knowing what the debugging surfaced,
here's what I'd change — roughly in order of time saved.

## 1. Build the diagnostic overlay *first*, and keep it

The single thing that actually cracked both hard bugs was the OLED enumeration
overlay (`ENUM c=.. a=.. b=..`, endpoint-alloc size dumps). It was built late,
on a throwaway `enum-debug` branch, and then discarded. On this hardware there's
no easy gdb/serial — the OLED *is* your printf. Starting over, build a
descriptor/enumeration trace overlay before touching any driver code, and keep
it permanently behind a `USB_HOST_DEBUG` flag. Time spent theorizing before that
existed was wasted.

## 2. Treat TinyUSB + macOS as the oracle from day one

Two facts converged late: the hub driver was *ported from* TinyUSB, and the
failing devices enumerate fine on macOS / an RP2040 TinyUSB host — which is what
finally proved these were fixable driver bugs, not hardware limits. Anchoring on
TinyUSB as both the reference implementation and the behavioral oracle at the
start would have avoided the **wrong "composite device / isochronous" rabbit
hole**. Rule: before hypothesizing, capture the actual descriptors from the
failing device (`ioreg` / `lsusb`) and diff the host's behavior against a
known-good host.

## 3. Model the shared control pipe 0 as *the* core abstraction

Almost every hard bug traces back to one thing — the AT32UC3B has a single
control pipe multiplexed across all device addresses:

- the fixed-64 EP0 size that broke the nanoKONTROL2 (EP0 = 8),
- `uhd_ep_free(0xFF)` tearing down the shared pipe on any disconnect,
- the poll-pause/resume gate needed so a downstream enumeration doesn't race the
  hub's own status poll.

These were discovered as three separate bugs. Fresh, design pipe 0 explicitly up
front — a per-address EP0-size table, an owned serialization queue, and a "never
free pipe 0 in hub mode" invariant — and the individual bugs mostly wouldn't
have happened.

## 4. Assemble a device zoo and test the matrix early

The endpoint bugs were device-specific and surfaced one at a time. Gather the
awkward cases deliberately at the start — a small-EP0 device (nanoKONTROL2), a
high-speed device that falls back to full speed with a 512 B bulk EP (Digitone),
an FTDI grid, an HID keyboard, a class-compliant MIDI interface — and test each
**directly and behind the hub**, individually then combined. That converts
"mystery incompatibility six weeks in" into "row 3 of the test matrix fails."

## 5. Compile-and-audit the dead vendor code before trusting its logic

~9 latent bugs lived in ASF scaffolding that *never compiled* (the giveaway was
a literal syntax error, `&uhi_msc_dev[]`). The underlying multiplexing logic was
actually sound — but the uninitialized `ep_ctrl_size`, the broken free-address
search (`while (usb_addr_free++)` that never ran), and the self-referential
struct-tag error weren't. Lesson: any `#error` / `#if 0`-gated path is unverified
code — compile it in isolation and read every line before assuming any of it
works.

## What I'd keep

The end architecture was right, so don't relitigate it: porting the hub class
driver from TinyUSB, the poll-pause/resume serialization gate, and clamping
non-isochronous pipes to 64 B at full speed. The leverage insight — grid /
keyboard / MIDI are *different* USB classes, so mixed combos need only the hub
layer and no per-driver multi-instancing — is what kept the change small, and is
worth leaning on again.

Same-class multi-instancing was scoped out at first (the original plan cited "two
keyboards" as the deferred case), but **MIDI later got it anyway** —
`UHI_MIDI_MAX_DEV == 2`, so two MIDI devices run at once. That turned out cheap
because MIDI's state was straightforward to array-ify; HID and serial were left
single-instance (serial is the expensive one — it also needs `monome.c`'s global
serial backend reworked).

## Net

The technical fixes were correct; the cost was **sequencing and method**:
instrument first, read descriptors before theorizing, use the reference stack as
an oracle, and design around pipe 0 instead of discovering it. Bugs #2–#4 would
have been an afternoon each instead of a debugging saga.

Durable lesson: *on a printf-less embedded target, the enumeration-trace overlay
is infrastructure, not a throwaway.*
