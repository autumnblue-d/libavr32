# USB Hub Support — PR Review Guide

A reviewer's entry point for the `hub-snapshot` branch of this libavr32 fork
(`autumnblue-d/libavr32`). It maps the diff into digestible chunks, points at the
deep-dive docs already in the tree, and flags where a maintainer's eyes matter
most. Written for someone who knows this hardware (hi dewb — this is the work
you scoped in aleph issue #67).

**Base:** `b37dc3f` (merge-base with `monome/libavr32` main).
**Size:** 23 commits, 27 files, **+2336 / −180**. Of that, ~1165 lines are the
new hub driver + its docs; the rest is the UHC/USBB plumbing plus unconditional
MIDI/HID fixes (see the gating note below).

---

## 0. TL;DR for the reviewer

- This finishes the **USB host hub / multi-device** path that ASF shipped as
  *never-compiled* scaffolding. A monome grid + a full-speed USB keyboard + a
  MIDI controller now run at once behind a **powered** hub.
- The whole hub *core* is behind `#define USB_HOST_HUB_SUPPORT` (an **application
  opt-in** — the consuming build defines it and compiles `uhi_hub.c`). With the
  flag off, the UHC/USBB/hub-driver paths are inert.
- **⚠️ But the snapshot is not entirely gated.** It also carries **always-on**
  MIDI-TX and HID fixes (§4c below). "Flag off ⇒ byte-identical to stock" holds
  only for the hub core, **not** for the whole branch. If you want a strictly
  byte-identical-when-off PR, those want splitting out.
- **The leverage insight** (worth internalizing before reading code): grid =
  serial (FTDI/CDC), keyboard = HID, MIDI = MIDI — all *different* USB classes.
  UHC dispatches one driver per class, and each existing singleton claims one
  device, so **mixed-class combos need only the new hub layer** — no per-driver
  multi-instancing required. That's why the change is as small as it is. Same-class
  duplicates are a separate axis: **MIDI has been multi-instanced (2 devices,
  `UHI_MIDI_MAX_DEV`)**; HID and serial have not (see §2).
- **The proof this was dead code:** a literal syntax error survived in a shipped
  ASF file — `uhi_msc_dev_sel = &uhi_msc_dev[];` (empty `[]` subscript). Nobody
  had ever built this path. Treat every hub-branch line as unvetted vendor code
  that was compiled here for the first time.

---

## 1. Read these first (they already exist in-tree)

Don't re-derive the design from the diff — three docs on this branch explain it:

| Doc | What it gives you |
|-----|-------------------|
| **`USB_HUB_HOW_IT_WORKS.md`** | The layer-by-layer design + rationale. **Read this before the code.** Covers the hub state machine, pipe-0 multiplexing, downstream-device lifecycle, the poll-pause/resume gate, and every known limitation. |
| **`HUB_RETRO.md`** | A short retrospective — what was hard and why. Useful for *understanding intent* and judging whether the fixes address root causes vs symptoms. |
| **`TELETYPE_HUB_PATCH.md`** | The consumer (teletype) side: the 3-file `module/*` patch, the NVRAM 200K→199K shrink, and the build recipe. Only relevant if you review the firmware wiring too. |

If you have 20 minutes: read §0 here + the "What makes it work, layer by layer"
section of `USB_HUB_HOW_IT_WORKS.md`, then skim the two files in §4a below.

---

## 2. Scope & non-goals

**In scope (delivered + hardware-verified):** hub enumeration, downstream-device
malloc/link/enumerate, control-pipe (pipe 0) time-multiplexing across addresses,
port reset handshake, poll-pause/resume serialization, low-speed-behind-hub
rejection, hub-class hardening (over-current recovery, ≤7 ports, settle delays),
root-port (dock) unplug teardown, duplicate-connect teardown.

**Same-class duplicates — partly done:** **MIDI is multi-instanced** —
`UHI_MIDI_MAX_DEV == 2`, so **two MIDI devices work at once** (free-slot search +
`add → slot` lookup in the transfer callbacks). This is the one driver that got
the per-driver multi-instancing treatment. **Still single-instance (not done):**
two HID keyboards, and grid + a second serial device (FTDI/CDC stays one device,
and `monome.c`'s global serial backend would need reworking).

**Explicit non-goals:** the remaining same-class duplicates above (two keyboards,
two serial); cascaded hub-behind-hub (`UHI_HUB_MAX == 1`, deliberate — a second
tier costs a status pipe the device budget can't spare); mixed USB speeds on one
tree (hardware limit, not fixable in firmware).

---

## 3. The one hardware fact everything hinges on

The AT32UC3B USBB host has **one** control pipe (pipe 0), shared across every
device address, and is **full-speed only** with a small pipe budget (~7). Almost
every non-trivial hunk is a consequence of that single pipe:

- transfers on pipe 0 must be **serialized** (a FIFO — Atmel already had this);
- pipe 0 must be **re-pointed at the target address** before each transfer
  (Atmel already had this too, under an `#error`);
- pipe 0 must be **resized** to each device's `bMaxPacketSize0` (this is the
  nanoKONTROL2 fix — it was pinned at 64, breaking 8-byte-EP0 devices);
- pipe 0 must **never be freed** on a downstream disconnect (it was, via
  `uhd_ep_free(…, 0xFF)`);
- the hub's own status poll must **pause** while a downstream device enumerates
  on pipe 0, and **resume** at end-of-enumeration — otherwise the two race.

Keep "there is only one control pipe" in your head and the diff reads as one
coherent change rather than nine scattered ones.

---

## 4. The diff, grouped for review

Reviewer priority: **P0 = the crux, read every line** · **P1 = important** ·
**P2 = supporting / low-risk**.

### 4a. The hub core — new code (P0)

| File | ± | What / where to focus |
|------|---|-----------------------|
| `src/usb/hub/uhi_hub.c` | **+713** | The whole hub class driver, hand-ported from TinyUSB `hub.c`. The async control-transfer state machine: descriptor → per-port PORT_POWER → status-change poll → GET_STATUS/CLEAR_FEATURE → `uhc_hub_port_change()`. **Focus:** `get_hub_by_addr()` state recovery (safe only because `UHI_HUB_MAX==1`), the reset handshake, over-current recovery, the `uhi_hub_sof` retry engine, low-speed reject mask. |
| `src/usb/hub/uhi_hub.h` | +108 | Driver contract + `UHI_HUB_MAX`, port masks. Small; read it to anchor the `.c`. |

### 4b. UHC / USBB plumbing — modified vendor code, all under the flag (P0)

| File | ± | What / where to focus |
|------|---|-----------------------|
| `asf/.../usbb/usbb_host.c` | +166 | **The pipe-0 crux.** Removes 3 `#error TODO`s over functional multiplexing; fixes uninitialized `ep_ctrl_size`; protects pipe 0 from free-all; **EP0 sizing** (`uhd_ep0_alloc` + per-address size table in `uhd_ctrl_phase_setup`) — the nanoKONTROL2 fix; the **bulk NAK throttle** (freeze-and-retry-at-SOF, the root-cause fix for grid starvation). |
| `asf/.../uhc/uhc.c` | +172 | Downstream lifecycle: **`uhc_hub_port_change()`** (malloc/link/enumerate + duplicate-connect teardown), rewritten free-address search (the `while(usb_addr_free++)` bug), NULL-safe unlink, `SOFTWARE_LIMIT` made non-fatal, root-port unplug teardown, low-speed reject in `uhc_enumeration_step4`. |
| `asf/.../uhc/uhc.h` | +24 | Struct **tag** on `uhc_device_t` (self-referential pointers never compiled untagged) + `uhc_hub_port_change` decl. |
| `asf/.../uhc/uhd.h` | +15 | UHD hooks for the above. |
| `asf/.../msc/host/uhi_msc.c` | +16 | Removes the MSC `#error`; **fixes the `&uhi_msc_dev[]` syntax error** (the smoking gun); MSC stays single-instance. |
| `conf/conf_usb_host.h` | +15 | The opt-in flag + `USB_HOST_UHI` list (UHI_HUB **first**). This is the gate — verify flag-off restores the original list exactly. |

### 4c. Always-on USB fixes — NOT gated by the flag (P1) ⚠️

These change behavior in **every** build, hub or not. Called out separately
because they break the "byte-identical when off" property and are arguably a
separate PR:

| File | ± | What |
|------|---|------|
| `src/usb/midi/midi.{c,h}` | +353 | MIDI-TX **non-blocking ring** (old path handed a stack buffer to async DMA + spin-waited → clobbered in-flight packets on rapid notes); batched drain; System Real-Time priority ring; 3-byte system-common CIN fix. |
| `src/usb/midi/uhi_midi.{c,h}` | +153 | MIDI **multi-device** support (`UHI_MIDI_MAX_DEV == 2` — the driver is array-ified with a free-slot search and per-`add` slot lookup, so two MIDI devices run at once) + connected-device-name cache. This is the only class that's been multi-instanced. |
| `src/usb/hid/uhi_hid.c` | −1 | Re-arm the interrupt-IN pipe on transfer error (keyboard went dead after a glitch). Ported from Daanyal Baber #79/#80. |
| `src/usb/ftdi/uhi_ftdi.c` | +10 | FTDI `#error` removed; grid stays single-instance. (Has 1 flag guard.) |
| `src/usb.{c,h}` | +32 | `usb_enumeration_active` + `uhi_hub_poll_resume()` hook from `usb_enum()` — the serialization gate. (2 flag guards.) |

### 4d. Supporting / low-risk (P2)

`asf/.../fat/{fat_unusual,navigation}.c` (+3, `-Wchar-subscripts` + a guard),
`src/usb/cdc/{cdc,uhi_cdc}.c` (+3), `src/monome.c` (+7, don't clobber the global
serial backend on hub/keyboard connect), `src/events.c` (+1), `src/random.c`
(+2). Plus the three docs and `libavr32-teletype-hub.patch` (+206, the
consumer-side wiring — review only if you're checking the teletype integration).

---

## 5. Highest-value things to scrutinize

If review time is limited, spend it here — these are where a subtle bug would hide:

1. **Pipe-0 EP0 resizing** (`usbb_host.c`, `uhd_ep0_alloc` + `uhd_ctrl_phase_setup`).
   The per-address size table is new. Does it resize correctly when a
   64-byte-EP0 grid and an 8-byte-EP0 nanoKONTROL2 share pipe 0? *The mixed-EP0
   case behind a hub is the one combination flagged "built, not yet exercised on
   hardware" in `USB_HUB_HOW_IT_WORKS.md` §control-pipe sizing.*
2. **The poll-pause/resume gate** (`uhi_hub.c` `on_conn_change_cleared` deliberately
   *not* re-arming + `usb.c` `usb_enum` → `uhi_hub_poll_resume`). This is the
   serialization contract onto pipe 0. Is there any enumeration exit path that
   fails to call resume, wedging the hub poll forever?
3. **The `uhc_dev_enum` race** — simultaneous power-on of two downstream devices
   can race the single global enum state (documented; mitigation is "plug one at
   a time"). Confirm you agree this is acceptable vs. wanting an explicit
   "enum outstanding" gate.
4. **Device-list leaks on failed enumeration.** Low-speed-behind-hub looping is
   now handled (reject mask), but *other* failed/looping enumerations can still
   leak a `uhc_device_t` (address climbs; power-cycle resets). Acceptable?
5. **NAK throttle blast radius** (`usbb_host.c`). It changes bulk-pipe scheduling
   for *all* bulk transfers, not just hub mode — confirm it can't regress
   single-device grid/MSC throughput. (There's a runtime bypass knob,
   `uhd_bulk_nak_throttle_set`, from commit `141d47c`.)

---

## 6. Open questions for the maintainer (decisions, not bugs)

- **Split the PR?** Hub core (gated) vs. always-on MIDI/HID fixes (§4c) are
  logically independent. Merge together or separate?
- **`UHI_HUB_MAX == 1`** (no cascaded hubs) — agree this is the right budget
  trade for grid+kbd+MIDI, or worth the extra status pipe for a second tier?
- **MSC single-instance** behind a hub — fine (you never mount two sticks), or
  do you want it multi-instanced for completeness?
- **NVRAM shrink** (200K→199K) lives on the *teletype* side (patch/config.mk),
  not this repo — flagged here only so the reseed-on-first-boot cost is on your
  radar when you test.

---

## 7. Verification done

- **Builds:** flag-off and flag-on both compile+link clean via `dewb/monome-build`
  (and a native arm64 toolchain). Hub core adds ~+1.4 KB when on.
- **Hardware (verified):** grid + full-speed keyboard + Elektron Analog Rytm
  through a hub / Elektron Overhub, **any port order**, incl. MIDI hot-plug while
  the grid runs; low-speed device behind the hub rejected cleanly without
  disrupting other ports; nanoKONTROL2 (8-byte EP0) enumerates; USB-C docks
  (Genesys-Logic) use the USB-2 side; empty card-reader in a dock no longer
  hijacks the UI (MSC media gate, teletype side).
- **Not yet exercised on hardware:** two *different* EP0 sizes sharing pipe 0
  behind a hub simultaneously (see §5.1).

## 8. Known limitations (all documented in `USB_HUB_HOW_IT_WORKS.md`)

Low-speed devices behind a hub unsupported (hardware — incl. Apple aluminium
keyboards, whose matrix is low-speed behind their own hub); disconnect not
detected mid-enumeration; failed enumerations can leak a device struct;
3 simultaneous devices is the pipe ceiling; plug one at a time; powered hub
required.

---

## 9. How the 23 commits build up (suggested review order)

The history is roughly chronological-by-capability; reviewing in order tells the
story:

1. `832884a` — the base: hub driver + UHC/USBB hub support (the bulk of §4a/4b).
2. `e7c3e92`, `fd707fe` — MIDI-TX priority ring + CIN fix (§4c).
3. `2635fbd`, `2f20d6f`, `f6357b4` — the docs + teletype patch.
4. `fffd53d` — low-speed reject behind a hub.
5. `0d4ba95`, `ffb6370` — dock unplug teardown + duplicate-connect teardown.
6. `7971d51`, `141d47c`, `d7abd29` — NAK throttle (+ a debug facility since
   removed; net-zero, so don't hunt for `USB_TOPO_DEBUG` in the final tree).
7. `8bdc7b0`, `0c900d7` — hub-class hardening + review cleanup.
8. `6c7d96c` — make the flag an application opt-in (the gating in §4b).
9. `5153003` — `get_hub_by_dev` → `hub_find` + `hub_alloc` split.
10. `9e6c52a` — MIDI multi-device + name cache (§4c).
11. (`06918d3`, `fc31152`, `072d72d`, `5d7c4a9`, `134c485`, `5371713` —
    patch rebuilds + FAT warning fixes; low-risk.)

---

## 10. Pointers

- Design deep-dive: **`USB_HUB_HOW_IT_WORKS.md`**
- Retrospective / intent: **`HUB_RETRO.md`**
- Teletype-side wiring + build: **`TELETYPE_HUB_PATCH.md`**, `libavr32-teletype-hub.patch`
- Original scoping discussion: monome **aleph issue #67**
- Behavioral oracle for the hub algorithm: **TinyUSB `src/host/hub.c`** (read as
  spec; not linkable here)
