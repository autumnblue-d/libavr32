# Teletype USB-hub patch (`libavr32-teletype-hub.patch`)

This branch (`usb-dock-fixes`) carries the USB **hub class driver**
(`src/usb/hub/uhi_hub.{c,h}`), the hub-aware `uhc.c`, and the USBB **bulk NAK
throttle** (fair bus scheduling — without it an idle bulk-IN poll from one
device starves another device's transfers). The accompanying patch,
[`libavr32-teletype-hub.patch`](./libavr32-teletype-hub.patch), wires all of
that into a **vanilla monome/teletype `main`**.

## What the patch does (teletype side only)

Everything else lives on this libavr32 branch; the patch touches two files:

- **`module/config.mk`** — compiles `uhi_hub.c`, adds `../src/usb/hub` to the
  include path, and defines `USB_HOST_HUB_SUPPORT`. The define is an
  **application opt-in**: without it (and the other two lines) this same
  libavr32 branch builds the original single-device firmware unchanged.
- **`module/main.c`** — the three module-side USB behaviors that field
  debugging showed are needed alongside the driver:
  - **MSC media gate**: only enter the USB-disk dialog when a LUN reports
    ready media (`uhi_msc_mem_test_unit_ready`). An empty card reader —
    common in USB-C docks — otherwise enumerates as mass storage and hijacks
    the UI until unplugged. Note: the check runs once at connect; a card
    inserted later requires a replug.
  - **Monome poll gating**: no grid bulk traffic while any device is
    enumerating (`usb_enumeration_active`, set by `src/usb.c` on this
    branch). A pending grid IN does not reliably survive a concurrent heavy
    enumeration.
  - **Deferred monome setup**: the grid setup dialogue runs only after 50 ms
    of enumeration quiet, instead of immediately at serial-connect. Without
    this (and the NAK throttle), a grid on a lower hub port than a composite
    MIDI device (e.g. Elektron Analog Rytm) stays dark.

## Prerequisite

The Teletype `libavr32` submodule must be on **this** branch
(`usb-dock-fixes`), which provides `src/usb/hub/uhi_hub.{c,h}`. On stock
`monome/libavr32` (main) the patch enables includes and sources that do not
exist and will not build.

## Apply (run from the **Teletype repo root**)

```sh
# libavr32 submodule already on usb-dock-fixes (this branch)
git apply --check libavr32/libavr32-teletype-hub.patch   # optional dry-run
git apply         libavr32/libavr32-teletype-hub.patch
```

The patch's own header (comment block before the first `diff`) repeats these
steps.

## Build (AVR32 toolchain via Docker)

```sh
docker run --rm --platform linux/amd64 -v "$(pwd)":/target \
    dewb/monome-build 'cd module && make clean && make'
# -> module/teletype.{elf,hex,bin}
```

## Verified

Applies cleanly (`git apply --check`) and builds on a pristine
`monome/teletype` main (`e390bac`) + this branch (`49ed16a` era), in both
configurations: patch applied (hub support on) and unpatched (original
single-device stack from the same branch). Hardware confirmation on the full
setup (this branch + the maintainer's teletype branch): grid + full-speed
keyboard + Elektron Analog Rytm work through an Overhub **in any port
order**, including MIDI hot-plug while the grid runs; a low-speed device
behind the hub is rejected cleanly without disrupting other ports.

## Not included

- The `USB_TOPO_DEBUG` on-screen trace facility: the libavr32 half is on this
  branch (compile-gated, inert), but the module-side OLED overlay/key
  bindings live only on the maintainer's teletype branch (`usb-dock-fixes`
  there; see its `USB_DOCK_NOTES.md` for the trace legend).
- The `trilogy`-branch `flash.c` reseed fixes — specific to that branch's
  enlarged flash layout.
