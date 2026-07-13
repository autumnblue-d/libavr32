# Teletype USB-hub patch (`libavr32-teletype-hub.patch`)

This branch (`hub-snapshot`) carries the USB **hub class driver**
(`src/usb/hub/uhi_hub.{c,h}`). The accompanying patch,
[`libavr32-teletype-hub.patch`](./libavr32-teletype-hub.patch), wires that driver
into a **vanilla Teletype `main`** and adds the low-speed-behind-hub handling.

## What the patch does

- **Teletype side** — `module/config.mk`: compiles `uhi_hub.c` and adds
  `../src/usb/hub` to the include path.
- **libavr32 side**:
  - `asf/.../uhc/uhc.c` — a hub-attached device is enumerated at the speed the
    **hub** reports for its port (not the bus-wide `uhd_get_speed()`); a
    **low-speed** device behind a hub is rejected cleanly (the UC3B USBB host
    cannot drive low-speed through a hub — see `USB_HUB_HOW_IT_WORKS.md`).
  - `src/usb/hub/uhi_hub.c/h` — per-port speed capture + a `rejected` port mask
    so a rejected device is not re-enumerated in a loop.
  - `USB_HUB_HOW_IT_WORKS.md` — documents the hardware limitation (AVR4950 cite).

There is **no on-screen debug code** in the patch.

## Prerequisite

The Teletype `libavr32` submodule must be on **this** branch (`hub-snapshot`,
`b11e173`), which provides `src/usb/hub/uhi_hub.{c,h}`. On stock
`monome/libavr32` (main) the patch references files that do not exist and will
not build.

## Apply (run from the **Teletype repo root**)

```sh
# libavr32 submodule already on hub-snapshot (this branch)
git apply --check libavr32/libavr32-teletype-hub.patch   # optional dry-run
git apply         libavr32/libavr32-teletype-hub.patch
```

The patch's own header (comment block before the first `diff`) repeats these
steps, including how to put the submodule on `hub-snapshot` from scratch.

## Build (AVR32 toolchain via Docker)

```sh
docker run --rm --platform linux/amd64 -v "$(pwd)":/target \
    dewb/monome-build 'cd module && make clean && make'
# -> module/teletype.{elf,hex,bin}
```

## Verified

Applies cleanly and builds on a pristine `monome/teletype` main +
`hub-snapshot` `b11e173` (clean-room test). Hardware confirmation: grid and
full-speed keyboard work through a hub; a low-speed keyboard (e.g. Apple
aluminium) behind its hub is ignored cleanly without disrupting other ports.

## Not included

The `trilogy`-branch `flash.c` reseed fixes are **not** part of this patch —
they are specific to that branch's enlarged flash layout (Kria/MP/ES banks).
