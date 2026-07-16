#ifndef _USB_DBG_H_
#define _USB_DBG_H_

#include <stdint.h>

// Tiny USB enumeration/topology trace ring, rendered on the module's OLED
// (installed hardware has no other output channel). Inert unless
// USB_TOPO_DEBUG is defined — build the module with `make USB_TOPO_DEBUG=1`.
//
// Producers push short lines from USB interrupt context:
//   uhc.c     — per-step enumeration transfer status ("s13 t2 n9"),
//               enumeration errors ("ERR <status> <try>"),
//               hub port changes ("P+ <port>" / "P- <port>")
//   usb.c     — end-of-enumeration events ("E <addr> <status>")
//   uhi_hub.c — downstream port-reset timeout ("hub rstTO")
// The module's screen-refresh handler renders the ring (see module/main.c).

#define USB_DBG_LINES 8
#define USB_DBG_COLS 32 // incl. NUL; 128 px / 4 px font = 32 visible chars

// -- producers (USB interrupt context) --
void usb_dbg_push(const char* s);
void usb_dbg_log_val(const char* tag, int32_t v);         // "tag v"
void usb_dbg_log2(const char* tag, int32_t a, int32_t b); // "tag a b"
void usb_dbg_log_step(uint8_t step, int32_t status, uint16_t n); // "sN tS nL"

// -- renderer (main loop) --
uint8_t usb_dbg_seq(void);   // bumped on every push; cheap change detection
uint8_t usb_dbg_count(void); // valid lines, saturates at USB_DBG_LINES
const char* usb_dbg_line(uint8_t i); // 0 = oldest retained .. count-1 = newest

// -- pipe-table dump --
// usb_dbg_log_pipe: one ring line per pipe, "p<N> a<addr> e<ep> [E][F][C]"
// (E = enabled, F = frozen, C = config-OK). uhd_dbg_dump_pipes() in
// usbb_host.c walks pipes 0..6; the module triggers it with ALT+F9.
void usb_dbg_log_pipe(uint8_t p, uint8_t addr, uint8_t ep, uint8_t flags);
void uhd_dbg_dump_pipes(void);

// -- IRQ-storm detector --
// usb_dbg_irq_tick(diag0, diag1) at USB ISR entry; usb_dbg_irq_calm() from the
// main loop (screen-refresh handler). If the ISR keeps firing while the main
// loop never breathes, the tick counter hits the threshold and "STORM
// <diag0> <diag1>" (hex) is drawn DIRECTLY from interrupt context via
// usb_dbg_emergency_render() — the normal trace render path is dead by then.
// Pass the USBB UHINT/UHINTE registers as diags: the set bit names the stuck
// interrupt (bit 8+p = pipe p; bit 5 = SOF).
//
// Timing signature: a real storm trips in ~0.1 s of freeze; a dead main loop
// with a HEALTHY USB (SOF-only, ~1 kHz) trips after ~100 s and shows only the
// SOF bit — both outcomes are informative.
void usb_dbg_irq_tick(uint32_t diag0, uint32_t diag1);
void usb_dbg_irq_calm(void);
// module-provided override (weak no-op default): draw one line, IRQ context
void usb_dbg_emergency_render(const char* s);

#endif // _USB_DBG_H_
