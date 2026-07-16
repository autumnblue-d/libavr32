/*
  usb_dbg.c

  USB enumeration/topology trace ring for debugging device bring-up on the
  module's OLED. See usb_dbg.h for the producer/consumer map.

  Inert unless USB_TOPO_DEBUG is defined (same pattern as uhi_hub.c), so the
  file can live in the build (config.mk) without costing flash in normal
  builds.
*/

#include "usb_dbg.h"

#ifdef USB_TOPO_DEBUG

#include <string.h>
#include "util.h" // itoa

static char ring[USB_DBG_LINES][USB_DBG_COLS];
static uint8_t ring_head;  // next slot to overwrite
static uint8_t ring_count; // valid lines (saturates at USB_DBG_LINES)
static volatile uint8_t ring_seq;

// Pushes run in USB interrupt context, the renderer in the main loop. No
// locking: a torn read garbles at worst one on-screen line of a debug build.

void usb_dbg_push(const char* s) {
	strncpy(ring[ring_head], s, USB_DBG_COLS - 1);
	ring[ring_head][USB_DBG_COLS - 1] = '\0';
	ring_head = (uint8_t)((ring_head + 1) % USB_DBG_LINES);
	if (ring_count < USB_DBG_LINES) ring_count++;
	ring_seq++;
}

// bounded strcat: newlib's strncat would need the remaining-space math anyway
static void cat(char* dst, const char* src) {
	uint8_t len = (uint8_t)strlen(dst);
	while (*src && len < USB_DBG_COLS - 1) dst[len++] = *src++;
	dst[len] = '\0';
}

static void cat_num(char* dst, int32_t v) {
	char num[12];
	itoa((int)v, num, 10);
	cat(dst, num);
}

void usb_dbg_log_val(const char* tag, int32_t v) {
	char b[USB_DBG_COLS];
	b[0] = '\0';
	cat(b, tag);
	cat(b, " ");
	cat_num(b, v);
	usb_dbg_push(b);
}

void usb_dbg_log2(const char* tag, int32_t a, int32_t b) {
	char buf[USB_DBG_COLS];
	buf[0] = '\0';
	cat(buf, tag);
	cat(buf, " ");
	cat_num(buf, a);
	cat(buf, " ");
	cat_num(buf, b);
	usb_dbg_push(buf);
}

void usb_dbg_log_step(uint8_t step, int32_t status, uint16_t n) {
	char b[USB_DBG_COLS];
	b[0] = '\0';
	cat(b, "s");
	cat_num(b, step);
	cat(b, " t");
	cat_num(b, status);
	cat(b, " n");
	cat_num(b, n);
	usb_dbg_push(b);
}

uint8_t usb_dbg_seq(void) {
	return ring_seq;
}

uint8_t usb_dbg_count(void) {
	return ring_count;
}

const char* usb_dbg_line(uint8_t i) {
	uint8_t idx = (uint8_t)((ring_head + USB_DBG_LINES - ring_count + i) %
	                        USB_DBG_LINES);
	return ring[idx];
}

void usb_dbg_log_pipe(uint8_t p, uint8_t addr, uint8_t ep, uint8_t flags) {
	char b[USB_DBG_COLS];
	b[0] = '\0';
	cat(b, "p");
	cat_num(b, p);
	cat(b, " a");
	cat_num(b, addr);
	cat(b, " e");
	cat_num(b, ep);
	cat(b, " ");
	if (flags & 1) cat(b, "E");
	if (flags & 2) cat(b, "F");
	if (flags & 4) cat(b, "C");
	usb_dbg_push(b);
}

//------ IRQ-storm detector

// 100k ticks between main-loop calms trips the alarm. Normal worst case is a
// few k per 63 ms refresh period; a storm at raw IRQ rate trips in ~0.1 s.
#define USB_DBG_STORM_TICKS 100000UL

static volatile uint32_t irq_ticks;

void usb_dbg_irq_calm(void) {
	irq_ticks = 0;
}

void usb_dbg_irq_tick(uint32_t diag0, uint32_t diag1) {
	if (++irq_ticks != USB_DBG_STORM_TICKS) return; // == so it fires ONCE

	char b[USB_DBG_COLS];
	char num[12];
	b[0] = '\0';
	cat(b, "STORM ");
	itoa((int)diag0, num, 16);
	cat(b, num);
	cat(b, " ");
	itoa((int)diag1, num, 16);
	cat(b, num);
	usb_dbg_push(b);           // in case the main loop ever breathes again
	usb_dbg_emergency_render(b); // ...but don't count on it
}

// Overridden by the module (module/main.c) with a real OLED line draw.
__attribute__((weak)) void usb_dbg_emergency_render(const char* s) {
	(void)s;
}

#endif // USB_TOPO_DEBUG
