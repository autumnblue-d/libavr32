#include "cdc.h"

// std
#include <stdint.h>

// asf
#include "uhc.h"
#include "print_funcs.h"
#include "uhi_cdc.h"

// libavr32
#include "events.h"

#ifdef USB_TOPO_DEBUG
#include "usb_dbg.h"
#endif
#include "monome.h"

#define CDC_RX_BUF_SIZE 64

//----- static vars
static uint8_t rxBuf[CDC_RX_BUF_SIZE];
static uint32_t rxBytes = 0;
static uint8_t rxBusy = 0;
static uint8_t txBusy = 0;
static uint8_t connected = 0;
static event_t e;

extern uint8_t* cdc_rx_buf() { return rxBuf; }
extern volatile uint8_t cdc_rx_bytes() { return rxBytes; }
extern volatile uint8_t cdc_rx_busy() { return rxBusy; }
extern volatile uint8_t cdc_tx_busy() { return txBusy; }
extern uint8_t cdc_connected(void) { return connected; }

void cdc_tx(void);
void cdc_rx(void);

void cdc_change(uhc_device_t* dev, uint8_t plug) {
#ifdef USB_TOPO_DEBUG
  usb_dbg_push(plug ? "chg+" : "chg-");
#endif
  if(plug) {
    //print_dbg("\r\ncdc connected");
    connected = true;
    e.type = kEventSerialConnect;
  } else {
    //print_dbg("\r\ncdc disconnected");
    connected = false;
    e.type = kEventSerialDisconnect;
  }
  event_post(&e);
}

static void cdc_rx_done(usb_add_t add,
                         usb_ep_t ep,
                         uhd_trans_status_t stat,
                         iram_size_t nb) {
#ifdef USB_TOPO_DEBUG
  // Grid bulk-IN completion errors: "RXE <status>" first of each streak.
  // (Status 7 timeouts are the grid's normal idle-poll rhythm — benign.)
  static uint8_t rx_err;
  if (stat != UHD_TRANS_NOERROR) {
    if (!rx_err) usb_dbg_log_val("RXE", stat);
    rx_err = 1;
  }
  else rx_err = 0;
#endif
  rxBytes = nb;

  // FIXME: if the buffer is full, it's a false receive
  if (rxBytes != CDC_RX_BUF_SIZE) {
    if(monome_read_serial != NULL) {
      (*monome_read_serial)();
    }
    /*
    print_dbg("\r\nrx: ");
    for(int i=0;i<rxBytes;i++) {
      print_dbg_ulong(rxBuf[i]);
      print_dbg(" ");
    }*/
  }

  rxBusy = false;
}

static void cdc_tx_done(usb_add_t add,
                         usb_ep_t ep,
                         uhd_trans_status_t stat,
                         iram_size_t nb) {
#ifdef USB_TOPO_DEBUG
  // Grid bulk-OUT completions, unambiguous and continuous:
  //   "TXE <status> <count>" on the 1st and every 64th ERROR completion
  //   "TXok <count>"         on the 1st and every 64th CLEAN completion
  // A dark grid with TXok counting = writes reach the wire and are ACKed;
  // TXE counting = writes die (status 7 = timeout: transfer never ran).
  static uint16_t tx_err_count, tx_ok_count;
  if (stat != UHD_TRANS_NOERROR) {
    if ((tx_err_count++ & 0x3F) == 0) usb_dbg_log2("TXE", stat, tx_err_count);
  }
  else {
    if ((tx_ok_count++ & 0x3F) == 0) usb_dbg_log_val("TXok", tx_ok_count);
  }
#endif
  txBusy = false;

  // FIXME: bunch of these at startup
  /*if (stat != UHD_TRANS_NOERROR) {
    print_dbg("\r\ntx transfer callback error. status: 0x");
    print_dbg_hex((u32)stat);
  }*/
}



//-------- extern functions
#ifdef USB_TOPO_DEBUG
static uint8_t gt_busy_streak;
#endif

void cdc_write(uint8_t* data, uint32_t bytes) {
#ifdef USB_TOPO_DEBUG
  // Sparse write-attempt counter: "gW <n>" on the 1st and every 64th call.
  // If gW keeps counting while the grid is dark, writes ARE flowing (data
  // misrouted/ignored); if gW stops, the refresh chain upstream is dead.
  static uint16_t gw_count;
  if ((gw_count++ & 0x3F) == 0) usb_dbg_log_val("gW", gw_count - 1);
#endif
  if (txBusy == false) {
#ifdef USB_TOPO_DEBUG
    gt_busy_streak = 0;
#endif
    txBusy = true;
    if(!uhi_cdc_out_run(data, bytes, &cdc_tx_done)) {
      print_dbg("\r\ntx transfer error");
#ifdef USB_TOPO_DEBUG
      usb_dbg_push("gtF"); // uhd refused the bulk-OUT submit
#endif
      txBusy = false;
    }
  }
#ifdef USB_TOPO_DEBUG
  else {
    // Write dropped: a previous TX never completed (txBusy stuck). If this
    // appears right after "mrx"/grid setup, the setup write's completion
    // callback was swallowed and every LED frame is being discarded.
    if (!gt_busy_streak) usb_dbg_push("gtBZ");
    gt_busy_streak = 1;
  }
#endif
}

void cdc_read(void) {
  if (rxBusy == false) {
    rxBytes = 0;
    rxBusy = true;
    if (!uhi_cdc_in_run((uint8_t*)rxBuf,
                         CDC_RX_BUF_SIZE, &cdc_rx_done)) {
      print_dbg("\r\n rx transfer error");
      rxBusy = false;
    }
  }
}

