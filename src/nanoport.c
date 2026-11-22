/*
  nanoport.c - network bridge for the NanoMig nanoport network card

  The core exposes a Zorro II network card to the Amiga (SANA-II driver
  nanoport.device). Its frame buffers are reached through SPI target 5
  (see misc/nanoport.v in the NanoMig core). This file moves the frames
  between the card and the Companion's own network interface, which is
  either WiFi (Pico W) or a USB ethernet adapter (ASIX).

  A WiFi client can only send frames with its own MAC address. The Amiga
  therefore uses its own (virtual) MAC address on the card side and the
  bridge translates between that and the MAC of the Companion's interface:

  - Amiga -> network: the source MAC (and the ARP sender MAC) is replaced
    by the interface MAC. DHCP requests get the broadcast flag, so the
    server answers by broadcast and the lease is bound to the Amiga's
    own MAC in the DHCP payload (chaddr).
  - network -> Amiga: broadcasts and multicasts go to both, the Amiga and
    the Companion's lwIP stack. Unicast IPv4 and ARP for the Amiga's IP
    address (learned from its outgoing frames) as well as DHCP replies for
    the Amiga's MAC only go to the Amiga, with the destination MAC (and
    the ARP target MAC) set back to the Amiga's MAC.
  - Amiga <-> Companion: frames between the Amiga and the Companion itself
    (e.g. FTP or telnet to the Companion) are looped back locally.

  The Amiga thus is a regular host with its own IP address in the local
  network (static or via DHCP). Only IPv4 and ARP are bridged.
*/

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/opt.h"           // TCPIP_THREAD_PRIO
#include "pico/cyw43_arch.h"   // cyw43_arch_lwip_begin/end, also used for usb ethernet

#include "nanoport.h"
#include "mcu_hw.h"
#include "spi.h"
#include "debug.h"

#define np_debugf(a, ...)  debugf("\033[0;35mNET: " a "\033[0m", ##__VA_ARGS__)

#define NP_MAX_FRAME   1518   // without FCS
#define NP_MIN_FRAME   14
// frames buffered for the Amiga (power of two), taken from the FreeRTOS heap
#if PICO_RP2350
#define NP_RXQ         8
#else
#define NP_RXQ         4      // the RP2040 is short of ram
#endif

// core status flags (sub command 0, first byte)
#define NP_ST_MAGIC    0xa0
#define NP_ST_ONLINE   0x01
#define NP_ST_TXPEND   0x04
#define NP_ST_RXFULL   0x08

static TaskHandle_t np_task_handle = NULL;

// the hooked network interface
static struct netif *np_netif = NULL;
static netif_input_fn np_orig_input = NULL;
static netif_linkoutput_fn np_orig_linkoutput = NULL;

static volatile bool np_online = false;     // driver has the card online
static uint8_t np_hmac[6];                  // MAC of the Companion's interface
static uint8_t np_vmac[6];                  // MAC of the Amiga
static volatile uint32_t np_amiga_ip = 0;   // learned, network byte order

// frames for the Amiga, single consumer (nanoport task). There are two
// producers, the interface driver (input) and lwIP (linkoutput), running in
// different tasks: np_rxq_lock serializes them.
struct np_frame {
  uint16_t len;
  uint8_t data[NP_MAX_FRAME];
};
static struct np_frame *np_rxq = NULL;
static volatile uint32_t np_rxq_wr = 0, np_rxq_rd = 0;
static SemaphoreHandle_t np_rxq_lock = NULL;

static uint8_t *np_txbuf = NULL;

static struct {
  uint32_t tx, rx, rx_drop, loop;
} np_stats;

/* ------------------------------------------------------------------ helpers */

static inline uint16_t get16(const uint8_t *p) { return (p[0] << 8) | p[1]; }
static inline uint32_t get32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }

#define ETH_TYPE(f)    get16((f)+12)
#define NP_IPHLEN(f)     (((f)[14] & 0x0f) * 4)

// offset of the DHCP (BOOTP) payload or 0 if this is no DHCP frame to <port>
static int np_dhcp_offset(const uint8_t *f, int len, uint16_t port) {
  if(len < 34 || ETH_TYPE(f) != 0x0800 || f[23] != 17) return 0;
  int udp = 14 + NP_IPHLEN(f);
  if(len < udp + 8 + 44 || get16(f+udp+2) != port) return 0;
  return udp + 8;
}

/* ------------------------------------------------------------------ to the Amiga */

static void np_queue(struct pbuf *p, int len, bool rewrite) {
  xSemaphoreTake(np_rxq_lock, portMAX_DELAY);
  if(np_rxq_wr - np_rxq_rd >= NP_RXQ) {
    np_stats.rx_drop++;
    xSemaphoreGive(np_rxq_lock);
    return;
  }

  uint8_t *d = np_rxq[np_rxq_wr & (NP_RXQ-1)].data;
  pbuf_copy_partial(p, d, len, 0);

  if(rewrite) {
    memcpy(d, np_vmac, 6);
    if(ETH_TYPE(d) == 0x0806 && len >= 42 && !memcmp(d+32, np_hmac, 6))
      memcpy(d+32, np_vmac, 6);
  }

  np_rxq[np_rxq_wr & (NP_RXQ-1)].len = len;
  np_rxq_wr++;
  xSemaphoreGive(np_rxq_lock);

  if(np_task_handle) xTaskNotifyGive(np_task_handle);
}

// 0 = Companion only, 1 = Amiga only, 2 = both
static int np_classify_in(const uint8_t *f, int len) {
  if(f[0] & 1) return 2;                                 // broadcast / multicast
  if(memcmp(f, np_hmac, 6)) return 0;

  uint16_t type = ETH_TYPE(f);
  if(type == 0x0806 && len >= 42)
    return (np_amiga_ip && get32(f+38) == np_amiga_ip) ? 1 : 0;

  if(type == 0x0800 && len >= 34) {
    if(np_amiga_ip && get32(f+30) == np_amiga_ip) return 1;
    int dhcp = np_dhcp_offset(f, len, 68);
    if(dhcp && !memcmp(f+dhcp+28, np_vmac, 6)) return 1;
  }
  return 0;
}

// replaces netif->input: frames received from the network
static err_t np_netif_input(struct pbuf *p, struct netif *netif) {
  if(np_online && p->tot_len >= NP_MIN_FRAME && p->tot_len <= NP_MAX_FRAME) {
    uint8_t h[96];
    int n = pbuf_copy_partial(p, h, sizeof(h), 0);
    int dest = np_classify_in(h, n < p->tot_len ? n : p->tot_len);
    if(dest) {
      np_queue(p, p->tot_len, dest == 1);
      if(dest == 1) {
	pbuf_free(p);
	return ERR_OK;
      }
    }
  }
  return np_orig_input(p, netif);
}

// replaces netif->linkoutput: frames sent by the Companion's own stack
static err_t np_netif_linkoutput(struct netif *netif, struct pbuf *p) {
  if(np_online && p->tot_len >= NP_MIN_FRAME && p->tot_len <= NP_MAX_FRAME) {
    uint8_t h[6];
    pbuf_copy_partial(p, h, 6, 0);

    // addressed to the Amiga: deliver locally only
    if(!memcmp(h, np_vmac, 6)) {
      np_stats.loop++;
      np_queue(p, p->tot_len, false);
      return ERR_OK;
    }
    // the Amiga doesn't hear our own broadcasts via the access point
    if(h[0] & 1) np_queue(p, p->tot_len, false);
  }
  return np_orig_linkoutput(netif, p);
}

/* ------------------------------------------------------------------ from the Amiga */

// hand an unmodified frame of the Amiga to the Companion's own stack
static void np_loopback(const uint8_t *f, int len) {
  struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
  if(!p) return;
  pbuf_take(p, f, len);
  if(np_orig_input(p, np_netif) != ERR_OK) pbuf_free(p);
}

static void np_send(uint8_t *f, int len) {
  np_stats.tx++;

  if(!(f[6] & 1)) memcpy(np_vmac, f+6, 6);   // follow the station address of the driver

  uint16_t type = ETH_TYPE(f);
  if(type != 0x0800 && type != 0x0806) return;   // IPv4 and ARP only

  // learn the Amiga's IP address
  uint32_t ip = 0;
  if(type == 0x0806 && len >= 42) ip = get32(f+28);
  if(type == 0x0800 && len >= 34) ip = get32(f+26);
  if(ip && ip != 0xffffffff && ip != np_amiga_ip) {
    np_amiga_ip = ip;
    np_debugf("Amiga IP is %d.%d.%d.%d", (int)(ip & 0xff), (int)((ip >> 8) & 0xff),
	      (int)((ip >> 16) & 0xff), (int)(ip >> 24));
  }

  cyw43_arch_lwip_begin();

  // to the Companion itself
  if(!memcmp(f, np_hmac, 6)) {
    np_stats.loop++;
    np_loopback(f, len);
    cyw43_arch_lwip_end();
    return;
  }
  if(f[0] & 1) np_loopback(f, len);

  // MAC translation
  memcpy(f+6, np_hmac, 6);
  if(type == 0x0806 && len >= 42 && !memcmp(f+22, np_vmac, 6))
    memcpy(f+22, np_hmac, 6);

  int dhcp = np_dhcp_offset(f, len, 67);
  if(dhcp) {
    f[dhcp+10] |= 0x80;                 // broadcast flag
    int udp = 14 + NP_IPHLEN(f);
    f[udp+6] = f[udp+7] = 0;            // no UDP checksum (allowed for IPv4)
  }

  struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_RAM);
  if(p) {
    pbuf_take(p, f, len);
    np_orig_linkoutput(np_netif, p);
    pbuf_free(p);
  }
  cyw43_arch_lwip_end();
}

/* ------------------------------------------------------------------ SPI */

static void np_begin(uint8_t cmd) {
  mcu_hw_spi_begin();
  mcu_hw_spi_tx_u08(SPI_TARGET_NET);
  mcu_hw_spi_tx_u08(cmd);
}

// returns false if the core has no nanoport
static bool np_status(uint8_t *st) {
  np_begin(SPI_NET_STATUS);
  for(int i=0;i<4;i++) st[i] = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();
  return (st[0] & 0xf0) == NP_ST_MAGIC;
}

static void np_set_info(bool link) {
  np_begin(SPI_NET_INFO);
  mcu_hw_spi_tx_u08(link ? 1 : 0);
  for(int i=0;i<6;i++) mcu_hw_spi_tx_u08(np_hmac[i] ^ (i == 0 ? 0x02 : 0));   // default Amiga MAC
  mcu_hw_spi_end();
}

static int np_tx_get(uint8_t *buf) {
  uint8_t l[2];
  np_begin(SPI_NET_TX_GET);
  mcu_hw_spi_rx_buffer(l, 2);
  int len = (l[0] << 8) | l[1];
  if(len >= NP_MIN_FRAME && len <= NP_MAX_FRAME) mcu_hw_spi_rx_buffer(buf, len);
  else len = 0;
  mcu_hw_spi_end();

  np_begin(SPI_NET_TX_DONE);
  mcu_hw_spi_end();
  return len;
}

static void np_rx_put(const uint8_t *buf, int len) {
  np_begin(SPI_NET_RX_PUT);
  mcu_hw_spi_tx_u08(len >> 8);
  mcu_hw_spi_tx_u08(len & 0xff);
  mcu_hw_spi_tx_buffer(buf, len);
  mcu_hw_spi_end();
}

/* ------------------------------------------------------------------ interface hook */

static void np_unhook(void) {
  if(!np_netif) return;
  cyw43_arch_lwip_begin();
  np_netif->input = np_orig_input;
  np_netif->linkoutput = np_orig_linkoutput;
  cyw43_arch_lwip_end();
  np_netif = NULL;
}

// follow the active interface (WiFi or USB ethernet), returns true on change
static bool np_hook(void) {
  struct netif *n = netif_default;
  bool up = n && netif_is_up(n) && netif_is_link_up(n) && n->hwaddr_len == 6 &&
    n->name[0] != 'p';   // not on ppp

  if(np_netif && (!up || n != np_netif)) {
    np_debugf("detached from %c%c", np_netif->name[0], np_netif->name[1]);
    np_unhook();
    return true;
  }
  if(!up || np_netif) return false;

  cyw43_arch_lwip_begin();
  np_orig_input = n->input;
  np_orig_linkoutput = n->linkoutput;
  memcpy(np_hmac, n->hwaddr, 6);
  memcpy(np_vmac, n->hwaddr, 6);
  np_vmac[0] ^= 0x02;              // locally administered variant of the interface MAC
  n->input = np_netif_input;
  n->linkoutput = np_netif_linkoutput;
  np_netif = n;
  cyw43_arch_lwip_end();

  np_debugf("attached to %c%c", n->name[0], n->name[1]);
  return true;
}

/* ------------------------------------------------------------------ task */

static void np_task(__attribute__((unused)) void *parms) {
  uint8_t st[4];
  bool present = false;

  for(;;) {
    // wait for the core (interrupt) or poll: frames waiting for a free slot
    // are retried after 2ms, everything else is checked every 500ms
    bool waiting = np_rxq_wr != np_rxq_rd;
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(waiting ? 2 : 500));

    if(!np_status(st)) {
      if(present) np_debugf("core has no nanoport");
      present = false;
      np_online = false;
      np_rxq_rd = np_rxq_wr;
      continue;
    }

    if(np_hook() || !present) np_set_info(np_netif != NULL);
    if(!present) np_debugf("nanoport core detected");
    present = true;

    bool online = (st[0] & NP_ST_ONLINE) && np_netif;
    if(online != np_online) {
      np_debugf("Amiga %s", online ? "online" : "offline");
      np_online = online;
      if(!online) np_rxq_rd = np_rxq_wr;
    }

    // move frames until there is nothing left to do
    bool busy = true;
    while(busy) {
      busy = false;

      int tx_pending = st[2] & 0x0f;
      while(tx_pending--) {
	int len = np_tx_get(np_txbuf);
	if(len && np_online) np_send(np_txbuf, len);
	busy = true;
      }

      int rx_free = st[1] & 0x0f;
      while(rx_free && np_rxq_wr != np_rxq_rd) {
	uint32_t i = np_rxq_rd & (NP_RXQ-1);
	if(np_online) {
	  np_rx_put(np_rxq[i].data, np_rxq[i].len);
	  np_stats.rx++;
	  rx_free--;
	}
	np_rxq_rd++;
	busy = true;
      }

      if(busy && !np_status(st)) break;
    }
  }
}

void nanoport_handle_event(void) {
  if(np_task_handle) xTaskNotifyGive(np_task_handle);
}

void nanoport_init(void) {
  np_rxq = pvPortMalloc(NP_RXQ * sizeof(struct np_frame));
  np_txbuf = pvPortMalloc(NP_MAX_FRAME);
  np_rxq_lock = xSemaphoreCreateMutex();
  if(!np_rxq || !np_txbuf || !np_rxq_lock) {
    np_debugf("not enough memory for the frame buffers");
    if(np_rxq) vPortFree(np_rxq);
    if(np_txbuf) vPortFree(np_txbuf);
    if(np_rxq_lock) vSemaphoreDelete(np_rxq_lock);
    return;
  }
  // Just above the lwIP and cyw43 tasks: a frame is moved as soon as the core
  // signals it (same priority meant waiting for the next tick). The network
  // stack isn't starved, the task sleeps once both rings are empty and the
  // RX side only gets work while the cyw43/USB tasks run. Earlier versions
  // ran at configMAX_PRIORITIES-4, far above everything network related.
  // The stack is large enough for frames looped back into lwIP.
  xTaskCreate(np_task, "nanoport", 2048, NULL, TCPIP_THREAD_PRIO + 1, &np_task_handle);
}
