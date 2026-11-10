/*
  a314_port.c

  A314 (clockport variant) over SPI target 6 of the core, see a314_cp.v / a314_host.v: the
  Companion plays the Raspberry Pi of the A314. The Amiga runs a314.device and a314eth.device
  (SANA-II) and talks to the "ethernet" service, which is implemented here (a314_proto.c); the
  frames are an IP interface of the Companion like plip_port.c, with the same peer forwarding
  of ppp_port.c.

  SPI target 6, bytes (the reply to a byte is the answer to the byte before it):
    6, 0 STATUS    status {pi_irq}
    6, 1 READ      addr hi, addr lo, 0, then data bytes
    6, 2 WRITE     addr hi, addr lo, then data bytes
    6, 3 IRQ       value for REG_IRQ of the Pi side (bit 7 = set, bits 1:0 = bits)
    6, 4 ID        status, 0xA3, version, log2 of the RAM size
  The SPI bytes are sent one call at a time, a burst without a gap would be too fast for the core.

  config.ini:  [A314]  ENABLE=1   switch it on (default 0)
                       IP=...     address of the Companion on this link, default 192.168.253.1
                       PEER=...   address of the Amiga, default 192.168.253.2
*/

#include "a314_port.h"
#include "../a314_proto.h"
#include "ppp_port.h"

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include "lwip/tcpip.h"
#include "lwip/tcp.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/netif.h"
#include "lwip/ip4.h"
#include "lwip/prot/ethernet.h"
#include "lwip/prot/etharp.h"
#include "lwip/prot/iana.h"

#include "../spi.h"
#include "../mcu_hw.h"
#include "../inifile.h"
#include "../debug.h"

#define FRAME_MAX   1514
#define A314_MTU    1500
#define TX_SLOTS    3
#define POLL_MS     50       // the interrupt is the normal trigger, this catches a missed one
#define FORCE_MS    1000     // service the protocol even without the interrupt bit
#define STATS_MS    30000

static struct netif a314_netif;
static TaskHandle_t task;
static struct a314 proto;

static const uint8_t our_mac[6] = { 0x02, 'A', '3', '1', '4', 0x01 };   // locally administered
static uint8_t peer_mac[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };    // until the Amiga has sent something

static ip4_addr_t our_ip, peer_ip;

// transmit queue, free running counters. tx_wr is only written under the core lock, tx_rd by the task
static struct { uint8_t data[FRAME_MAX] __attribute__((aligned(4))); uint16_t len; } txq[TX_SLOTS];
static uint32_t tx_wr, tx_rd;
static volatile bool tx_blocked;        // a frame was refused, tcp has to be told when there is room
static volatile uint32_t tx_drops;

static uint8_t rxbuf[FRAME_MAX] __attribute__((aligned(4)));
static uint32_t n_rx, n_tx, n_rx_bad;

/* ---- SPI ---- */

static void a_begin(uint8_t cmd) {
  mcu_hw_spi_begin();
  mcu_hw_spi_tx_u08(SPI_TARGET_A314);
  mcu_hw_spi_tx_u08(cmd);
}

static uint8_t a_status(void) {
  a_begin(SPI_A314_STATUS);
  uint8_t st = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();
  return st;
}

static void a_mem_read(void *ctx, uint16_t addr, uint8_t *dst, uint16_t len) {
  (void)ctx;
  a_begin(SPI_A314_READ);
  mcu_hw_spi_tx_u08(addr >> 8);
  mcu_hw_spi_tx_u08(addr & 0xff);
  mcu_hw_spi_tx_u08(0);
  for(uint16_t i = 0; i < len; i++) dst[i] = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();
}

static void a_mem_write(void *ctx, uint16_t addr, const uint8_t *src, uint16_t len) {
  (void)ctx;
  a_begin(SPI_A314_WRITE);
  mcu_hw_spi_tx_u08(addr >> 8);
  mcu_hw_spi_tx_u08(addr & 0xff);
  for(uint16_t i = 0; i < len; i++) mcu_hw_spi_tx_u08(src[i]);
  mcu_hw_spi_end();
}

static void a_irq_write(void *ctx, uint8_t val) {
  (void)ctx;
  a_begin(SPI_A314_IRQ);
  mcu_hw_spi_tx_u08(val);
  mcu_hw_spi_end();
}

// 0 if there is no A314 interface in the core, else log2 of the RAM size
static int a_id(void) {
  a_begin(SPI_A314_ID);
  mcu_hw_spi_tx_u08(0);                      // status
  uint8_t magic = mcu_hw_spi_tx_u08(0);
  uint8_t ver = mcu_hw_spi_tx_u08(0);
  uint8_t aw = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();
  return (magic == 0xa3 && ver >= 1 && aw >= 10 && aw <= 16) ? aw : 0;
}

static void frame_input(uint16_t len);

static void a_frame_from_amiga(void *ctx, const uint8_t *f, uint16_t len) {
  (void)ctx;
  if(len > FRAME_MAX) { n_rx_bad++; return; }
  memcpy(rxbuf, f, len);
  frame_input(len);
}

/* ---- lwIP side ---- */

// netif->output, called by lwIP (tcpip thread, core lock held): never wait here
static err_t a314_output(struct netif *netif, struct pbuf *p, const ip4_addr_t *ipaddr) {
  (void)netif; (void)ipaddr;

  if(p->tot_len > A314_MTU) return ERR_MEM;

  uint32_t wr = tx_wr;
  if(wr - __atomic_load_n(&tx_rd, __ATOMIC_ACQUIRE) >= TX_SLOTS) {
    tx_drops++;
    tx_blocked = true;
    return ERR_MEM;           // tcp keeps the segment queued, other packets are lost
  }

  uint8_t *f = txq[wr % TX_SLOTS].data;
  memcpy(f, peer_mac, 6);
  memcpy(f + 6, our_mac, 6);
  f[12] = ETHTYPE_IP >> 8;
  f[13] = ETHTYPE_IP & 0xff;
  pbuf_copy_partial(p, f + 14, p->tot_len, 0);
  txq[wr % TX_SLOTS].len = 14 + p->tot_len;

  __atomic_store_n(&tx_wr, wr + 1, __ATOMIC_RELEASE);
  if(task) xTaskNotifyGive(task);
  return ERR_OK;
}

static err_t a314_netif_init(struct netif *netif) {
  netif->name[0] = 'a';
  netif->name[1] = '3';
  netif->output = a314_output;
  netif->mtu = A314_MTU;
  netif->hwaddr_len = 6;
  memcpy(netif->hwaddr, our_mac, 6);
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;
  return ERR_OK;
}

// tcpip thread: segments that a314_output() refused are sent now
static void tx_resume(void *arg) {
  (void)arg;
  tcp_txnow();
}

// ARP request of the Amiga: we are the answer for every address except its own
static void arp_input(const uint8_t *f, uint16_t len) {
  if(len < 14 + SIZEOF_ETHARP_HDR) return;
  const struct etharp_hdr *arp = (const struct etharp_hdr *)(f + 14);
  if(arp->opcode != PP_HTONS(ARP_REQUEST) || arp->hwlen != 6 || arp->protolen != 4) return;

  ip4_addr_t sip, dip;
  memcpy(&sip, &arp->sipaddr, 4);
  memcpy(&dip, &arp->dipaddr, 4);
  if(ip4_addr_isany_val(sip) || ip4_addr_cmp(&dip, &peer_ip)) return;   // probe or its own address

  uint8_t *r = txq[tx_wr % TX_SLOTS].data;
  if(tx_wr - __atomic_load_n(&tx_rd, __ATOMIC_ACQUIRE) >= TX_SLOTS) return;

  memcpy(r, arp->shwaddr.addr, 6);
  memcpy(r + 6, our_mac, 6);
  r[12] = ETHTYPE_ARP >> 8;
  r[13] = ETHTYPE_ARP & 0xff;
  struct etharp_hdr *a = (struct etharp_hdr *)(r + 14);
  a->hwtype   = PP_HTONS(LWIP_IANA_HWTYPE_ETHERNET);
  a->proto    = PP_HTONS(ETHTYPE_IP);
  a->hwlen    = 6;
  a->protolen = 4;
  a->opcode   = PP_HTONS(ARP_REPLY);
  memcpy(a->shwaddr.addr, our_mac, 6);
  memcpy(&a->sipaddr, &dip, 4);
  memcpy(a->dhwaddr.addr, arp->shwaddr.addr, 6);
  memcpy(&a->dipaddr, &sip, 4);
  txq[tx_wr % TX_SLOTS].len = 14 + SIZEOF_ETHARP_HDR;
  __atomic_store_n(&tx_wr, tx_wr + 1, __ATOMIC_RELEASE);
}

// frame of the Amiga in rxbuf. a314_output() writes tx_wr under the core lock, so an ARP
// reply that is queued from here needs the core lock as well
static void frame_input(uint16_t len) {
  if(len < 14 || len > FRAME_MAX) { n_rx_bad++; return; }
  n_rx++;

  uint16_t type = (rxbuf[12] << 8) | rxbuf[13];
  if(type == ETHTYPE_ARP) {
    memcpy(peer_mac, rxbuf + 6, 6);
    LOCK_TCPIP_CORE();
    arp_input(rxbuf, len);
    UNLOCK_TCPIP_CORE();
  } else if(type == ETHTYPE_IP) {
    memcpy(peer_mac, rxbuf + 6, 6);
    struct pbuf *p = pbuf_alloc(PBUF_RAW, len - 14, PBUF_POOL);
    if(!p) { n_rx_bad++; return; }
    pbuf_take(p, rxbuf + 14, len - 14);
    if(a314_netif.input(p, &a314_netif) != ERR_OK) pbuf_free(p);
  }
  // everything else, e.g. the magic frames of the driver, is only counted
}

/* ---- task ---- */

// report what the Amiga side does, once per change
static void report(void) {
  static uint32_t svc, con, rst, ref, cls;
  if(proto.st_service && !svc) debugf("A314: a314.device found (restart counter %u)", proto.restart);
  if(proto.st_restart != rst)  debugf("A314: a314.device restarted (restart counter %u)", proto.restart);
  if(proto.st_connect != con)  debugf("A314: ethernet service connected, stream %u", proto.stream);
  if(proto.st_refused != ref)  debugf("A314: connect to '%s' refused (%s)", (char*)proto.last_refused,
				      proto.connected ? "ethernet is already connected" : "unknown service");
  if(proto.st_reset != cls)    debugf("A314: stream closed by the amiga");
  svc = proto.st_service; con = proto.st_connect; rst = proto.st_restart; ref = proto.st_refused; cls = proto.st_reset;
}

static void service(bool force) {
  if(force || (a_status() & 1)) { a314_service(&proto); report(); }

  for(int round = 0; round < 8 && tx_rd != __atomic_load_n(&tx_wr, __ATOMIC_ACQUIRE) && a314_can_send(&proto); round++) {
    uint32_t rd = tx_rd;
    if(!a314_send_frame(&proto, txq[rd % TX_SLOTS].data, txq[rd % TX_SLOTS].len)) break;
    __atomic_store_n(&tx_rd, rd + 1, __ATOMIC_RELEASE);
    n_tx++;

    // room again: let tcp send what was refused (try again at the next frame if the mailbox is full)
    if(tx_blocked && tcpip_try_callback(tx_resume, NULL) == ERR_OK) tx_blocked = false;
  }
}

static void a314_task(void *arg) {
  (void)arg;

  while(!mcu_hw_tcpip_ready()) vTaskDelay(pdMS_TO_TICKS(100));

  int aw = a_id();
  if(!aw) {
    debugf("A314: no A314 interface in this core");
    vTaskDelete(NULL);
    return;
  }
  struct a314_ops ops = { NULL, a_mem_read, a_mem_write, a_irq_write, a_frame_from_amiga };
  a314_init(&proto, &ops, 1UL << aw);

  LOCK_TCPIP_CORE();
  ip4_addr_t mask, gw;
  IP4_ADDR(&mask, 255, 255, 255, 0);
  ip4_addr_set_zero(&gw);
  netif_add(&a314_netif, &our_ip, &mask, &gw, NULL, a314_netif_init, tcpip_input);
  netif_set_link_up(&a314_netif);
  netif_set_up(&a314_netif);
  ppp_port_peer_set(PEER_A314, &a314_netif, &peer_ip, false);
  ppp_port_peer_up(PEER_A314);
  UNLOCK_TCPIP_CORE();

  char ours[IP4ADDR_STRLEN_MAX], his[IP4ADDR_STRLEN_MAX];
  debugf("A314: up, %u bytes RAM, companion %s, amiga %s", (unsigned)(1UL << aw),
	 ip4addr_ntoa_r(&our_ip, ours, sizeof(ours)), ip4addr_ntoa_r(&peer_ip, his, sizeof(his)));

  TickType_t last_stats = xTaskGetTickCount(), last_lan = last_stats, last_force = last_stats;
  uint32_t last_rx = 0, last_tx = 0;

  for(;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(POLL_MS));

    TickType_t now = xTaskGetTickCount();
    bool force = now - last_force >= pdMS_TO_TICKS(FORCE_MS);
    if(force) last_force = now;
    service(force);

    if(now - last_lan >= pdMS_TO_TICKS(1000)) {    // wifi may get its address long after we started
      last_lan = now;
      LOCK_TCPIP_CORE();
      ppp_port_peer_poll(PEER_A314);
      UNLOCK_TCPIP_CORE();
    }
    if(now - last_stats >= pdMS_TO_TICKS(STATS_MS)) {
      last_stats = now;
      if(n_rx != last_rx || n_tx != last_tx)
	debugf("A314: connects %u restarts %u, frames from amiga %u (bad %u) to amiga %u, dropped %u, tx queue full %u",
	       (unsigned)proto.st_connect, (unsigned)proto.st_restart, (unsigned)n_rx, (unsigned)n_rx_bad,
	       (unsigned)n_tx, (unsigned)proto.st_dropped, (unsigned)tx_drops);
      last_rx = n_rx; last_tx = n_tx;
    }
  }
}

void a314_port_init(void) {
  if(task || !inifile_config_get_int("a314", "enable", 0)) return;

  ip4_addr_set_u32(&our_ip,  lwip_htonl(inifile_config_get_ip("a314", "ip",   0xc0a8fd01)));  // 192.168.253.1
  ip4_addr_set_u32(&peer_ip, lwip_htonl(inifile_config_get_ip("a314", "peer", 0xc0a8fd02)));  // 192.168.253.2

  if(xTaskCreate(a314_task, "a314", 1024, NULL, configMAX_PRIORITIES-10, &task) != pdPASS)
    debugf("A314: cannot create task");
}

void a314_handle_event(void) {
  if(task) xTaskNotifyGive(task);
}
