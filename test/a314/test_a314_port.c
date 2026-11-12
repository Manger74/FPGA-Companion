// Host test of a314_port.c + a314_proto.c + the forwarding hook of ppp_port.c with real lwIP (NO_SYS=1),
// the SPI byte protocol of target 6 (model of a314_host.v / a314_cp.v) and a model of a314.device + a314eth.device
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <task.h>
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "lwip/ip4.h"
#include "lwip/icmp.h"
#include "lwip/inet_chksum.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"

#include "a314_model.h"

/* ---- RTOS / platform stubs ---- */
static jmp_buf back;
static void (*task_fn)(void *);
static TickType_t ticks;
static int task_deleted;
int xTaskCreate(void (*f)(void*), const char *n, int s, void *a, int p, TaskHandle_t *h) { task_fn = f; if(h) *h = (void*)1; return 1; }
uint32_t ulTaskNotifyTake(int c, TickType_t t) { longjmp(back, 1); }
int xTaskNotifyGive(TaskHandle_t t) { return 1; }
TickType_t xTaskGetTickCount(void) { return ticks; }
void vTaskDelay(TickType_t t) {}
void vTaskDelete(TaskHandle_t t) { task_deleted++; }
u32_t sys_now(void) { return ticks; }
bool mcu_hw_tcpip_ready(void) { return true; }
static int cfg_burst;
int inifile_config_get_int(char *s, char *n, int def) { return !strcmp(n, "enable") ? 1 : !strcmp(n, "burst") ? cfg_burst : def; }
uint32_t inifile_config_get_ip(char *s, char *n, uint32_t def) {
  return !strcmp(n, "peer") ? 0xc0a80132 /* 192.168.1.50 */ : def;
}

/* ---- SPI: the core's target 6 ---- */
static int no_core;                    // ID reply of a core without A314
void mcu_hw_spi_begin(void) { spi_begin6(); }
void mcu_hw_spi_end(void) {}
unsigned char mcu_hw_spi_tx_u08(unsigned char b) {
  uint8_t r = spi_tx6(b);
  if(no_core && core.cmd == 4) return 0;
  return r;
}

static unsigned long fake_us, n_block;
unsigned long mcu_hw_micros(void) { return fake_us += 3; }
void mcu_hw_spi_write(const unsigned char *d, int len) { n_block++; for(int i = 0; i < len; i++) spi_tx6(d[i]); }

#define printf_dbg(...)
#include <a314_port.c>
#include "hook_only.c"

/* ---- lan interface ---- */
static struct netif lanif;
static uint8_t lanif_out[16][1600]; static int lanif_out_len[16], lanif_out_n;
static const uint8_t lan_mac[6] = {2,0,0,0,0,1}, host_mac[6] = {2,0,0,0,0,20}, amiga_mac[6] = {0x02,0xaa,0,0,0,0x50};
static err_t lan_linkoutput(struct netif *n, struct pbuf *p) {
  lanif_out_len[lanif_out_n] = pbuf_copy_partial(p, lanif_out[lanif_out_n], p->tot_len, 0); lanif_out_n++; return ERR_OK;
}
static err_t lan_init(struct netif *n) {
  n->name[0] = 'e'; n->name[1] = 'n'; n->output = etharp_output; n->linkoutput = lan_linkoutput;
  n->mtu = 1500; n->hwaddr_len = 6; memcpy(n->hwaddr, lan_mac, 6);
  n->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_LINK_UP;
  return ERR_OK;
}

static int mk_ip_icmp(uint8_t *f, const uint8_t *dmac, const uint8_t *smac, uint32_t src, uint32_t dst, int ttl, int plen) {
  memcpy(f, dmac, 6); memcpy(f + 6, smac, 6); f[12] = 8; f[13] = 0;
  uint8_t *ip = f + 14; int len = 20 + 8 + plen;
  memset(ip, 0, len);
  ip[0] = 0x45; ip[2] = len >> 8; ip[3] = len; ip[6] = 0x40; ip[8] = ttl; ip[9] = 1;
  uint32_t s = lwip_htonl(src), d = lwip_htonl(dst); memcpy(ip + 12, &s, 4); memcpy(ip + 16, &d, 4);
  uint8_t *ic = ip + 20; ic[0] = 8; ic[4] = 0x12; ic[5] = 0x34; ic[7] = 1;
  for(int i = 0; i < plen; i++) ic[8 + i] = i;
  uint16_t c = inet_chksum(ic, 8 + plen); ic[2] = c & 0xff; ic[3] = c >> 8;
  c = inet_chksum(ip, 20); ip[10] = c & 0xff; ip[11] = c >> 8;
  return 14 + len;
}
static int mk_arp(uint8_t *f, int op, const uint8_t *smac, uint32_t sip, const uint8_t *dmac, uint32_t dip, const uint8_t *eth_dst) {
  memcpy(f, eth_dst, 6); memcpy(f + 6, smac, 6); f[12] = 8; f[13] = 6;
  uint8_t *a = f + 14; a[0] = 0; a[1] = 1; a[2] = 8; a[3] = 0; a[4] = 6; a[5] = 4; a[6] = 0; a[7] = op;
  memcpy(a + 8, smac, 6); uint32_t s = lwip_htonl(sip), d = lwip_htonl(dip); memcpy(a + 14, &s, 4);
  memcpy(a + 18, dmac, 6); memcpy(a + 24, &d, 4);
  return 14 + 28;
}
static void lan_inject(const uint8_t *f, int len) {
  struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL); pbuf_take(p, f, len);
  if(lanif.input(p, &lanif) != ERR_OK) pbuf_free(p);
}
static uint32_t ip_of(const uint8_t *ip, int off) { uint32_t v; memcpy(&v, ip + off, 4); return lwip_ntohl(v); }

// the Amiga and the Companion each get their turn
static void pump(void) { for(int i = 0; i < 4; i++) { service(false); a_service(); } }
static void amiga_send(const uint8_t *f, int len) {
  bool ok = a_send_frame(f, len); CHECK(ok, "Amiga has a free send buffer");
  pump();
}

int main(int argc, char **argv) {
  cfg_burst = argc > 1;                 // any argument: [A314] BURST=1
  static const uint8_t bcast[6] = {255,255,255,255,255,255}, zero[6] = {0};
  uint8_t f[1600]; int n;

  lwip_init();
  ip4_addr_t a, mk, gw; IP4_ADDR(&a, 192,168,1,10); IP4_ADDR(&mk, 255,255,255,0); IP4_ADDR(&gw, 192,168,1,1);
  netif_add(&lanif, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4, NULL, lan_init, netif_input);
  netif_set_default(&lanif); netif_set_up(&lanif); netif_set_link_up(&lanif);

  // 0. a core without A314: the task ends
  core_reset(); no_core = 1;
  a314_port_init();
  CHECK(task_fn != NULL, "task created when [a314] enable=1");
  task_fn(NULL);
  CHECK(task_deleted == 1, "no A314 in the core: task ends");
  no_core = 0;
  task = NULL;

  // 1. start with the core
  core_reset(); sim_clear_amiga();
  lanif_out_n = 0;
  a314_port_init();
  if(!setjmp(back)) task_fn(NULL);
  CHECK(task_deleted == 1 && proto.ram_size == M_RAM, "task runs, RAM size taken from the core");
  CHECK(a_status() == 0, "STATUS over SPI: no interrupt");
  netif_set_addr(&lanif, &a, &mk, &gw);
  lanif_out_n = 0; ppp_port_peer_poll(PEER_A314);
  CHECK(lanif_out_n == 1 && lanif_out[0][12] == 8 && lanif_out[0][13] == 6 && lanif.input == lan_input,
	"lan hooked, gratuitous ARP for the peer");

  // 2. a314.device starts, a314eth connects and offers buffers
  a_start();
  CHECK(a_status() == 1, "STATUS over SPI: set_pi_irq seen");
  pump();
  a_connect("ethernet", eth_stream); pump();
  CHECK(eth_connected && proto.connected, "CONNECT ethernet through the SPI protocol");
  a_post_rx_buffer(0); a_post_rx_buffer(1); pump();
  CHECK(proto.n_rx == 2, "two receive buffers");

  // 3. ARP request of the Amiga for its gateway: proxy answer
  n = mk_arp(f, 1, amiga_mac, 0xc0a80132, zero, 0xc0a80101, bcast);
  amiga_send(f, n);
  CHECK(a_rx_n == 1 && a_rxq[0].len == 42 && !memcmp(a_rxq[0].data, amiga_mac, 6) && !memcmp(a_rxq[0].data + 6, our_mac, 6)
	&& a_rxq[0].data[21] == 2 && ip_of(a_rxq[0].data + 14, 14) == 0xc0a80101, "ARP reply for 192.168.1.1 arrives at the Amiga");
  a_rx_n = 0;

  // 4. probe for its own address: no answer
  n = mk_arp(f, 1, amiga_mac, 0, zero, 0xc0a80132, bcast); amiga_send(f, n);
  CHECK(a_rx_n == 0, "no answer to an ARP probe for its own address");

  // 5. Amiga pings a lan host: forwarded
  n = mk_ip_icmp(f, our_mac, amiga_mac, 0xc0a80132, 0xc0a80114, 64, 32); amiga_send(f, n);
  CHECK(lanif_out_n == 2 && lanif_out[1][21] == 1, "lan: ARP request for 192.168.1.20");
  n = mk_arp(f, 2, host_mac, 0xc0a80114, lan_mac, 0xc0a8010a, lan_mac); lan_inject(f, n);
  CHECK(lanif_out_n == 3 && lanif_out[2][13] == 0 && !memcmp(lanif_out[2], host_mac, 6), "lan: ping forwarded after ARP");
  uint8_t *ip = lanif_out[2] + 14;
  CHECK(ip_of(ip, 12) == 0xc0a80132 && ip_of(ip, 16) == 0xc0a80114 && ip[8] == 63 && inet_chksum(ip, 20) == 0, "src unchanged, ttl 63, checksum valid");

  // 6. the host pings the Amiga: proxy ARP, then to the Amiga through the A314
  lanif_out_n = 0;
  n = mk_arp(f, 1, host_mac, 0xc0a80114, zero, 0xc0a80132, bcast); lan_inject(f, n);
  CHECK(lanif_out_n == 1 && lanif_out[0][21] == 2, "lan: proxy ARP reply for the Amiga");
  n = mk_ip_icmp(f, lan_mac, host_mac, 0xc0a80114, 0xc0a80132, 64, 100); lan_inject(f, n);
  pump();
  uint8_t *o = a_rxq[0].data;
  CHECK(a_rx_n == 1 && a_rxq[0].len == 14 + 20 + 8 + 100 && !memcmp(o, amiga_mac, 6) && !memcmp(o + 6, our_mac, 6)
	&& o[12] == 8 && o[13] == 0 && o[22] == 63 && inet_chksum(o + 14, 20) == 0 && inet_chksum(o + 34, 108) == 0,
	"frame for the Amiga: MAC learned, ttl 63, checksums");
  a_rx_n = 0;

  // 6b. a large frame for the Amiga (written as one SPI block with BURST=1)
  n = mk_ip_icmp(f, lan_mac, host_mac, 0xc0a80114, 0xc0a80132, 64, 1400); lan_inject(f, n);
  pump();
  o = a_rxq[0].data;
  CHECK(a_rx_n == 1 && a_rxq[0].len == 14 + 20 + 8 + 1400 && inet_chksum(o + 14, 20) == 0 && inet_chksum(o + 34, 1408) == 0,
	"1442 byte frame arrives unchanged");
  a_rx_n = 0;

  // 7. ping to the Companion on the link is answered
  n = mk_ip_icmp(f, our_mac, amiga_mac, 0xc0a80132, 0xc0a8fd01, 64, 16); amiga_send(f, n);
  o = a_rxq[0].data;
  CHECK(a_rx_n == 1 && o[34] == 0 && ip_of(o + 14, 12) == 0xc0a8fd01 && ip_of(o + 14, 16) == 0xc0a80132, "echo reply from the Companion's link address");
  a_rx_n = 0;

  // 8. the Amiga has no receive buffer: frames wait (3 slots), the rest is refused, all drain when buffers come back
  //    (take the buffers away: the model re-posts after each frame, so post only after the burst)
  memset(buf_pending, 0, sizeof(buf_pending));
  proto.n_rx = 0;                                           // pretend both buffers are used
  n = mk_ip_icmp(f, lan_mac, host_mac, 0xc0a80114, 0xc0a80132, 64, 8);
  tx_drops = 0;
  for(int i = 0; i < 6; i++) lan_inject(f, n);
  service(false);
  CHECK(tx_wr - tx_rd == 3 && tx_drops == 3 && a_rx_n == 0, "no receive buffer: 3 frames queued, 3 refused");
  a_post_rx_buffer(0); pump();
  CHECK(a_rx_n == 3 && tx_wr == tx_rd, "after a buffer comes back all queued frames drain (the driver re-posts after every frame)");
  a_rx_n = 0;

  // 9. frames of the Amiga that are not IP/ARP are only counted; short frames too
  uint8_t mg[64] = {0}; memcpy(mg, bcast, 6); memcpy(mg + 6, amiga_mac, 6); mg[12] = 0xff; mg[13] = 0xfe;
  uint32_t bad0 = n_rx_bad;
  amiga_send(mg, 60);
  CHECK(a_rx_n == 0 && n_rx_bad == bad0, "unknown ethertype is consumed");
  { uint8_t m[8]; put_msg(m, 520 + 2 * 1520, 10, 2); a_send_pkt(6, eth_stream, 8, m); buf_pending[2] = true; pump(); }
  CHECK(!buf_pending[2], "short frame: buffer given back");

  // 9b. timing statistics: every kind of buffer event has been seen, the report resets them
  CHECK(s_dl_amiga.n > 0 && s_dl_queue.n > 0 && s_ul_comp.n > 0 && s_ul_amiga.n > 0,
	"timing: download (amiga, queue) and upload (amiga, companion) measured");
  CHECK(s_ul_amiga.n < s_ul_comp.n, "timing: first use of a send buffer has no amiga time");
  timing_report();
  CHECK(!s_dl_amiga.n && !s_dl_queue.n && !s_ul_comp.n && !s_ul_amiga.n && !t_busy, "timing: report resets the sums");

  // 10. Amiga reset: link goes through the restart and works again
  sim_clear_amiga();
  a_start(); pump();
  CHECK(!proto.connected, "after the restart: connection gone");
  a_connect("ethernet", eth_stream); pump(); a_post_rx_buffer(0); a_post_rx_buffer(1); pump();
  n = mk_arp(f, 1, amiga_mac, 0xc0a80132, zero, 0xc0a80101, bcast); amiga_send(f, n);
  CHECK(a_rx_n == 1, "traffic works after the restart");

  // 11. missed interrupt: the forced service (every second) finds the work
  a_rx_n = 0;
  { uint8_t m[8]; put_msg(m, 520 + 2 * 1520, 10, 2); a_write_to_a2r(6, eth_stream, 8, m); a_write_amiga_cap(); buf_pending[2] = true; }
  service(false);
  CHECK(buf_pending[2], "no interrupt, no service");
  service(true);
  a_service();
  CHECK(!buf_pending[2], "forced service takes the request");

  printf("   SPI bytes in total: %u, block writes: %lu\n", (unsigned)core.n_spi, n_block);
  CHECK((n_block > 0) == (cfg_burst != 0), "block writes only with BURST=1");
  printf("%s (%d Checks, %d Fehler)\n", fails ? "FAIL" : "PASS", checks, fails);
  return fails != 0;
}
