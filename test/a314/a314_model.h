/*
  Software model for the host tests of the A314 interface:

   - the core: a314_cp.v (RAM, pointer, interrupt bits as in cp_pi_if.v) and a314_host.v (SPI target 6
     with the byte pipeline of the real thing: the reply to a byte is decided by the byte before it)
   - the Amiga: a314.device (pi_if_cp.c) and a314eth.device (device.c), as far as the protocol goes
*/
#ifndef A314_MODEL_H
#define A314_MODEL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#define M_AW         13
#define M_RAM        (1u << M_AW)

static int fails, checks;
#define CHECK(c, msg) do { checks++; if(!(c)) { printf("FEHLER: %s (Zeile %d)\n", msg, __LINE__); fails++; } } while(0)

/* ---------------------------------------------------------------- core */
static struct {
  uint8_t  ram[M_RAM];
  uint16_t ptr;                  // pointer of the Amiga
  bool     cp_irq, pi_irq;       // INT6 / interrupt of the Pi
  // SPI target 6
  int      pos; uint8_t cmd, reply, addr_hi; uint16_t haddr; uint32_t n_spi;
} core;

static void core_reset(void) { memset(&core, 0, sizeof(core)); }

static void irq_write(uint8_t v) {                // REG_IRQ write, same for both sides
  uint8_t cur = (core.pi_irq << 1) | core.cp_irq;
  cur = (v & 0x80) ? (cur | (v & 3)) : (cur & ~(v & 3));
  core.pi_irq = (cur >> 1) & 1; core.cp_irq = cur & 1;
}

// Amiga side: one byte access to the register block (reg 0..3)
static uint8_t cp_access(bool we, int reg, uint8_t d) {
  uint8_t r = 0;
  switch(reg) {
  case 0: r = core.ram[core.ptr & (M_RAM - 1)]; if(we) core.ram[core.ptr & (M_RAM - 1)] = d; core.ptr++; break;
  case 1: r = core.cp_irq; if(we) irq_write(d); break;
  case 2: if(we) core.ptr = (core.ptr & 0xff00) | d; break;
  case 3: if(we) core.ptr = (core.ptr & 0x00ff) | (d << 8); break;
  }
  return we ? 0 : r;
}
static void cp_wr(int reg, uint8_t d) { cp_access(true, reg, d); }
static uint8_t cp_rd(int reg) { return cp_access(false, reg, 0); }

// SPI target 6 as a314_host.v: first byte after the target is the command
__attribute__((unused)) static void spi_begin6(void) { core.pos = -2; core.reply = 0xEE; }
__attribute__((unused)) static uint8_t spi_tx6(uint8_t b) {
  uint8_t ret = core.reply;
  core.n_spi++;
  if(core.pos == -2) { assert(b == 6); core.pos = -1; return ret; }
  if(core.pos == -1) { core.cmd = b; core.reply = core.pi_irq; core.pos = 0; return ret; }
  int cnt = core.pos++;
  switch(core.cmd) {
  case 0: core.reply = 0; break;
  case 1: if(cnt == 0) { core.addr_hi = b; core.reply = 0; }
          else if(cnt == 1) { core.haddr = ((core.addr_hi << 8) | b) & (M_RAM - 1); core.reply = 0; }
          else { core.reply = core.ram[core.haddr]; core.haddr = (core.haddr + 1) & (M_RAM - 1); }
          break;
  case 2: if(cnt == 0) core.addr_hi = b;
          else if(cnt == 1) core.haddr = ((core.addr_hi << 8) | b) & (M_RAM - 1);
          else { core.ram[core.haddr] = b; core.haddr = (core.haddr + 1) & (M_RAM - 1); }
          break;
  case 3: if(cnt == 0) irq_write(b); break;
  case 4: core.reply = cnt == 0 ? 0xA3 : cnt == 1 ? 1 : cnt == 2 ? M_AW : 0; break;
  default: core.reply = 0;
  }
  return ret;
}

/* ---------------------------------------------------------------- Amiga */
#define REG_SRAM 0
#define REG_IRQ  1
#define REG_LO   2
#define REG_HI   3
#define A2R_BASE 0
#define R2A_BASE 256
#define CAP_BASE 512

static void set_ptr(uint16_t a) { cp_wr(REG_LO, a & 0xff); cp_wr(REG_HI, a >> 8); }

static struct { uint8_t r2a_tail, a2r_head, a2r_tail, r2a_head; } cap;   // struct ComAreaPtrs of the Amiga

static void a_write_mem(uint16_t a, const uint8_t *s, int n) { set_ptr(a); for(int i = 0; i < n; i++) cp_wr(REG_SRAM, s[i]); }
static void a_read_mem(uint8_t *d, uint16_t a, int n)  { set_ptr(a); for(int i = 0; i < n; i++) d[i] = cp_rd(REG_SRAM); }
static void a_set_pi_irq(void) { cp_wr(REG_IRQ, 0x82); }

static bool a_probe(void) {
  cp_wr(REG_HI, CAP_BASE >> 8); cp_wr(REG_LO, 6);
  cp_wr(REG_SRAM, 0x99); cp_wr(REG_SRAM, 0xbd);
  cp_wr(REG_LO, 7);
  if(cp_rd(REG_SRAM) != 0xbd) return false;
  cp_wr(REG_LO, 6);
  return cp_rd(REG_SRAM) == 0x99;
}
static void a_read_pi_cap(void) { set_ptr(CAP_BASE); cap.r2a_tail = cp_rd(REG_SRAM); cap.a2r_head = cp_rd(REG_SRAM); }
static void a_write_amiga_cap(void) { set_ptr(CAP_BASE + 2); cp_wr(REG_SRAM, cap.a2r_tail); cp_wr(REG_SRAM, cap.r2a_head); }

// a314.device start: probe, clear_cap, update_restart_counter, set_pi_irq
static void a_start(void) {
  assert(a_probe());
  set_ptr(CAP_BASE); for(int i = 0; i < 4; i++) cp_wr(REG_SRAM, 0);
  memset(&cap, 0, sizeof(cap));
  set_ptr(CAP_BASE + 4); uint8_t rc = cp_rd(REG_SRAM);
  set_ptr(CAP_BASE + 4); cp_wr(REG_SRAM, rc + 1); cp_wr(REG_SRAM, 0xa3); cp_wr(REG_SRAM, 0x14);
  a_set_pi_irq();
}

static void a_write_to_a2r(uint8_t type, uint8_t stream, uint8_t len, const uint8_t *data) {
  uint8_t off = cap.a2r_tail;
  uint8_t hdr[3] = { len, type, stream };
  set_ptr(A2R_BASE + off);
  for(int i = 0; i < 3; i++) { cp_wr(REG_SRAM, hdr[i]); if(++off == 0) set_ptr(A2R_BASE); }
  for(int i = 0; i < len; i++) { cp_wr(REG_SRAM, data[i]); if(++off == 0) set_ptr(A2R_BASE); }
  cap.a2r_tail = off;
}
static void a_send_pkt(uint8_t type, uint8_t stream, uint8_t len, const uint8_t *data) {
  a_write_to_a2r(type, stream, len, data);
  a_write_amiga_cap();
  a_set_pi_irq();
}

/* a314eth.device: 2 receive and 2 send buffers of RAW_MTU at the start of the allocator */
#define NBUF  4
#define RAW_MTU 1518
static const uint16_t buf_addr[NBUF] = { 520, 520 + 1520, 520 + 2 * 1520, 520 + 3 * 1520 };   // 0,1 = rx; 2,3 = tx
static bool buf_pending[NBUF];       // request to the Pi is out
static uint8_t eth_stream = 1;
static bool eth_connected; static int eth_connect_result = -1;
static struct { uint8_t data[RAW_MTU]; int len; } a_rxq[16]; static int a_rx_n;    // frames received by a314eth
static int a_tx_done;                                                          // WRITE_FRAME_RES seen
static uint32_t a_pkts_seen;

static void put_msg(uint8_t *m, uint32_t addr, uint16_t len, uint16_t kind) {
  m[0] = addr >> 24; m[1] = addr >> 16; m[2] = addr >> 8; m[3] = addr; m[4] = len >> 8; m[5] = len; m[6] = kind >> 8; m[7] = kind;
}
static void a_connect(const char *name, uint8_t stream) { a_send_pkt(4, stream, strlen(name), (const uint8_t *)name); }
static void a_post_rx_buffer(int i) {
  uint8_t m[8]; put_msg(m, buf_addr[i], RAW_MTU, 1);
  buf_pending[i] = true;
  a_send_pkt(6, eth_stream, 8, m);
}
static bool a_send_frame(const uint8_t *f, int len) {            // a314eth SANA-II write
  for(int i = 2; i < 4; i++) if(!buf_pending[i]) {
    a_write_mem(buf_addr[i], f, len);
    uint8_t m[8]; put_msg(m, buf_addr[i], len, 2);
    buf_pending[i] = true;
    a_send_pkt(6, eth_stream, 8, m);
    return true;
  }
  return false;
}

// IntServer + driver task: take what the Pi has put into R2A
static void a_service(void) {
  if(!core.cp_irq) return;                       // the interrupt line
  if(cp_rd(REG_IRQ) == 0) return;                // spurious
  cp_wr(REG_IRQ, 0x01);                          // clear_amiga_irq
  a_read_pi_cap();
  int len = (uint8_t)(cap.r2a_tail - cap.r2a_head);
  if(!len) return;
  static uint8_t b[256];
  uint8_t off = cap.r2a_head;
  set_ptr(R2A_BASE + off);
  for(int i = 0; i < len; i++) { b[i] = cp_rd(REG_SRAM); if(++off == 0) set_ptr(R2A_BASE); }
  const uint8_t *p = b;
  while(p < b + len) {
    uint8_t plen = p[0], type = p[1], stream = p[2]; p += 3;
    a_pkts_seen++;
    if(type == 5) { eth_connect_result = p[0]; eth_connected = p[0] == 0; }
    else if(type == 6 && stream == eth_stream && plen == 8) {
      uint32_t addr = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
      uint16_t flen = (p[4] << 8) | p[5], kind = (p[6] << 8) | p[7];
      int bi = -1; for(int i = 0; i < NBUF; i++) if(buf_addr[i] == addr) bi = i;
      if(bi >= 0) buf_pending[bi] = false;       // bad requests of the tests come back with their own address
      if(kind == 3) {
        assert(bi >= 0);
        assert(a_rx_n < 16 && flen <= RAW_MTU);
        a_read_mem(a_rxq[a_rx_n].data, addr, flen); a_rxq[a_rx_n].len = flen; a_rx_n++;
        a_post_rx_buffer(bi);                    // buffer is free again
      } else if(kind == 4) a_tx_done++;
    }
    p += plen;
  }
  cap.r2a_head = cap.r2a_tail;
  a_write_amiga_cap();
  a_set_pi_irq();
}

static void sim_clear_amiga(void) {               // forget everything of a previous driver run
  memset(buf_pending, 0, sizeof(buf_pending));
  eth_connected = false; eth_connect_result = -1; a_rx_n = 0; a_tx_done = 0;
}

#endif
