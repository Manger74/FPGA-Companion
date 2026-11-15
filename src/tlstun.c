/*
 * tlstun - plaintext to TLS port forwarder (stunnel-like, client mode)
 *
 * A plain TCP connection to a local port is forwarded to host:port via TLS,
 * so retro machines can talk to TLS-only servers without doing TLS themselves.
 *
 * config.ini:
 *   [TLSTUN]
 *   MAP=993:imap.example.com:993,465:smtp.example.com:465
 *   CA=cacert.pem         ; PEM root certificates on the SD card
 *   VERIFY=required       ; or "none" (testing only, no server authentication)
 *
 * Certificate checks need the time, so with VERIFY=required connections are
 * refused until SNTP has delivered it.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/netif.h"

#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/platform_time.h"
#include "mbedtls/net_sockets.h"   // error codes only
#include "mbedtls/error.h"

#include "ff.h"
#include "sdc.h"
#include "inifile.h"
#include "debug.h"
#include "tlstun.h"

#ifndef TLSTUN_MAX_MAPS
#define TLSTUN_MAX_MAPS      4
#endif
#ifndef TLSTUN_MAX_SESSIONS
#define TLSTUN_MAX_SESSIONS  3
#endif
#define TLSTUN_STACK_WORDS   2048     // mbedtls handshake needs a few KB
#define TLSTUN_TIMEOUT_S     30       // TLS socket receive timeout
#define TLSTUN_BUF_LEN       1024

static struct {
  uint16_t lport, rport;
  char host[64];
  int lfd;
} s_map[TLSTUN_MAX_MAPS];
static int s_num_maps;

static mbedtls_x509_crt s_ca;         // shared, read-only once loaded
static bool s_verify;

typedef struct { int fd, map; } job_t;
static QueueHandle_t s_jobs;

/* per session context, static so the FreeRTOS heap is not involved */
static struct worker_s {
  mbedtls_ssl_context ssl;
  mbedtls_ssl_config conf;
  mbedtls_entropy_context entropy;
  mbedtls_ctr_drbg_context drbg;
  unsigned char buf[TLSTUN_BUF_LEN];
  StaticTask_t tcb;
  StackType_t stack[TLSTUN_STACK_WORDS];
} s_worker[TLSTUN_MAX_SESSIONS];

/* ---- time for certificate validity checks -------------------------------- */
static volatile uint32_t s_utc;
static volatile TickType_t s_utc_tick;

void tlstun_set_time(uint32_t utc) {
  s_utc_tick = xTaskGetTickCount();
  s_utc = utc;
}

static mbedtls_time_t tlstun_time(mbedtls_time_t *t) {
  mbedtls_time_t now = s_utc ?
    s_utc + (xTaskGetTickCount() - s_utc_tick) / configTICK_RATE_HZ : 0;
  if(t) *t = now;
  return now;
}

mbedtls_ms_time_t mbedtls_ms_time(void) {   // MBEDTLS_PLATFORM_MS_TIME_ALT
  return (mbedtls_ms_time_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* ---- mbedtls bio on lwip sockets ----------------------------------------- */
static int bio_send(void *ctx, const unsigned char *buf, size_t len) {
  int n = lwip_send(*(int *)ctx, buf, len, 0);
  return n < 0 ? MBEDTLS_ERR_NET_SEND_FAILED : n;
}

static int bio_recv(void *ctx, unsigned char *buf, size_t len) {
  int n = lwip_recv(*(int *)ctx, buf, len, 0);
  if(n >= 0) return n;   // 0 = EOF
  return (errno == EAGAIN || errno == EWOULDBLOCK) ?
    MBEDTLS_ERR_SSL_TIMEOUT : MBEDTLS_ERR_NET_RECV_FAILED;
}

static int send_all(int fd, const unsigned char *p, int len) {
  while(len > 0) {
    int n = lwip_send(fd, p, len, 0);
    if(n <= 0) return -1;
    p += n; len -= n;
  }
  return 0;
}

static int tls_write_all(mbedtls_ssl_context *ssl, const unsigned char *p, int len) {
  while(len > 0) {
    int n = mbedtls_ssl_write(ssl, p, len);
    if(n == MBEDTLS_ERR_SSL_WANT_READ || n == MBEDTLS_ERR_SSL_WANT_WRITE) continue;
    if(n <= 0) return -1;
    p += n; len -= n;
  }
  return 0;
}

static int tcp_connect(const char *host, uint16_t port) {
  char sport[6];
  struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM }, *res;
  snprintf(sport, sizeof(sport), "%u", port);
  if(lwip_getaddrinfo(host, sport, &hints, &res) || !res) return -1;

  int fd = lwip_socket(AF_INET, SOCK_STREAM, 0);
  if(fd >= 0 && lwip_connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
    lwip_close(fd);
    fd = -1;
  }
  lwip_freeaddrinfo(res);
  return fd;
}

static void keepalive(int fd) {
  int one = 1, idle = 60, intvl = 10, cnt = 3;
  lwip_setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
  lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
  lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
}

/* ---- one forwarded connection -------------------------------------------- */
static void session(struct worker_s *w, int cfd, int m) {
  const char *host = s_map[m].host;
  int ret, sfd;

  if(s_verify && !s_utc) {
    debugf("TLSTUN: no time from SNTP yet, refusing %s", host);
    return;
  }
  if((sfd = tcp_connect(host, s_map[m].rport)) < 0) {
    debugf("TLSTUN: connect to %s:%u failed", host, s_map[m].rport);
    return;
  }
  struct timeval tv = { .tv_sec = TLSTUN_TIMEOUT_S };
  lwip_setsockopt(sfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  lwip_setsockopt(sfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  keepalive(sfd);
  keepalive(cfd);

  mbedtls_ssl_init(&w->ssl);
  if((ret = mbedtls_ssl_setup(&w->ssl, &w->conf)) ||
     (ret = mbedtls_ssl_set_hostname(&w->ssl, host)))   // SNI and name check
    goto out;
  mbedtls_ssl_set_bio(&w->ssl, &sfd, bio_send, bio_recv, NULL);

  while((ret = mbedtls_ssl_handshake(&w->ssl)))
    if(ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
      goto out;

  debugf("TLSTUN: %u -> %s:%u (%s, %s)", s_map[m].lport, host, s_map[m].rport,
         mbedtls_ssl_get_version(&w->ssl), mbedtls_ssl_get_ciphersuite(&w->ssl));

  bool cdone = false;   // client half-closed, only server data left to forward
  for(;;) {
    fd_set rd;
    FD_ZERO(&rd);
    FD_SET(sfd, &rd);
    // decrypted data may already be buffered without the socket being readable
    if(!mbedtls_ssl_get_bytes_avail(&w->ssl)) {
      if(!cdone) FD_SET(cfd, &rd);
      if(lwip_select((cfd > sfd ? cfd : sfd) + 1, &rd, NULL, NULL, NULL) <= 0) break;
    }

    if(FD_ISSET(sfd, &rd)) {
      int n = mbedtls_ssl_read(&w->ssl, w->buf, sizeof(w->buf));
      if(n == MBEDTLS_ERR_SSL_WANT_READ || n == MBEDTLS_ERR_SSL_WANT_WRITE ||
         n == MBEDTLS_ERR_SSL_TIMEOUT) continue;
      if(n <= 0 || send_all(cfd, w->buf, n)) break;   // close_notify, EOF, error
    }
    if(!cdone && FD_ISSET(cfd, &rd)) {
      int n = lwip_recv(cfd, w->buf, sizeof(w->buf), 0);
      if(n == 0) {          // pass the half-close on, keep reading replies
        cdone = true;
        mbedtls_ssl_close_notify(&w->ssl);
        lwip_shutdown(sfd, SHUT_WR);
      } else if(n < 0 || tls_write_all(&w->ssl, w->buf, n)) break;
    }
  }
  if(!cdone) mbedtls_ssl_close_notify(&w->ssl);
  ret = 0;

out:
  if(ret) {
    char err[80];
    mbedtls_strerror(ret, err, sizeof(err));
    debugf("TLSTUN: %s: %s (-0x%04x)", host, err, -ret);
    uint32_t flags = mbedtls_ssl_get_verify_result(&w->ssl);
    if(flags && flags != (uint32_t)-1) {
      mbedtls_x509_crt_verify_info(err, sizeof(err), "", flags);
      debugf("TLSTUN: certificate: %s", err);
    }
  }
  mbedtls_ssl_free(&w->ssl);
  lwip_close(sfd);
}

static void worker_thread(void *arg) {
  struct worker_s *w = arg;
  job_t job;
  for(;;) {
    xQueueReceive(s_jobs, &job, portMAX_DELAY);
    session(w, job.fd, job.map);
    lwip_close(job.fd);
  }
}

/* ---- setup --------------------------------------------------------------- */
static int worker_setup(struct worker_s *w) {
  mbedtls_ssl_config_init(&w->conf);
  mbedtls_entropy_init(&w->entropy);
  mbedtls_ctr_drbg_init(&w->drbg);
  if(mbedtls_ctr_drbg_seed(&w->drbg, mbedtls_entropy_func, &w->entropy,
                           (const unsigned char *)"tlstun", 6) ||
     mbedtls_ssl_config_defaults(&w->conf, MBEDTLS_SSL_IS_CLIENT,
                                 MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT))
    return -1;
  mbedtls_ssl_conf_rng(&w->conf, mbedtls_ctr_drbg_random, &w->drbg);
  mbedtls_ssl_conf_authmode(&w->conf, s_verify ? MBEDTLS_SSL_VERIFY_REQUIRED
                                               : MBEDTLS_SSL_VERIFY_NONE);
  mbedtls_ssl_conf_ca_chain(&w->conf, &s_ca, NULL);
  return 0;
}

// load PEM file from SD card, returns number of certificates
static int load_ca(const char *name) {
  char path[80];
  FIL fil;
  UINT got;
  snprintf(path, sizeof(path), "%s/%s", CARD_MOUNTPOINT, name);
  if(f_open(&fil, path, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
    debugf("TLSTUN: cannot open %s", path);
    return 0;
  }
  FSIZE_t size = f_size(&fil);
  unsigned char *pem = malloc(size + 1);
  if(pem && f_read(&fil, pem, size, &got) == FR_OK && got == size) {
    pem[size] = 0;   // PEM parser needs the terminating NUL
    int bad = mbedtls_x509_crt_parse(&s_ca, pem, size + 1);
    if(bad) debugf("TLSTUN: %d certificates in %s not loaded", bad, path);
  }
  free(pem);
  f_close(&fil);

  int n = 0;
  for(mbedtls_x509_crt *c = &s_ca; c && c->raw.len; c = c->next) n++;
  return n;
}

// "lport:host:rport,lport:host:rport,..."
static void parse_maps(const char *s) {
  while(s && *s && s_num_maps < TLSTUN_MAX_MAPS) {
    unsigned lp, rp;
    char host[sizeof(s_map[0].host)];
    if(sscanf(s, "%u:%63[^:]:%u", &lp, host, &rp) == 3 && lp && rp &&
       lp < 65536 && rp < 65536) {
      s_map[s_num_maps].lport = lp;
      s_map[s_num_maps].rport = rp;
      strcpy(s_map[s_num_maps++].host, host);
    } else
      debugf("TLSTUN: ignoring bad MAP entry '%s'", s);
    s = strchr(s, ',');
    if(s) s++;
  }
}

static int listen_on(uint16_t port) {
  struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = PP_HTONS(port),
                            .sin_addr.s_addr = PP_HTONL(INADDR_ANY) };
  int one = 1, fd = lwip_socket(AF_INET, SOCK_STREAM, 0);
  if(fd < 0) return -1;
  lwip_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  if(lwip_bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0 || lwip_listen(fd, 1) < 0) {
    lwip_close(fd);
    return -1;
  }
  return fd;
}

static void tlstun_thread(void *arg) {
  (void)arg;
  // config.ini is read by another task, and the socket API must not be
  // touched before lwip is up (see telnetd.c)
  while(!inifile_config_is_read() || netif_default == NULL)
    vTaskDelay(pdMS_TO_TICKS(200));

  parse_maps(inifile_config_get_str("tlstun", "map"));
  if(!s_num_maps) { vTaskDelete(NULL); return; }

  s_verify = inifile_config_get_int("tlstun", "verify", 0) == 0;
  mbedtls_platform_set_time(tlstun_time);
  mbedtls_x509_crt_init(&s_ca);
  char *ca = inifile_config_get_str("tlstun", "ca");
  int ncerts = ca ? load_ca(ca) : 0;
  if(s_verify && !ncerts) {
    debugf("TLSTUN: no CA certificates, disabled (set CA= or VERIFY=none)");
    vTaskDelete(NULL); return;
  }
  if(!s_verify) debugf("TLSTUN: WARNING, server certificates are NOT verified");

  s_jobs = xQueueCreate(1, sizeof(job_t));   // at most one connection waits
  for(int i = 0; i < TLSTUN_MAX_SESSIONS; i++) {
    if(worker_setup(&s_worker[i])) {
      debugf("TLSTUN: TLS setup failed");
      vTaskDelete(NULL); return;
    }
    xTaskCreateStatic(worker_thread, "tlstun_w", TLSTUN_STACK_WORDS, &s_worker[i],
                      configMAX_PRIORITIES - 3, s_worker[i].stack, &s_worker[i].tcb);
  }

  int maxfd = -1;
  for(int i = 0; i < s_num_maps; i++) {
    s_map[i].lfd = listen_on(s_map[i].lport);
    if(s_map[i].lfd < 0)
      debugf("TLSTUN: cannot listen on port %u", s_map[i].lport);
    else {
      debugf("TLSTUN: port %u -> %s:%u", s_map[i].lport, s_map[i].host, s_map[i].rport);
      if(s_map[i].lfd > maxfd) maxfd = s_map[i].lfd;
    }
  }
  debugf("TLSTUN: %d CA certificates, %d sessions max", ncerts, TLSTUN_MAX_SESSIONS);

  for(;;) {
    fd_set rd;
    FD_ZERO(&rd);
    for(int i = 0; i < s_num_maps; i++)
      if(s_map[i].lfd >= 0) FD_SET(s_map[i].lfd, &rd);
    if(maxfd < 0 || lwip_select(maxfd + 1, &rd, NULL, NULL, NULL) <= 0) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    for(int i = 0; i < s_num_maps; i++) {
      if(s_map[i].lfd < 0 || !FD_ISSET(s_map[i].lfd, &rd)) continue;
      job_t job = { lwip_accept(s_map[i].lfd, NULL, NULL), i };
      if(job.fd < 0) continue;
      if(xQueueSend(s_jobs, &job, 0) != pdTRUE) {
        debugf("TLSTUN: all sessions busy, dropping connection on port %u", s_map[i].lport);
        lwip_close(job.fd);
      }
    }
  }
}

void tlstun_init(void) {
  if(xTaskCreate(tlstun_thread, "tlstun", 1024, NULL, configMAX_PRIORITIES - 3, NULL) != pdPASS)
    debugf("TLSTUN: failed to create task");
}
