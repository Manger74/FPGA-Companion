/*
 * tlstun.h - plaintext to TLS port forwarder (stunnel-like, client mode)
 */
#ifndef TLSTUN_H
#define TLSTUN_H

#include <stdint.h>

void tlstun_init(void);
void tlstun_set_time(uint32_t utc);   // from SNTP, needed for certificate checks

#endif // TLSTUN_H
