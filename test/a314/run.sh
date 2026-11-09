#!/bin/sh
# Host tests of the A314 interface of the Companion:
#   test_a314_proto  a314_proto.c (rings, ethernet service) against a model of the core and of a314.device
#   test_a314_port   a314_port.c with real lwIP (NO_SYS=1), the byte protocol of SPI target 6 and the same model
# Needs gcc, python3 and git (or LWIP=<lwip checkout>). Shares stubs and lwipopts.h with test/plip.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
PLIP=$HERE/../plip
SRC=$HERE/../../src
OUT=${OUT:-/tmp/a314-hosttest}
mkdir -p "$OUT"

echo "== a314_proto.c (with address and undefined behaviour sanitizer)"
gcc -g -O0 -Wall -Wextra -fsanitize=address,undefined -std=gnu11 -I"$SRC" \
    "$HERE/test_a314_proto.c" "$SRC/a314_proto.c" -o "$OUT/test_a314_proto"
"$OUT/test_a314_proto"

echo "== a314_port.c with lwIP"
LWIP=${LWIP:-$OUT/lwip}
[ -d "$LWIP" ] || git clone -q --depth 1 --branch STABLE-2_2_1_RELEASE https://github.com/lwip-tcpip/lwip.git "$LWIP"

# the part of ppp_port.c that does not depend on PPP itself (peer table, hooks, proxy ARP)
python3 - "$SRC/rp2040/ppp_port.c" "$OUT/hook_only.c" <<'PY'
import sys
s = open(sys.argv[1]).read()
a = s.index("/* ---- LAN side")
b = s.index("// called by lwIP, tcpip thread\nstatic void ppp_status")
pre = '''#include "ppp_port.h"
#include <string.h>
#include "lwip/tcp.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "lwip/prot/ethernet.h"
#include "lwip/prot/etharp.h"
#include "lwip/prot/iana.h"
#include "lwip/ip4.h"
#include "lwip/ip4_frag.h"
#include "lwip/prot/tcp.h"
#include "lwip/inet_chksum.h"
#include "../debug.h"
'''
open(sys.argv[2], "w").write(pre + s[a:b])
PY

SRCS=$(ls "$LWIP"/src/core/*.c "$LWIP"/src/core/ipv4/*.c "$LWIP"/src/netif/ethernet.c |
       grep -v -e ip6 -e altcp -e dns.c -e dhcp -e autoip -e igmp)
gcc -g -O0 -Wall -Wno-unused-function -std=gnu11 -DENABLE_PPP_SERVER \
    -I"$HERE" -I"$OUT" -I"$PLIP" -I"$PLIP/stub" -I"$LWIP/contrib/ports/unix/port/include" \
    -I"$SRC/rp2040" -I"$SRC" -I"$LWIP/src/include" \
    "$HERE/test_a314_port.c" "$SRC/a314_proto.c" "$PLIP/plat.c" $SRCS -o "$OUT/test_a314_port"
"$OUT/test_a314_port"
