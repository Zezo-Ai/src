#!/usr/local/bin/python3

print("send icmp6 with hop by hop header with router alert")

import os
import sys
from struct import pack
from addr import *
from scapy.all import *

if len(sys.argv) != 2:
	print("usage: icmp6_hop_ra.py Nn")
	exit(2)

N=sys.argv[1]
IF=eval("IF_"+N);
ADDR6=eval("ADDR6_"+N);

pid=os.getpid()
eid=pid & 0xffff
payload=b"ABCDEFGHIJKLMNOP"
packet=IPv6(src=ADDR6, dst=ADDR6)/ \
    IPv6ExtHdrHopByHop(options=[Pad1(),Pad1(),RouterAlert(),\
    PadN(optdata=b"\x11\x22\x33\x44\x55\x66")])/ \
    ICMPv6Unknown(type=6, code=0, msgbody=payload)

# send() does not work for some reason, add the bpf loopback layer manually
bpf=pack('!I', 24) + bytes(packet)
sendp(bpf, iface=IF)
