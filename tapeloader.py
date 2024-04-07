#! /usr/bin/python3

from boot import make_unix_machine
from machine import PDP1170
from mmio import MMIO
from op4 import op4_dispatch_table
from pdptraps import PDPTrap, PDPTraps
import sys

fh = open(sys.argv[1], 'rb')

pdp = make_unix_machine(loglevel='DEBUG')
start = None

while True:
    header = fh.read(6)

    if len(header) != 6:
        break

    count = (header[3] << 8) | header[2]
    p = (header[5] << 8) | header[4]

    csum = sum([header[i] for i in range(2, 6)])

    # print(f'{p:06o}: {count}')

    if count == 0 or p == 1:
        break

    if count == 6:
        start = p

    for i in range(count - 6):
        pbase = p & ~1
        word = pdp.mmu.wordRW(pbase, None)

        b = int.from_bytes(fh.read(1), 'big')
        csum += b

        if (p & 1) == 1:
            word = (word & 0x00ff) | (b << 8)
        else:
            word = (word & 0xff00) | b

        pdp.mmu.wordRW(pbase, word)
        p += 1

    csum += int.from_bytes(fh.read(1), 'big')  # fcs
    csum &= 255
    if csum != 255:
        print(f'Checksum failure {csum:03o} != 255')

fh.close()

if start == None:
    start = 0o0200  # assume BIC file

print(f'Start at {start:06o}')

pdp.run(pc = start)
