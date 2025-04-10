# MIT License
#
# Copyright (c) 2023 Neil Webber
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from types import SimpleNamespace

import breakpoints as BKP
from machine import PDP1170
from unibus import BusCycle
from kw11 import KW11
from kl11 import KL11
from branches import BRANCH_CODES
from pdptraps import PDPTraps
import unittest
import random
import time
import os
import io
import hashlib
import boot

from pdpasmhelper import InstructionBlock


# subclass of KW11 to allow for selectable HZ rate
class KW11HZ(KW11):
    def __init__(self, *args, hz, **kwargs):
        self.HZ = hz
        super().__init__(*args, **kwargs)


class TestMethods(unittest.TestCase):

    PDPLOGLEVEL = 'INFO'

    # used to create various instances, collects all the options
    # detail into this one place... mostly this is about loglevel
    @classmethod
    def make_pdp(cls, memwords=64*1024, loglevel=None):
        if loglevel is None:
            loglevel = cls.PDPLOGLEVEL
        m128 = [0] * memwords
        return PDP1170(loglevel=loglevel, physmem=m128)

    @staticmethod
    def ioaddr(p, offs):
        """Given a within-IO-page IO offset, return an IO addr."""
        return (offs + p.mmu.iopage_base) & 0o177777

    # convenience routine to load word values into physical memory
    @staticmethod
    def loadphysmem(p, words, addr):
        for a, w in enumerate(words, start=(addr >> 1)):
            # make sure no bogus values get into memory
            if w < 0 or w > 65535:
                raise ValueError(f"Illegal value {w} in loadphysmem")
            p.physmem[a] = w

    # some of these can't be computed at class definition time, so...
    @classmethod
    def usefulconstants(cls):

        p = cls.make_pdp()           # meh, need this for some constants

        ns = SimpleNamespace()

        # Kernel instruction space PDR registers
        ns.KISD0 = cls.ioaddr(p, p.mmu.APR_KERNEL_OFFS)
        ns.KISD7 = ns.KISD0 + 0o16

        # Kernel data space PDR registers
        ns.KDSD0 = ns.KISD0 + 0o20
        ns.KDSD7 = ns.KDSD0 + 0o16

        # Kernel instruction space PAR registers
        ns.KISA0 = ns.KDSD0 + 0o20
        ns.KISA7 = ns.KISA0 + 0o16

        # Kernel data space PAR registers
        ns.KDSA0 = ns.KISA0 + 0o20
        ns.KDSA7 = ns.KDSA0 + 0o16

        # User mode similar
        ns.UISD0 = cls.ioaddr(p, p.mmu.APR_USER_OFFS)
        ns.UDSD0 = ns.UISD0 + 0o20
        ns.UISA0 = ns.UDSD0 + 0o20
        ns.UDSA0 = ns.UISA0 + 0o20

        ns.MMR0 = cls.ioaddr(p, p.mmu.MMR0_OFFS)
        ns.MMR3 = cls.ioaddr(p, p.mmu.MMR3_OFFS)

        return ns

    def check16(self, p):
        """Verifies no illegal values ended up in physical memory or regs."""
        for a, w in enumerate(p.physmem):
            if w < 0 or w > 65535:
                raise ValueError(f"Illegal physmem value {w} @ {oct(a<<1)}")
        for r, w in enumerate(p.r):
            if w < 0 or w > 65535:
                raise ValueError(f"Illegal reg value {w} @ {r}")

    #
    # Create and return a test machine with a simple memory mapping:
    #    Kernel Instruction space seg 0 points to physical 0
    #    Kernel Data space segment 0 also points to physical 0
    #    User instruction space seg 0 points to physical 0o20000
    #    User Data space seg 0 points to physical 0o40000
    # and turns on the MMU
    #
    # premmu is an optional list of instructions to execute
    # before turning on the MMU
    #
    # postmmu is an optional list of instructions to execute
    # after turning on the MMU
    #

    def simplemapped_pdp(self, p=None, *, premmu=[], postmmu=[]):
        if p is None:
            p = self.make_pdp()

        cn = self.usefulconstants()

        # this is a table of instructions that ...
        #  Puts the system stack at 0o20000   (8K)
        #  Puts 0o22222 into physical location 0o20000
        #  Puts 0o33333 into physical location 0o20002
        #  Puts 0o44444 into physical location 0o40000
        #  Sets Kernel Instruction space A0 to point to physical 0
        #  Sets Kernel Data space A0 to point to physical 0
        #  Sets Kernel Data space A7 to point to the IO page
        #  Sets User Instruction space A0 to point to physical 0o20000
        #  sets User Data space D0 to point to physical 0o40000
        # and turns on the MMU with I/D sep
        #
        # These instructions will be placed at 2K in memory
        #

        a = InstructionBlock()
        a.mov(0o20000, 'sp')           # start system stack at 8k

        # write the constants as described above
        a.mov(0o22222, a.ptr(0o20000))
        a.mov(0o33333, a.ptr(0o20002))
        a.mov(0o44444, a.ptr(0o40000))

        # point both kernel seg 0 PARs to physical zero
        a.clr(a.ptr(cn.KISA0))
        a.clr(a.ptr(cn.KDSA0))

        # kernel seg 7 D space PAR to I/O page (at 22-bit location)
        a.mov(0o017760000 >> 6, a.ptr(cn.KDSA0 + (7 * 2)))

        # user I seg 0 to 0o20000, user D seg 0 to 0o40000
        a.mov(0o20000 >> 6, a.ptr(cn.UISA0))
        a.mov(0o40000 >> 6, a.ptr(cn.UDSA0))

        # set the PDRs for segment zero
        a.mov(0o077406, 'r3')
        # 77406 = PDR<2:0> = ACF = 0o110 = read/write
        #         PLF<14:8> =0o0774 = full length (128*64 bytes = 8K)

        a.mov('r3', a.ptr(cn.KISD0))
        a.mov('r3', a.ptr(cn.KDSD0))
        a.mov('r3', a.ptr(cn.UISD0))
        a.mov('r3', a.ptr(cn.UDSD0))

        # PDR for segment 7
        a.mov('r3', a.ptr(cn.KDSD0 + (7 * 2)))

        # set previous mode to USER, keeping current mode KERNEL, pri 7
        a.mov((p.KERNEL << 14) | (p.USER << 12) | (7 << 5),
              a.ptr(self.ioaddr(p, p.PS_OFFS)))

        # turn on 22-bit mode, unibus mapping, and I/D sep for k & u
        a.mov(0o000065, a.ptr(cn.MMR3))

        # Instructions supplied by caller, to be executed before
        # enabling the MMU. They are "literals" since they have
        # already been assembled.
        for w in premmu:
            a.literal(w)

        # turn on relocation mode ...
        a.inc(a.ptr(cn.MMR0))

        # and the post-MMU instructions
        for w in postmmu:
            a.literal(w)
        a.halt()

        instloc = 0o4000             # 2K
        self.loadphysmem(p, a, instloc)
        return p, instloc

    def u64mapped_pdp(self, p=None, /, *pdpargs, **pdpkwargs):
        # Make a PDP11 with:
        #   * kernel space mapped to first 56K of memory (straight through)
        #   * top of kernel space mapped to I/O page
        #   * 64KB of user space mapped to phys 64K-128K
        #
        # No I/D separation.
        #
        if p is None:
            p = self.make_pdp(*pdpargs, **pdpkwargs)

        cn = self.usefulconstants()

        a = InstructionBlock()
        a.mov(0o160000, 'sp')           # start system stack at top of K mem

        # kernel segs 0 .. 6 to physical zero .. 56K
        a.mov(cn.KISA0, 'r4')
        for i in range(7):
            a.mov((i * 8192) >> 6, '(r4)+')

        # kernel seg 7 to I/O page
        a.mov(0o760000 >> 6, '(r4)')

        # user segs 0 .. 7 to physical 64K .. 128K
        a.mov(cn.UISA0, 'r4')
        for i in range(8):
            a.mov((65536 + (i * 8192)) >> 6, '(r4)+')

        # set K and U PDRs
        # 77406 = PDR<2:0> = ACF = 0o110 = read/write
        #         PLF<14:8> =0o0774 = full length (128*64 bytes = 8K)

        a.mov(0o77406, 'r0')

        a.mov(cn.KISD0, 'r4')
        for i in range(8):
            a.mov('r0', '(r4)+')

        a.mov(cn.UISD0, 'r4')
        for i in range(8):
            a.mov('r0', '(r4)+')

        # turn on relocation mode ...
        a.inc(a.ptr(cn.MMR0))

        # and halt
        a.literal(0)

        instloc = 0o4000             # 2K
        self.loadphysmem(p, a, instloc)
        return p, instloc

    # test a typical sequence to set up a stack
    # This is really a test of label support in pdpasmhelper
    def test_fwdlabrel(self):
        a = InstructionBlock()
        a.clr('r0')     # just to have something here
        # typical sequence to put stack at end of code
        a.mov('pc', 'r0')
        a.add(a.getlabel('stack', idxrel=True), 'r0')
        a.mov('r0', 'sp')
        a.halt()
        a.literal(0o111111)
        a.literal(0o177777)
        a.label('stack')

        p = self.make_pdp()
        self.loadphysmem(p, a, 0o10000)
        p.run(pc=0o10000)
        self.check16(p)
        self.assertEqual(p.mmu.wordRW(p.r[6] - 2), 0o177777)
        self.assertEqual(p.mmu.wordRW(p.r[6] - 4), 0o111111)

    # these tests end up testing other stuff too of course, including MMU
    def test_mfpi(self):

        tvecs = []

        for result, r1tval in ((0o33333, 2), (0o22222, 0)):
            # r1=r1tval, mfpi (r1) -> r0; expect r0 = result
            a = InstructionBlock()
            a.mov(r1tval, 'r1')
            a.mfpi('(r1)')
            a.mov('(sp)+', 'r0')
            tvecs.append((result, list(a)),)

        for result, insts in tvecs:
            with self.subTest(result=result, insts=insts):
                p, pc = self.simplemapped_pdp(postmmu=insts)
                p.run(pc=pc)
                self.check16(p)
                self.assertEqual(p.r[0], result)

    def test_mfpxsp(self):
        cn = self.usefulconstants()

        u = InstructionBlock()
        u.mov('r2', 'r6')
        u.trap(0)

        premmu = InstructionBlock()
        premmu.mov(0o14000, premmu.ptr(0o34))  # set vector 034 to 14000
        premmu.clr(premmu.ptr(0o36))           # PSW for trap - zero work
        premmu.mov(0o20000, 'r0')

        for uinst in u:
            premmu.mov(uinst, '(r0)+')
        premmu.mov(0o123456, 'r2')
        premmu.mov(0o140340, '-(sp)')      # push user-ish PSW to K stack
        premmu.clr('-(sp)')                # new user PC = 0

        postmmu = InstructionBlock()
        postmmu.rtt()                      # RTT - goes to user mode, addr 0

        p, pc = self.simplemapped_pdp(premmu=premmu, postmmu=postmmu)

        # put the trap handler at 14000 as expected
        th = InstructionBlock()
        th.mfpd('sp')
        th.mov('(sp)+', 'r3')
        th.halt()
        self.loadphysmem(p, th, 0o14000)
        p.run(pc=pc)
        self.check16(p)
        self.assertEqual(p.r[2], p.r[3])

    def test_mtpi(self):
        cn = self.usefulconstants()

        ts = InstructionBlock()
        ts.mov(0o1717, '-(sp)')        # pushing 0o1717
        ts.mtpi(ts.ptr(0o02))          # and MTPI it to user location 2
        ts.clr(ts.ptr(cn.MMR0))        # turn MMU back off
        ts.mov(ts.ptr(0o20002), 'r0')  # r0 = (020002)

        tvecs = ((0o1717, ts),)

        for r0result, insts in tvecs:
            with self.subTest(r0result=r0result, insts=insts):
                p, pc = self.simplemapped_pdp(postmmu=insts)
                p.run(pc=pc)
                self.check16(p)
                self.assertEqual(p.r[0], r0result)

    def test_psw(self):
        p = self.make_pdp()

        # the test is assembled from:
        #         .org 4000
        #         halt = 0
        #         mov $20000,sp   / establish a stack on general principles
        #         clr r0               / clears all PSW bits except Z
        #         inc r0               / clears Z
        #         sen
        #         bmi 1f
        #         mov $66610,r2
        #         halt
        # 1:      cln
        #         bpl 1f
        #         mov $66650,r2
        #         halt
        # 1:      clr r0
        #         inc r0
        #         sez
        #         beq 1f
        #         mov $66604,r2
        #         halt
        # 1:      clz
        #         bne 1f
        #         mov $66644,r2
        #         halt
        # 1:      clr r0
        #         inc r0
        #         sev
        #         bvs 1f
        #         mov $66602,r2
        #         halt
        # 1:      clv
        #         bvc 1f
        #         mov $66642,r2
        #         halt
        # 1:      clr r0
        #         inc r0
        #         sec
        #         bcs 1f
        #         mov $66601,r2
        #         halt
        # 1:      clc
        #         bcc 1f
        #         mov $66641,r2
        #         halt
        # 1:      mov $77, r2
        #         halt

        insts = [0o12706, 0o20000, 0o5000, 0o5200, 0o270, 0o100403, 0o12702,
                 0o66610, 0o0, 0o250, 0o100003, 0o12702, 0o66650, 0o0,
                 0o5000, 0o5200, 0o264, 0o1403, 0o12702, 0o66604, 0o0,
                 0o244, 0o1003, 0o12702, 0o66644, 0o0, 0o5000, 0o5200, 0o262,
                 0o102403, 0o12702, 0o66602, 0o0, 0o242, 0o102003, 0o12702,
                 0o66642, 0o0, 0o5000, 0o5200, 0o261, 0o103403, 0o12702,
                 0o66601, 0o0, 0o241, 0o103003, 0o12702, 0o66641, 0o0,
                 0o12702, 0o77, 0o0]

        self.loadphysmem(p, insts, 0o4000)
        p.run(pc=0o4000)
        self.assertEqual(p.r[2], 0o77)

    def test_add_sub(self):
        p = self.make_pdp()

        testvecs = (
            # (op0, op1, expected op0 + op1, nzvc, expected op0 - op1, nzvc)
            # None for nzvc means dont test that (yet/for-now/need to verify)
            (1, 1, 2, 0, 0, 4),        # 1 + 1 = 2(_); 1 - 1 = 0(Z)
            (1, 32767, 32768, 0o12, 32766, 0),
            (0, 0, 0, 0o04, 0, 0o04),
            (32768, 1, 32769, 0o10, 32769, 0o13),
            (65535, 1, 0, 0o05, 2, 1),
        )

        testloc = 0o10000
        add_loc = testloc
        sub_loc = testloc + 4

        for addsub, loc in (('add', add_loc), ('sub', sub_loc)):
            a = InstructionBlock()
            getattr(a, addsub)('r0', 'r1')
            a.halt()
            self.loadphysmem(p, a, loc)

        for r0, r1, added, a_nzvc, subbed, s_nzvc in testvecs:
            with self.subTest(r0=r0, r1=r1, op="add"):
                p.r[0] = r0
                p.r[1] = r1
                p.run(pc=add_loc)
                self.check16(p)
                self.assertEqual(p.r[1], added)
                if a_nzvc is not None:
                    self.assertEqual(p.psw & 0o17, a_nzvc)

            with self.subTest(r0=r0, r1=r1, op="sub"):
                p.r[0] = r0
                p.r[1] = r1
                p.run(pc=sub_loc)
                self.check16(p)
                self.assertEqual(p.r[1], subbed)
                if s_nzvc is not None:
                    self.assertEqual(p.psw & 0o17, s_nzvc)

    # test BNE (and, implicitly, INC/DEC)
    def test_bne(self):
        p = self.make_pdp()
        loopcount = 0o1000

        a = InstructionBlock()
        a.mov(loopcount, 'r1')
        a.clr('r0')
        a.label('LOOP')
        a.inc('r0')
        a.dec('r1')
        a.bne('LOOP')
        a.halt()

        instloc = 0o4000
        self.loadphysmem(p, a, instloc)

        p.run(pc=instloc)
        self.check16(p)
        self.assertEqual(p.r[0], loopcount)
        self.assertEqual(p.r[1], 0)

    def test_neg(self):
        # test results verified with SIMH
        # r0 test value, r0 result value, n, z, v, c
        testvectors = (
            (0, 0, False, True, False, False),
            (1, 0o177777, True, False, False, True),
            (0o100000, 0o100000, True, False, True, True),
            (0o177777, 1, False, False, False, True))

        p = self.make_pdp()
        instloc = 0o4000
        a = InstructionBlock()
        a.neg('r0')
        a.halt()
        self.loadphysmem(p, a, instloc)
        for r0_in, r0_out, n, z, v, c in testvectors:
            with self.subTest(r0_in=r0_in):
                p.r[0] = r0_in
                p.run(pc=instloc)
                self.check16(p)
                self.assertEqual(p.r[0], r0_out)
                self.assertEqual(bool(p.psw_n), n)
                self.assertEqual(bool(p.psw_z), z)
                self.assertEqual(bool(p.psw_v), v)
                self.assertEqual(bool(p.psw_c), c)

    def test_negb(self):
        # test results verified with SIMH
        # r0 test value, r0 result value, n, z, v, c
        testvectors = (
            (0, 0, False, True, False, False),
            (1, 0o377, True, False, False, True),
            (0o200, 0o200, True, False, True, True),
            (0o377, 1, False, False, False, True),
            (0o400, 0o400, False, True, False, False),
            (0o401, 0o777, True, False, False, True),
        )

        p = self.make_pdp()
        instloc = 0o4000
        a = InstructionBlock()
        a.negb('r0')
        a.halt()
        self.loadphysmem(p, a, instloc)
        for r0_in, r0_out, n, z, v, c in testvectors:
            with self.subTest(r0_in=r0_in):
                p.r[0] = r0_in
                p.run(pc=instloc)
                self.check16(p)
                self.assertEqual(p.r[0], r0_out)
                self.assertEqual(bool(p.psw_n), n)
                self.assertEqual(bool(p.psw_z), z)
                self.assertEqual(bool(p.psw_v), v)
                self.assertEqual(bool(p.psw_c), c)

    # test BEQ and BNE (BNE was also tested in test_bne)
    def test_eqne(self):
        p = self.make_pdp()

        goodval = 0o4321            # arbitrary, not zero
        a = InstructionBlock()
        a.clr('r1')             # if successful r1 will become goodval
        a.clr('r0')
        a.beq('good')
        a.halt()                # stop here if BEQ fails
        a.label('good')
        a.literal(0o000257)     # 1f: CCC .. clear all the condition codes
        a.bne('good2')
        a.halt()                # stop here if BNE fails
        a.label('good2')
        a.mov(goodval, 'r1')    # indicate success
        a.halt()

        instloc = 0o4000
        self.loadphysmem(p, a, instloc)
        p.run(pc=instloc)
        self.check16(p)
        self.assertEqual(p.r[1], goodval)

    # create the instruction sequence shared by test_cc and test_ucc
    def _cc_unscc(self, br1, br2):
        a = InstructionBlock()
        # program is:
        #       CLR R0
        #       MOV @#05000,R1      ; see discussion below
        #       MOV @#05002,R2      ; see discussion below
        #       CMP R1,R2
        #       br1 1f              ; see discussion
        #       HALT
        #    1: DEC R0
        #       CMP R2,R1
        #       br2 1f              ; see discussion
        #       HALT
        #    1: DEC R0
        #       HALT
        #
        # The test_cc and test_unscc tests will poke various test
        # cases into locations 5000 and 5002, knowing the order of
        # the operands in the two CMP instructions and choosing
        # test cases and br1/br2 accordingly.
        #
        # If the program makes it to the end R0 will be 65554 (-2)

        a.clr('r0')
        a.mov(a.ptr(0o5000), 'r1')
        a.mov(a.ptr(0o5002), 'r2')
        a.cmp('r1', 'r2')
        a.literal((br1 & 0o177400) | 1)   # br1 1f
        a.halt()
        a.dec('r0')
        a.cmp('r2', 'r1')
        a.literal((br2 & 0o177400) | 1)   # br2 1f
        a.halt()
        a.dec('r0')
        a.halt()
        return a

    def test_cc(self):
        # various condition code tests
        p = self.make_pdp()

        insts = self._cc_unscc(BRANCH_CODES['blt'], BRANCH_CODES['bgt'])

        instloc = 0o4000
        self.loadphysmem(p, insts, instloc)

        # just a convenience so the test data can use neg numbers
        def s2c(x):
            return x & 0o177777

        for lower, higher in ((0, 1), (s2c(-1), 0), (s2c(-1), 1),
                              (s2c(-32768), 32767),
                              (s2c(-32768), 0), (s2c(-32768), 32767),
                              (17, 42), (s2c(-42), s2c(-17))):
            p.physmem[0o5000 >> 1] = lower
            p.physmem[0o5002 >> 1] = higher
            with self.subTest(lower=lower, higher=higher):
                p.run(pc=instloc)
                self.check16(p)
                self.assertEqual(p.r[0], 65534)

        # probably never a good idea, but ... do some random values
        for randoms in range(1000):
            a = random.randint(-32768, 32767)
            b = random.randint(-32768, 32767)
            while a == b:
                b = random.randint(-32768, 32767)
            if a > b:
                a, b = b, a
            p.physmem[0o5000 >> 1] = s2c(a)
            p.physmem[0o5002 >> 1] = s2c(b)
            with self.subTest(lower=a, higher=b):
                p.run(pc=instloc)
                self.assertEqual(p.r[0], 65534)

    def test_mode3(self):
        # @(Rn)+ mode
        instloc = 0o4000
        ptrloc = 0o020000
        dataloc = 0o030000
        bval = 0o42
        a = InstructionBlock()
        a.mov(ptrloc, 'r0')
        a.mov(dataloc, a.ptr(ptrloc))
        a.movb(bval, '@(r0)+')
        a.halt()
        p = self.make_pdp()
        self.loadphysmem(p, a, instloc)
        p.run(pc=instloc)
        self.check16(p)
        self.assertEqual(p.physmem[dataloc >> 1], bval)
        self.assertEqual(p.r[0], ptrloc+2)

    def test_mode5(self):
        # @-(Rn) mode
        instloc = 0o4000
        ptrloc = 0o020000
        dataloc = 0o030000
        bval = 0o42
        a = InstructionBlock()
        a.mov(ptrloc+2, 'r0')            # +2 because pre-decrement
        a.mov(dataloc, a.ptr(ptrloc))
        a.movb(bval, '@-(r0)')
        a.halt()
        p = self.make_pdp()
        self.loadphysmem(p, a, instloc)
        p.run(pc=instloc)
        self.check16(p)
        self.assertEqual(p.physmem[dataloc >> 1], bval)
        self.assertEqual(p.r[0], ptrloc)

    def test_swab(self):
        # mostly about testing the N bit which behaves... this way
        instloc = 0o4000
        a = InstructionBlock()
        a.mov(0o377, 'r0')
        a.swab('r0')
        a.halt()
        a.swab('r0')
        a.halt()
        p = self.make_pdp()
        self.loadphysmem(p, a, instloc)
        p.run(pc=instloc)
        self.check16(p)
        self.assertEqual(p.r[0], 0o377 << 8)
        self.assertFalse(p.psw_n)
        p.run()           # resume after first halt
        self.check16(p)
        self.assertEqual(p.r[0], 0o377)
        self.assertTrue(p.psw_n)

    def test_unscc(self):
        # more stuff like test_cc but specifically testing unsigned Bxx codes
        p = self.make_pdp()

        insts = self._cc_unscc(BRANCH_CODES['blo'], BRANCH_CODES['bhi'])
        instloc = 0o4000
        self.loadphysmem(p, insts, instloc)

        for lower, higher in ((0, 1), (0, 65535), (32768, 65535),
                              (65534, 65535),
                              (32767, 32768),
                              (17, 42)):
            p.physmem[0o5000 >> 1] = lower
            p.physmem[0o5002 >> 1] = higher
            with self.subTest(lower=lower, higher=higher):
                p.run(pc=instloc)
                self.check16(p)
                self.assertEqual(p.r[0], 65534)

        # probably never a good idea, but ... do some random values
        for randoms in range(1000):
            a = random.randint(0, 65535)
            b = random.randint(0, 65535)
            while a == b:
                b = random.randint(0, 65535)
            if a > b:
                a, b = b, a
            p.physmem[0o5000 >> 1] = a
            p.physmem[0o5002 >> 1] = b
            with self.subTest(lower=a, higher=b):
                p.run(pc=instloc)
                self.check16(p)
                self.assertEqual(p.r[0], 65534)

    def test_ash1(self):
        # this code sequence taken from Unix startup, it's not really
        # much of a test.
        a = InstructionBlock()
        a.mov(0o0122451, 'r2')
        neg6 = -6 & 0xFFFF
        a.ash(neg6, 'r2')
        a.bic(0o0176000, 'r2')
        a.halt()

        p = self.make_pdp()
        instloc = 0o4000
        self.loadphysmem(p, a, instloc)
        p.run(pc=instloc)
        self.check16(p)
        self.assertEqual(p.r[2], 0o1224)

    # this is not invoked automatically but performs all possible
    # 16 bit ASH values crossed with all possible 6-bit shift counts.
    def exhaustive_ash(self):
        p = self.make_pdp()
        a = InstructionBlock()

        a.ash('r4', 'r0')             # assumes r4 is shf, r0 is value
        a.halt()
        instloc = 0o10000
        self.loadphysmem(p, a, instloc)

        # left shifts
        for shf in range(1, 32):
            # these are the sign bit and all the "higher than 16 bits) bits
            # expected as a result of shifting left shf places
            vbits = (((1 << shf) - 1) << 16) | 0x8000
            p.r[4] = shf
            for value in range(32768):

                # positive value test
                p.r[0] = value
                p.run(pc=instloc)
                result = value << shf
                overflow = (result & vbits)
                result &= 0xFFFF
                with self.subTest(value=value, shf=shf):
                    self.assertEqual(p.r[0], result)
                    self.assertEqual(bool(p.psw_v), bool(overflow))

                # negative value test, nearly identical except for overflow
                value += 32768
                p.r[0] = value
                p.run(pc=instloc)
                result = value << shf
                overflow = ((result & vbits) != vbits)
                result &= 0xFFFF
                with self.subTest(value=value, shf=shf):
                    self.assertEqual(p.r[0], result)
                    self.assertEqual(bool(p.psw_v), bool(overflow))

        # right shifts, similar but different
        for shf in range(32, 64):
            p.r[4] = shf
            for value in range(32768):
                # positive value test
                p.r[0] = value
                p.run(pc=instloc)
                result = value >> (64 - shf)
                result &= 0xFFFF
                with self.subTest(value=value, shf=shf):
                    self.assertEqual(p.r[0], result)
                    self.assertEqual(bool(p.psw_v), False)

                # negative value test
                value += 32768
                p.r[0] = value
                p.run(pc=instloc)

                # These are the extra sign extension bits
                vbits = (((1 << shf) - 1) << 16) | 0x8000

                result = (vbits | value) >> (64 - shf)
                result &= 0xFFFF
                with self.subTest(value=value, shf=shf):
                    self.assertEqual(p.r[0], result)
                    self.assertEqual(bool(p.psw_v), False)

    def test_ash_psw_v(self):

        testvectors = (
            # (value, shift count, result, psw_v
            (0o100000, 1, 0, True),
            (0o040001, 2, 4, True),         # Issue #22
            (0o050001, 2, 16384+4, True),   # also Issue #22
            (0o177777, 1, 65534, False),
            (0o177777, 15, 32768, False),
            (0o177777, 16, 0, True),
            (0o120000, 2, 32768, True),
            (0o140000, 1, 32768, False),
            (0o100001, 2, 4, True),
            (0o40001, 2, 4, True),
        )

        for value, shf, result, psw_v in testvectors:
            a = InstructionBlock()
            a.mov(value, 'r0')
            a.ash(shf, 'r0')
            a.halt()

            p = self.make_pdp()
            instloc = 0o10000
            self.loadphysmem(p, a, instloc)
            p.run(pc=instloc)
            with self.subTest(value=value, shf=shf):
                self.assertEqual(p.r[0], result)
                self.assertEqual(bool(p.psw_v), bool(psw_v))

    def test_ashc_ror(self):
        # test that ashc with odd register and right shift == rotate
        # (as per pdp11 processor handbook)
        a = InstructionBlock()
        a.literal(0o73104)        # a.ashc('r4', 'r1')
        a.halt()

        p = self.make_pdp()
        instloc = 0o10000
        self.loadphysmem(p, a, instloc)
        p.r[4] = 0o77

        p.r[1] = 5
        p.run(pc=instloc)
        self.assertEqual(p.r[1], 0o100002)
        p.run(pc=instloc)
        self.assertEqual(p.r[1], 0o040001)
        p.run(pc=instloc)
        self.assertEqual(p.r[1], 0o120000)

    def test_shiftb(self):
        # test correct operation of byte operations on registers
        # r2 counts test progress
        a = InstructionBlock()
        a.clr('r2')
        a.mov(0o177401, 'r0')
        a.literal(0o106300)        # ASLB R0
        a.cmp(0o177402, 'r0')
        a.bne('fail')
        a.inc('r2')
        a.halt()
        a.label('fail')
        a.halt()
        p = self.make_pdp()
        instloc = 0o4000
        self.loadphysmem(p, a, instloc)
        p.run(pc=instloc)
        self.check16(p)
        self.assertEqual(p.r[2], 1)

    def test_byteops(self):
        # more tests of various byte operations
        # r2 counts test progress
        a = InstructionBlock()
        a.clr('r2')
        a.mov(0o177400, 'r0')
        a.tstb('r0')
        a.bne('fail')
        a.inc('r2')
        a.decb('r0')
        a.cmp(0o0177777, 'r0')
        a.bne('fail')
        a.inc('r2')
        a.halt()
        a.label('fail')
        a.halt()
        p = self.make_pdp()
        instloc = 0o4000
        self.loadphysmem(p, a, instloc)
        p.run(pc=instloc)
        self.check16(p)
        self.assertEqual(p.r[2], 2)

    def test_br(self):
        # though the bug has been fixed, this is a test of whether
        # all branch offset values work correctly. Barn door shut...
        p = self.make_pdp()

        # the idea is a block of INC R0 instructions
        # followed by a halt, then a spot for a branch
        # then a block of INC R1 instructions followed by a halt
        #
        # By tweaking the BR instruction (different forward/back offsets)
        # and starting execution at the BR, the result on R0 and R1
        # will show if the correct branch offset was effected.
        #
        # NOTE: 0o477 (branch offset -1) is a tight-loop branch to self
        #             and that case is tested separately.
        #
        insts = [0o5200] * 300    # 300 INC R0 instructions
        insts += [0]              # 1 HALT instruction
        insts += [0o477]          # BR instruction .. see below

        # want to know where in memory this br will is
        brspot = len(insts) - 1

        insts += [0o5201] * 300   # 300 INC R1 instructions
        insts += [0]              # 1 HALT instruction

        # put that mess into memory at an arbitrary spot
        baseloc = 0o10000
        self.loadphysmem(p, insts, baseloc)

        # test the negative offsets:
        #  Set R0 to 65535 (-1)
        #  Set R1 to 17
        #   -1 is a special case, that's the tight loop and not tested here
        #   -2 reaches the HALT instruction only, R0 will remain 65535
        #   -3 reaches back to one INC R0, R0 will be 0
        #   -4 reaches back two INC R0's, R0 will be 1
        # and so on

        # 0o400 | offset starting at 0o376 will be the BR -2 case
        expected_R0 = 65535
        for offset in range(0o376, 0o200, -1):
            p.physmem[(baseloc >> 1) + brspot] = (0o400 | offset)
            p.r[0] = 65535
            p.r[1] = 17

            # note the 2* because PC is an addr vs physmem word index
            p.run(pc=baseloc + (2*brspot))
            self.check16(p)

            with self.subTest(offset=offset):
                self.assertEqual(p.r[0], expected_R0)
                self.assertEqual(p.r[1], 17)
            expected_R0 = (expected_R0 + 1) & 0o177777

        # and the same sort of test but with forward branching

        expected_R1 = 42 + 300
        for offset in range(0, 0o200):
            p.physmem[(baseloc >> 1) + brspot] = (0o400 | offset)
            p.r[0] = 17
            p.r[1] = 42

            # note the 2* because PC is an addr vs physmem word index
            p.run(pc=baseloc + (2*brspot))
            self.check16(p)

            with self.subTest(offset=offset):
                self.assertEqual(p.r[0], 17)
                self.assertEqual(p.r[1], expected_R1)
            expected_R1 = (expected_R1 - 1) & 0o177777

    def test_mul(self):
        # huge table of data generated via SIMH
        # each entry is X (16 bits), Y (16), X*Y (2 x 16 bits)
        simhdata = [
            0o177777, 0o177777, 0, 1, 0o177776, 0o177777, 0,
            2, 0o177000, 0o177777, 0, 0o001000, 0o176543, 0o177777, 0,
            0o001235, 0o105432, 0o177777, 0, 0o072346, 0o100420, 0o177777, 0,
            0o077360, 0o100002, 0o177777, 0, 0o077776, 0o100001, 0o177777, 0,
            0o077777, 0o100000, 0o177777, 0, 0o100000, 0o077777, 0o177777,
            0o177777, 0o100001, 0o077776, 0o177777, 0o177777, 0o100002,
            0o077775, 0o177777, 0o177777, 0o100003, 0o054321, 0o177777,
            0o177777, 0o123457, 0o043210, 0o177777, 0o177777, 0o134570,
            0o032412, 0o177777, 0o177777, 0o145366, 7, 0o177777, 0o177777,
            0o177771, 6, 0o177777, 0o177777, 0o177772, 5, 0o177777, 0o177777,
            0o177773, 4, 0o177777, 0o177777, 0o177774, 3, 0o177777, 0o177777,
            0o177775, 2, 0o177777, 0o177777, 0o177776, 1, 0o177777, 0o177777,
            0o177777, 0o177777, 0o177776, 0, 2, 0o177776, 0o177776, 0, 4,
            0o177000, 0o177776, 0, 0o002000, 0o176543, 0o177776, 0, 0o002472,
            0o105432, 0o177776, 0, 0o164714, 0o100420, 0o177776, 0, 0o176740,
            0o100002, 0o177776, 0, 0o177774, 0o100001, 0o177776, 0, 0o177776,
            0o100000, 0o177776, 1, 0, 0o077777, 0o177776, 0o177777, 2,
            0o077776, 0o177776, 0o177777, 4, 0o077775, 0o177776, 0o177777, 6,
            0o054321, 0o177776, 0o177777, 0o047136, 0o043210, 0o177776,
            0o177777, 0o071360, 0o032412, 0o177776, 0o177777, 0o112754, 7,
            0o177776, 0o177777, 0o177762, 6, 0o177776, 0o177777, 0o177764, 5,
            0o177776, 0o177777, 0o177766, 4, 0o177776, 0o177777, 0o177770, 3,
            0o177776, 0o177777, 0o177772, 2, 0o177776, 0o177777, 0o177774, 1,
            0o177776, 0o177777, 0o177776, 0o177777, 0o177000, 0, 0o001000,
            0o177776, 0o177000, 0, 0o002000, 0o177000, 0o177000, 4, 0,
            0o176543, 0o177000, 5, 0o035000, 0o105432, 0o177000, 0o000351,
            0o146000, 0o100420, 0o177000, 0o000375, 0o160000, 0o100002,
            0o177000, 0o000377, 0o176000, 0o100001, 0o177000, 0o000377,
            0o177000, 0o100000, 0o177000, 0o000400, 0, 0o077777, 0o177000,
            0o177400, 0o001000, 0o077776, 0o177000, 0o177400, 0o002000,
            0o077775, 0o177000, 0o177400, 0o003000, 0o054321, 0o177000,
            0o177516, 0o057000, 0o043210, 0o177000, 0o177562, 0o170000,
            0o032412, 0o177000, 0o177625, 0o166000, 7, 0o177000, 0o177777,
            0o171000, 6, 0o177000, 0o177777, 0o172000, 5, 0o177000, 0o177777,
            0o173000, 4, 0o177000, 0o177777, 0o174000, 3, 0o177000, 0o177777,
            0o175000, 2, 0o177000, 0o177777, 0o176000, 1, 0o177000, 0o177777,
            0o177000, 0o177777, 0o176543, 0, 0o001235, 0o177776, 0o176543, 0,
            0o002472, 0o177000, 0o176543, 5, 0o035000, 0o176543, 0o176543, 6,
            0o152111, 0o105432, 0o176543, 0o000461, 0o076416, 0o100420,
            0o176543, 0o000513, 0o134460, 0o100002, 0o176543, 0o000516,
            0o075306, 0o100001, 0o176543, 0o000516, 0o076543, 0o100000,
            0o176543, 0o000516, 0o100000, 0o077777, 0o176543, 0o177261,
            0o101235, 0o077776, 0o176543, 0o177261, 0o102472, 0o077775,
            0o176543, 0o177261, 0o103727, 0o054321, 0o176543, 0o177427,
            0o162723, 0o043210, 0o176543, 0o177507, 0o127230, 0o032412,
            0o176543, 0o177565, 0o062336, 7, 0o176543, 0o177777, 0o166665, 6,
            0o176543, 0o177777, 0o170122, 5, 0o176543, 0o177777, 0o171357, 4,
            0o176543, 0o177777, 0o172614, 3, 0o176543, 0o177777, 0o174051, 2,
            0o176543, 0o177777, 0o175306, 1, 0o176543, 0o177777, 0o176543,
            0o177777, 0o105432, 0, 0o072346, 0o177776, 0o105432, 0, 0o164714,
            0o177000, 0o105432, 0o000351, 0o146000, 0o176543, 0o105432,
            0o000461, 0o076416, 0o105432, 0o105432, 0o032541, 0o037244,
            0o100420, 0o105432, 0o034766, 0o145640, 0o100002, 0o105432,
            0o035162, 0o013064, 0o100001, 0o105432, 0o035162, 0o105432,
            0o100000, 0o105432, 0o035163, 0, 0o077777, 0o105432, 0o142615,
            0o072346, 0o077776, 0o105432, 0o142615, 0o164714, 0o077775,
            0o105432, 0o142616, 0o057262, 0o054321, 0o105432, 0o153561,
            0o100072, 0o043210, 0o105432, 0o157713, 0o000720, 0o032412,
            0o105432, 0o163707, 0o150404, 7, 0o105432, 0o177774, 0o146666, 6,
            0o105432, 0o177775, 0o041234, 5, 0o105432, 0o177775, 0o133602, 4,
            0o105432, 0o177776, 0o026150, 3, 0o105432, 0o177776, 0o120516, 2,
            0o105432, 0o177777, 0o013064, 1, 0o105432, 0o177777, 0o105432,
            0o177777, 0o100420, 0, 0o077360, 0o177776, 0o100420, 0, 0o176740,
            0o177000, 0o100420, 0o000375, 0o160000, 0o176543, 0o100420,
            0o000513, 0o134460, 0o105432, 0o100420, 0o034766, 0o145640,
            0o100420, 0o100420, 0o037361, 0o020400, 0o100002, 0o100420,
            0o037567, 0o001040, 0o100001, 0o100420, 0o037567, 0o100420,
            0o100000, 0o100420, 0o037570, 0, 0o077777, 0o100420, 0o140210,
            0o077360, 0o077776, 0o100420, 0o140210, 0o176740, 0o077775,
            0o100420, 0o140211, 0o076320, 0o054321, 0o100420, 0o151765,
            0o157020, 0o043210, 0o100420, 0o156406, 0o170200, 0o032412,
            0o100420, 0o162663, 0o055240, 7, 0o100420, 0o177774, 0o103560, 6,
            0o100420, 0o177775, 0o003140, 5, 0o100420, 0o177775, 0o102520, 4,
            0o100420, 0o177776, 0o002100, 3, 0o100420, 0o177776, 0o101460, 2,
            0o100420, 0o177777, 0o001040, 1, 0o100420, 0o177777, 0o100420,
            0o177777, 0o100002, 0, 0o077776, 0o177776, 0o100002, 0, 0o177774,
            0o177000, 0o100002, 0o000377, 0o176000, 0o176543, 0o100002,
            0o000516, 0o075306, 0o105432, 0o100002, 0o035162, 0o013064,
            0o100420, 0o100002, 0o037567, 0o001040, 0o100002, 0o100002,
            0o037776, 4, 0o100001, 0o100002, 0o037776, 0o100002, 0o100000,
            0o100002, 0o037777, 0, 0o077777, 0o100002, 0o140001, 0o077776,
            0o077776, 0o100002, 0o140001, 0o177774, 0o077775, 0o100002,
            0o140002, 0o077772, 0o054321, 0o100002, 0o151630, 0o030642,
            0o043210, 0o100002, 0o156274, 0o106420, 0o032412, 0o100002,
            0o162573, 0o065024, 7, 0o100002, 0o177774, 0o100016, 6, 0o100002,
            0o177775, 0o000014, 5, 0o100002, 0o177775, 0o100012, 4, 0o100002,
            0o177776, 0o000010, 3, 0o100002, 0o177776, 0o100006, 2, 0o100002,
            0o177777, 4, 1, 0o100002, 0o177777, 0o100002, 0o177777, 0o100001,
            0, 0o077777, 0o177776, 0o100001, 0, 0o177776, 0o177000, 0o100001,
            0o000377, 0o177000, 0o176543, 0o100001, 0o000516, 0o076543,
            0o105432, 0o100001, 0o035162, 0o105432, 0o100420, 0o100001,
            0o037567, 0o100420, 0o100002, 0o100001, 0o037776, 0o100002,
            0o100001, 0o100001, 0o037777, 1, 0o100000, 0o100001, 0o037777,
            0o100000, 0o077777, 0o100001, 0o140000, 0o177777, 0o077776,
            0o100001, 0o140001, 0o077776, 0o077775, 0o100001, 0o140001,
            0o177775, 0o054321, 0o100001, 0o151627, 0o154321, 0o043210,
            0o100001, 0o156274, 0o043210, 0o032412, 0o100001, 0o162573,
            0o032412, 7, 0o100001, 0o177774, 0o100007, 6, 0o100001, 0o177775,
            6, 5, 0o100001, 0o177775, 0o100005, 4, 0o100001, 0o177776, 4, 3,
            0o100001, 0o177776, 0o100003, 2, 0o100001, 0o177777, 2, 1,
            0o100001, 0o177777, 0o100001, 0o177777, 0o100000, 0, 0o100000,
            0o177776, 0o100000, 1, 0, 0o177000, 0o100000, 0o000400, 0,
            0o176543, 0o100000, 0o000516, 0o100000, 0o105432, 0o100000,
            0o035163, 0, 0o100420, 0o100000, 0o037570, 0, 0o100002, 0o100000,
            0o037777, 0, 0o100001, 0o100000, 0o037777, 0o100000, 0o100000,
            0o100000, 0o040000, 0, 0o077777, 0o100000, 0o140000, 0o100000,
            0o077776, 0o100000, 0o140001, 0, 0o077775, 0o100000, 0o140001,
            0o100000, 0o054321, 0o100000, 0o151627, 0o100000, 0o043210,
            0o100000, 0o156274, 0, 0o032412, 0o100000, 0o162573, 0, 7,
            0o100000, 0o177774, 0o100000, 6, 0o100000, 0o177775, 0, 5,
            0o100000, 0o177775, 0o100000, 4, 0o100000, 0o177776, 0, 3,
            0o100000, 0o177776, 0o100000, 2, 0o100000, 0o177777, 0, 1,
            0o100000, 0o177777, 0o100000, 0o177777, 0o077777, 0o177777,
            0o100001, 0o177776, 0o077777, 0o177777, 2, 0o177000, 0o077777,
            0o177400, 0o001000, 0o176543, 0o077777, 0o177261, 0o101235,
            0o105432, 0o077777, 0o142615, 0o072346, 0o100420, 0o077777,
            0o140210, 0o077360, 0o100002, 0o077777, 0o140001, 0o077776,
            0o100001, 0o077777, 0o140000, 0o177777, 0o100000, 0o077777,
            0o140000, 0o100000, 0o077777, 0o077777, 0o037777, 1, 0o077776,
            0o077777, 0o037776, 0o100002, 0o077775, 0o077777, 0o037776, 3,
            0o054321, 0o077777, 0o026150, 0o023457, 0o043210, 0o077777,
            0o021503, 0o134570, 0o032412, 0o077777, 0o015204, 0o145366, 7,
            0o077777, 3, 0o077771, 6, 0o077777, 2, 0o177772, 5, 0o077777, 2,
            0o077773, 4, 0o077777, 1, 0o177774, 3, 0o077777, 1, 0o077775, 2,
            0o077777, 0, 0o177776, 1, 0o077777, 0, 0o077777, 0o177777,
            0o077776, 0o177777, 0o100002, 0o177776, 0o077776, 0o177777, 4,
            0o177000, 0o077776, 0o177400, 0o002000, 0o176543, 0o077776,
            0o177261, 0o102472, 0o105432, 0o077776, 0o142615, 0o164714,
            0o100420, 0o077776, 0o140210, 0o176740, 0o100002, 0o077776,
            0o140001, 0o177774, 0o100001, 0o077776, 0o140001, 0o077776,
            0o100000, 0o077776, 0o140001, 0, 0o077777, 0o077776, 0o037776,
            0o100002, 0o077776, 0o077776, 0o037776, 4, 0o077775, 0o077776,
            0o037775, 0o100006, 0o054321, 0o077776, 0o026147, 0o147136,
            0o043210, 0o077776, 0o021503, 0o071360, 0o032412, 0o077776,
            0o015204, 0o112754, 7, 0o077776, 3, 0o077762, 6, 0o077776, 2,
            0o177764, 5, 0o077776, 2, 0o077766, 4, 0o077776, 1, 0o177770, 3,
            0o077776, 1, 0o077772, 2, 0o077776, 0, 0o177774, 1, 0o077776, 0,
            0o077776, 0o177777, 0o077775, 0o177777, 0o100003, 0o177776,
            0o077775, 0o177777, 6, 0o177000, 0o077775, 0o177400, 0o003000,
            0o176543, 0o077775, 0o177261, 0o103727, 0o105432, 0o077775,
            0o142616, 0o057262, 0o100420, 0o077775, 0o140211, 0o076320,
            0o100002, 0o077775, 0o140002, 0o077772, 0o100001, 0o077775,
            0o140001, 0o177775, 0o100000, 0o077775, 0o140001, 0o100000,
            0o077777, 0o077775, 0o037776, 3, 0o077776, 0o077775, 0o037775,
            0o100006, 0o077775, 0o077775, 0o037775, 0o000011, 0o054321,
            0o077775, 0o026147, 0o072615, 0o043210, 0o077775, 0o021503,
            0o026150, 0o032412, 0o077775, 0o015204, 0o060342, 7, 0o077775, 3,
            0o077753, 6, 0o077775, 2, 0o177756, 5, 0o077775, 2, 0o077761, 4,
            0o077775, 1, 0o177764, 3, 0o077775, 1, 0o077767, 2, 0o077775, 0,
            0o177772, 1, 0o077775, 0, 0o077775, 0o177777, 0o054321, 0o177777,
            0o123457, 0o177776, 0o054321, 0o177777, 0o047136, 0o177000,
            0o054321, 0o177516, 0o057000, 0o176543, 0o054321, 0o177427,
            0o162723, 0o105432, 0o054321, 0o153561, 0o100072, 0o100420,
            0o054321, 0o151765, 0o157020, 0o100002, 0o054321, 0o151630,
            0o030642, 0o100001, 0o054321, 0o151627, 0o154321, 0o100000,
            0o054321, 0o151627, 0o100000, 0o077777, 0o054321, 0o026150,
            0o023457, 0o077776, 0o054321, 0o026147, 0o147136, 0o077775,
            0o054321, 0o026147, 0o072615, 0o054321, 0o054321, 0o017320,
            0o055241, 0o043210, 0o054321, 0o014170, 0o052410, 0o032412,
            0o054321, 0o011146, 0o136452, 7, 0o054321, 2, 0o066667, 6,
            0o054321, 2, 0o012346, 5, 0o054321, 1, 0o136025, 4, 0o054321, 1,
            0o061504, 3, 0o054321, 1, 0o005163, 2, 0o054321, 0, 0o130642, 1,
            0o054321, 0, 0o054321, 0o177777, 0o043210, 0o177777, 0o134570,
            0o177776, 0o043210, 0o177777, 0o071360, 0o177000, 0o043210,
            0o177562, 0o170000, 0o176543, 0o043210, 0o177507, 0o127230,
            0o105432, 0o043210, 0o157713, 0o000720, 0o100420, 0o043210,
            0o156406, 0o170200, 0o100002, 0o043210, 0o156274, 0o106420,
            0o100001, 0o043210, 0o156274, 0o043210, 0o100000, 0o043210,
            0o156274, 0, 0o077777, 0o043210, 0o021503, 0o134570, 0o077776,
            0o043210, 0o021503, 0o071360, 0o077775, 0o043210, 0o021503,
            0o026150, 0o054321, 0o043210, 0o014170, 0o052410, 0o043210,
            0o043210, 0o011556, 0o124100, 0o032412, 0o043210, 0o007234,
            0o164520, 7, 0o043210, 1, 0o166670, 6, 0o043210, 1, 0o123460, 5,
            0o043210, 1, 0o060250, 4, 0o043210, 1, 0o015040, 3, 0o043210, 0,
            0o151630, 2, 0o043210, 0, 0o106420, 1, 0o043210, 0, 0o043210,
            0o177777, 0o032412, 0o177777, 0o145366, 0o177776, 0o032412,
            0o177777, 0o112754, 0o177000, 0o032412, 0o177625, 0o166000,
            0o176543, 0o032412, 0o177565, 0o062336, 0o105432, 0o032412,
            0o163707, 0o150404, 0o100420, 0o032412, 0o162663, 0o055240,
            0o100002, 0o032412, 0o162573, 0o065024, 0o100001, 0o032412,
            0o162573, 0o032412, 0o100000, 0o032412, 0o162573, 0, 0o077777,
            0o032412, 0o015204, 0o145366, 0o077776, 0o032412, 0o015204,
            0o112754, 0o077775, 0o032412, 0o015204, 0o060342, 0o054321,
            0o032412, 0o011146, 0o136452, 0o043210, 0o032412, 0o007234,
            0o164520, 0o032412, 0o032412, 0o005375, 0o022144, 7, 0o032412, 1,
            0o071506, 6, 0o032412, 1, 0o037074, 5, 0o032412, 1, 0o004462, 4,
            0o032412, 0, 0o152050, 3, 0o032412, 0, 0o117436, 2, 0o032412, 0,
            0o065024, 1, 0o032412, 0, 0o032412, 0o177777, 7, 0o177777,
            0o177771, 0o177776, 7, 0o177777, 0o177762, 0o177000, 7, 0o177777,
            0o171000, 0o176543, 7, 0o177777, 0o166665, 0o105432, 7, 0o177774,
            0o146666, 0o100420, 7, 0o177774, 0o103560, 0o100002, 7, 0o177774,
            0o100016, 0o100001, 7, 0o177774, 0o100007, 0o100000, 7, 0o177774,
            0o100000, 0o077777, 7, 3, 0o077771, 0o077776, 7, 3, 0o077762,
            0o077775, 7, 3, 0o077753, 0o054321, 7, 2, 0o066667, 0o043210, 7,
            1, 0o166670, 0o032412, 7, 1, 0o071506, 7, 7, 0, 0o000061, 6, 7, 0,
            0o000052, 5, 7, 0, 0o000043, 4, 7, 0, 0o000034, 3, 7, 0, 0o000025,
            2, 7, 0, 0o000016, 1, 7, 0, 7, 0o177777, 6, 0o177777, 0o177772,
            0o177776, 6, 0o177777, 0o177764, 0o177000, 6, 0o177777, 0o172000,
            0o176543, 6, 0o177777, 0o170122, 0o105432, 6, 0o177775, 0o041234,
            0o100420, 6, 0o177775, 0o003140, 0o100002, 6, 0o177775, 0o000014,
            0o100001, 6, 0o177775, 6, 0o100000, 6, 0o177775, 0, 0o077777, 6,
            2, 0o177772, 0o077776, 6, 2, 0o177764, 0o077775, 6, 2, 0o177756,
            0o054321, 6, 2, 0o012346, 0o043210, 6, 1, 0o123460, 0o032412, 6,
            1, 0o037074, 7, 6, 0, 0o000052, 6, 6, 0, 0o000044, 5, 6, 0,
            0o000036, 4, 6, 0, 0o000030, 3, 6, 0, 0o000022, 2, 6, 0, 0o000014,
            1, 6, 0, 6, 0o177777, 5, 0o177777, 0o177773, 0o177776, 5,
            0o177777, 0o177766, 0o177000, 5, 0o177777, 0o173000, 0o176543, 5,
            0o177777, 0o171357, 0o105432, 5, 0o177775, 0o133602, 0o100420, 5,
            0o177775, 0o102520, 0o100002, 5, 0o177775, 0o100012, 0o100001, 5,
            0o177775, 0o100005, 0o100000, 5, 0o177775, 0o100000, 0o077777, 5,
            2, 0o077773, 0o077776, 5, 2, 0o077766, 0o077775, 5, 2, 0o077761,
            0o054321, 5, 1, 0o136025, 0o043210, 5, 1, 0o060250, 0o032412, 5,
            1, 0o004462, 7, 5, 0, 0o000043, 6, 5, 0, 0o000036, 5, 5, 0,
            0o000031, 4, 5, 0, 0o000024, 3, 5, 0, 0o000017, 2, 5, 0, 0o000012,
            1, 5, 0, 5, 0o177777, 4, 0o177777, 0o177774, 0o177776, 4,
            0o177777, 0o177770, 0o177000, 4, 0o177777, 0o174000, 0o176543, 4,
            0o177777, 0o172614, 0o105432, 4, 0o177776, 0o026150, 0o100420, 4,
            0o177776, 0o002100, 0o100002, 4, 0o177776, 0o000010, 0o100001, 4,
            0o177776, 4, 0o100000, 4, 0o177776, 0, 0o077777, 4, 1, 0o177774,
            0o077776, 4, 1, 0o177770, 0o077775, 4, 1, 0o177764, 0o054321, 4,
            1, 0o061504, 0o043210, 4, 1, 0o015040, 0o032412, 4, 0, 0o152050,
            7, 4, 0, 0o000034, 6, 4, 0, 0o000030, 5, 4, 0, 0o000024, 4, 4, 0,
            0o000020, 3, 4, 0, 0o000014, 2, 4, 0, 0o000010, 1, 4, 0, 4,
            0o177777, 3, 0o177777, 0o177775, 0o177776, 3, 0o177777, 0o177772,
            0o177000, 3, 0o177777, 0o175000, 0o176543, 3, 0o177777, 0o174051,
            0o105432, 3, 0o177776, 0o120516, 0o100420, 3, 0o177776, 0o101460,
            0o100002, 3, 0o177776, 0o100006, 0o100001, 3, 0o177776, 0o100003,
            0o100000, 3, 0o177776, 0o100000, 0o077777, 3, 1, 0o077775,
            0o077776, 3, 1, 0o077772, 0o077775, 3, 1, 0o077767, 0o054321, 3,
            1, 0o005163, 0o043210, 3, 0, 0o151630, 0o032412, 3, 0, 0o117436,
            7, 3, 0, 0o000025, 6, 3, 0, 0o000022, 5, 3, 0, 0o000017, 4, 3, 0,
            0o000014, 3, 3, 0, 0o000011, 2, 3, 0, 6, 1, 3, 0, 3, 0o177777, 2,
            0o177777, 0o177776, 0o177776, 2, 0o177777, 0o177774, 0o177000, 2,
            0o177777, 0o176000, 0o176543, 2, 0o177777, 0o175306, 0o105432, 2,
            0o177777, 0o013064, 0o100420, 2, 0o177777, 0o001040, 0o100002, 2,
            0o177777, 4, 0o100001, 2, 0o177777, 2, 0o100000, 2, 0o177777, 0,
            0o077777, 2, 0, 0o177776, 0o077776, 2, 0, 0o177774, 0o077775, 2,
            0, 0o177772, 0o054321, 2, 0, 0o130642, 0o043210, 2, 0, 0o106420,
            0o032412, 2, 0, 0o065024, 7, 2, 0, 0o000016, 6, 2, 0, 0o000014, 5,
            2, 0, 0o000012, 4, 2, 0, 0o000010, 3, 2, 0, 6, 2, 2, 0, 4, 1, 2,
            0, 2, 0o177777, 1, 0o177777, 0o177777, 0o177776, 1, 0o177777,
            0o177776, 0o177000, 1, 0o177777, 0o177000, 0o176543, 1, 0o177777,
            0o176543, 0o105432, 1, 0o177777, 0o105432, 0o100420, 1, 0o177777,
            0o100420, 0o100002, 1, 0o177777, 0o100002, 0o100001, 1, 0o177777,
            0o100001, 0o100000, 1, 0o177777, 0o100000, 0o077777, 1, 0,
            0o077777, 0o077776, 1, 0, 0o077776, 0o077775, 1, 0, 0o077775,
            0o054321, 1, 0, 0o054321, 0o043210, 1, 0, 0o043210, 0o032412, 1,
            0, 0o032412, 7, 1, 0, 7, 6, 1, 0, 6, 5, 1, 0, 5, 4, 1, 0, 4, 3, 1,
            0, 3, 2, 1, 0, 2, 1, 1, 0, 1, 0
        ]

        a = InstructionBlock()
        # get data from simhdata and runs R0:R1 = X*Y; compares results
        a.mov(0o40000, 'sp')       # just make sure there's a stack, not used
        a.mov(0o20000, 'r3')       # simhdata at 020000
        a.label('loop')
        a.mov('(r3)+', 'r0')       # load X
        a.beq('done')              # zero is the end sentinel
        # a.mul('(r3)+', 'r0')       # R0:R1 = X*Y
        a.literal(0o070023)
        a.cmp('(r3)+', 'r0')       # check high order
        a.bne('fail')
        a.cmp('(r3)+', 'r1')       # check low order
        a.bne('fail')
        a.br('loop')
        a.label('done')
        a.clr('r3')                # r3 being zero indicates success
        a.halt()
        a.label('fail')
        a.halt()                   # r3 points after miscompare

        p = self.make_pdp()
        instloc = 0o10000
        self.loadphysmem(p, a, instloc)
        self.loadphysmem(p, simhdata, 0o20000)
        p.run(pc=instloc)
        self.check16(p)
        self.assertEqual(p.r[3], 0)

    def test_div(self):
        # test the div instruction
        # The 32-bit int in R and R|1 is divided by the src operand

        p = self.make_pdp()

        a = InstructionBlock()
        # The test cases will be X / Y:
        #    X : 1, 255, 4096, 10017, 32767, 32768, 32769
        #        and then those same values with 690000 added to them
        #    Y : -50 .. 50 but skipping 0 and using a large number
        #
        # The code is written this way so that the resulting block
        # is completely self-contained (does not rely on python to
        # drive it). This made it easier to cross-verify w/SIMH

        # As described above: X test values.
        #   *** SHA256 NOTE: DO NOT CHANGE THESE TEST VALUES.
        #       THE TEST RELIES ON A PRECOMPUTED SHA256 HASH BASED
        #       ON RESULTS FROM THESE VALUES
        xvals = (1, 255, 4096, 10017, 32767, 32768, 32769)

        xtable = 0o20000       # address for storing the above
        results = 0o30000      # address for storing results list

        # instead of div by zero, div by this randomish large number
        #   *** DO NOT CHANGE; see sha256 note above
        largedivisor = 10017    # has to be 16 bits or less

        # The divisor will run from -this to this.
        #   *** DO NOT CHANGE; see sha256 note above
        divisorrange = 50

        a.clr(a.ptr(0o177776))
        a.mov(xtable, 'r0')
        for x in xvals:
            # same data but negated
            xneg = ((p.MASK32 + 1) - x) & p.MASK32

            # same data but 690000 arbitrarily added (to make it 32bit)
            xplus = x + 690000

            # ...and negated
            xplusneg = ((p.MASK32 + 1) - xplus) & p.MASK32

            # put all of those into the dividend table
            for v in (x, xneg, xplus, xplusneg):
                a.mov((v >> 16) & p.MASK16, '(r0)+')
                a.mov(v & p.MASK16, '(r0)+')

        a.clr('(r0)+')               # sentinel
        a.clr('(r0)')                # sentinel

        # test loop. Divisor in r4. Dividend in r2/r3
        # xval pointer in r0. Results pointer in r1
        a.mov(results, 'r1')
        a.mov(-divisorrange, 'r4')
        a.label('outer')
        a.mov(xtable, 'r0')
        a.label('inner')
        a.mov('(r0)+', 'r2')
        a.mov('(r0)+', 'r3')
        a.tst('r2')
        a.bne('divide')
        a.tst('r3')
        a.bne('divide')

        # hit the sentinel, bump the divisor
        # look for the large divisor and forge a zero to get to 1

        a.cmp('r4', largedivisor)
        a.bne('bump')
        a.clr('r4')

        a.label('bump')
        a.inc('r4')
        a.bne('nz')

        a.mov(largedivisor, 'r4')
        a.br('outer')

        a.label('nz')
        a.cmp('r4', divisorrange)
        a.ble('outer')

        a.mov(69, 'r0')        # this indicates success
        a.halt()
        a.label('divide')

        # divide instruction hand-assembled
        a.literal(0o071204)

        # first save the PSW
        a.mov(a.ptr(0o177776), '(r1)+')
        a.mov('r2', '(r1)+')
        a.mov('r3', '(r1)+')
        a.br('inner')

        self.loadphysmem(p, a, 0o10000)
        p.run(pc=0o10000)
        self.check16(p)

        # --- this is how the above was exported for use in SIMH ---
        # with open('div.ini', 'w') as f:
        #     for s in a.simh(startaddr=0o10000):
        #        f.write(s + '\n')
        #
        # Then that .ini file was loaded into SIMH, the program was run,
        # and the output data was downloaded from SIMH. The results are
        # huge, so that was sha256 hashed into this one "good" hash value.

        good = ('f5e525b90728cb6fc4eecd97ad4b36c'
                '995d2e5b8890f7c0531284615ee9958d4')

        # so see if the live results hash to the same value
        def bytify():
            for a in range(0o30000, p.r[1], 2):
                m = p.mmu.wordRW(a)
                yield m & 0xFF
                yield (m >> 8) & 0xFF

        h = hashlib.sha256()
        h.update(bytes(bytify()))

        self.assertEqual(good, h.hexdigest())

    def test_pcwrap(self):
        # tests that the PC correctly wraps 0177776 --> 0
        p, a = self.u64mapped_pdp()
        p.run(pc=a)

        # put machine into USER mode, clear all registers (on principle)
        # NOTE: DO NOT BASH p.psw_curmode directly, that bypasses all
        #       the KSP vs USP magic
        p.psw = (p.USER << 14)

        for i in range(8):
            p.r[i] = 0

        # jam a MOV R2,R3 instruction at the end of user space
        # and a MOV R3,R4 instruction at location 0
        # The test verifies these correctly wrapped around
        p.mmu.wordRW(0o177776, 0o010203)
        p.mmu.wordRW(0, 0o010304)

        # NOTE ... when PC reaches location 2, it executes a HALT (0).
        # This causes a ReservedInstruction trap in user mode. Since
        # none of the vectors have been filled in, control will transfer
        # (in KERNEL mode, because PSW in vector is zero) to location 0,
        # where it will find another HALT instruction (i.e., 0) and stop.
        # So all this works without making more scaffolding for that...

        # this magic number MUST be negative because the N bit is
        # also checked in the saved PSW check below.
        magic = 0o123321
        p.r[2] = magic
        p.run(pc=0o177776)
        self.check16(p)

        # both MOV instructions should have executed
        self.assertEqual(p.r[2], magic)
        self.assertEqual(p.r[3], magic)
        self.assertEqual(p.r[4], magic)

        # the ReservedInstruction trap should have put the (user) PC
        # value which is 2 past the halt instruction (i.e., 4) on stack
        self.assertEqual(p.mmu.wordRW(0o160000 - 4), 4)

        # and the saved PSW which should be plain user mode, N-bit
        self.assertEqual(p.mmu.wordRW(0o160000 - 2), (p.USER << 14) | 0o10)

    def test_trap(self):
        # test some traps

        p = self.make_pdp()

        # put a handlers for different traps into memory
        # starting at location 0o10000 (4K). This just knows
        # that each handler is 3 words long, the code being:
        #     MOV something,R4
        #     RTT
        #
        # where the "something" changes with each handler.
        handlers_addr = 0o10000
        handlers = (
            0o012704, 0o4444, 0o000006,      # for vector 0o004
            0o012704, 0o1010, 0o000006,      # for vector 0o010
            0o012704, 0o3030, 0o000006,      # for vector 0o030
            0o012704, 0o3434, 0o000006       # for vector 0o034
        )
        self.loadphysmem(p, handlers, handlers_addr)

        # and just jam the vectors in place
        p.physmem[2] = handlers_addr        # vector 0o004
        p.physmem[3] = 0                    # new PSW, stay in kernel mode
        p.physmem[4] = handlers_addr + 6    # each handler above was 6 bytes
        p.physmem[5] = 0
        p.physmem[12] = handlers_addr + 12  # vector 0o30 (EMT)
        p.physmem[13] = 0
        p.physmem[14] = handlers_addr + 18  # vector 0o34 (TRAP)
        p.physmem[15] = 0

        # (tnum, insts)
        testvectors = (
            # this will reference an odd address, trap 4
            (0o4444, (
                # establish reasonable stack pointer (at 8K)
                0o012706, 0o20000,
                # CLR R3 and R4 so will know if they get set to something
                0o005003, 0o005004,
                # put 0o1001 into R0
                0o012700, 0o1001,
                # and reference it ... boom!
                0o011001,
                # show that the RTT got to here by putting magic into R3
                0o012703, 0o123456)),

            # this will execute a reserved instruction trap 10
            (0o1010, (
                # establish reasonable stack pointer (at 8K)
                0o012706, 0o20000,
                # CLR R3 and R4 so will know if they get set to something
                0o005003, 0o005004,
                # 0o007777 is a reserved instruction ... boom!
                0o007777,
                # show that the RTT got to here by putting magic into R3
                0o012703, 0o123456)),

            # this will execute an EMT instruction
            (0o3030, (
                # establish reasonable stack pointer (at 8K)
                0o012706, 0o20000,
                # CLR R3 and R4 so will know if they get set to something
                0o005003, 0o005004,
                # EMT #42
                0o104042,
                # show that the RTT got to here by putting magic into R3
                0o012703, 0o123456)),

            # this will execute an actual TRAP instruction
            (0o3434, (
                # establish reasonable stack pointer (at 8K)
                0o012706, 0o20000,
                # CLR R3 and R4 so will know if they get set to something
                0o005003, 0o005004,
                # TRAP #17
                0o104417,
                # show that the RTT got to here by putting magic into R3
                0o012703, 0o123456)),
            )

        for R4, insts in testvectors:
            self.loadphysmem(p, insts, 0o3000)
            p.run(pc=0o3000)
            self.check16(p)
            self.assertEqual(p.r[3], 0o123456)
            self.assertEqual(p.r[4], R4)

    def test_trapcodes(self):
        # a more ambitious testing of TRAP which verifies all
        # available TRAP instruction codes work

        p = self.make_pdp()
        # poke the TRAP vector info directly in
        p.physmem[14] = 0o10000           # vector 0o34 (TRAP) --> 0o10000
        p.physmem[15] = 0

        # this trap handler puts the trap # into R3
        handler = InstructionBlock()
        # the saved PC is at the top of the stack ... get it
        handler.mov('(sp)', 'r0')
        # get the low byte of the instruction which is the trap code
        # note that the PC points after the TRAP instruction so:
        handler.movb('-2(r0)', 'r3')
        handler.rtt()

        self.loadphysmem(p, handler, 0o10000)

        # just bash a stack pointer directly in
        p.r[6] = 0o20000       # 8K and working down

        for i in range(256):
            a = InstructionBlock()
            a.trap(i)          # TRAP #i
            a.mov('r3', 'r1')  # MOV R3,R1 just to show RTT worked
            a.halt()

            self.loadphysmem(p, a, 0o30000)
            p.run(pc=0o30000)
            self.check16(p)
            self.assertEqual(p.r[3], p.r[1])

            # because the machine code did MOVB, values over 127 get
            # sign extended, so take that into consideration
            if i > 127:
                trapexpected = 0xFF00 | i
            else:
                trapexpected = i
            self.assertEqual(p.r[1], trapexpected)

    def test_trapconditions(self):
        # test various types of instruction sequences that should
        # produce specific types of traps.

        # The original code, which was also used in SIMH to
        # generate/validate the expected test behavior of various
        # illegal things...
        #
        # start:  mov $stack,sp
        #
        # / make the vector table
        #         clr r1
        #         mov $decrs,r2
        #         mov $128.,r3
        # vectloop:
        #         mov r2,(r1)+
        #         mov $PS7,(r1)+
        #         tst (r2)+               / just incrementing r2 by 2
        #         sob r3, vectloop
        #
        # / r4 is the magic for figuring out the trap
        #         mov $128.,r4
        #         jmp *$010000            / run the offending instructions
        #
        # The "decrs" is 128 "dec r4" instructions and each trap vector
        # jumps to a corresponding place in the sequence so that at the end
        # r4 becomes the trap number

        # now make all that as shown above
        startaddr = 0o1000
        tsaddr = 0o10000
        a = InstructionBlock()

        # this is position-independent variation of original, because
        # that's what pdpasmhelper (only) supports for forward refs
        a.mov('pc', 'r0')
        a.add(a.getlabel('stack', idxrel=True), 'r0')
        a.mov('r0', 'sp')

        a.clr('r1')

        # same shenanigans for decrs / pc-rel
        a.mov('pc', 'r2')
        a.add(a.getlabel('decrs', idxrel=True), 'r2')

        a.mov(128, 'r3')
        a.label('vectloop')
        a.mov('r2', '(r1)+')
        a.mov(0o340, '(r1)+')
        a.tst('(r2)+')
        a.sob('r3', 'vectloop')
        a.mov(128, 'r4')
        a.jmp(a.ptr(tsaddr))

        # these are unnecessary but serve as a miscalculation barrier too
        a.halt()
        a.halt()
        a.halt()

        a.label('decrs')
        for i in range(128):
            a.dec('r4')

        # it's just nicer to have a vector address, not a "vector number"
        a.asl('r4')
        a.asl('r4')
        a.halt()

        # room for a trivial stack
        for _ in range(16):
            a.literal(0)
        a.label('stack')

        # test vectors are: (inst-sequence, expected-trap)
        testvectors = []

        # JMP to a register is a ReservedInstruction 0o10 trap
        t = InstructionBlock()
        t.jmp('r0')
        testvectors.append((list(t), 0o10))

        # accessing an odd address is an AddressError 0o04
        t = InstructionBlock()
        t.clr('r0')
        t.inc('r0')
        t.tst('(r0)')
        testvectors.append((list(t), 0o04))

        # another test of the exact same thing, only different
        t = InstructionBlock()
        t.clr('r0')
        t.tst('1(r0)')
        testvectors.append((list(t), 0o04))

        for insts, tx in testvectors:
            with self.subTest(insts=insts, tx=tx):
                p = self.make_pdp()
                self.loadphysmem(p, list(a), startaddr)
                self.loadphysmem(p, insts, tsaddr)
                p.run(pc=startaddr)
                self.check16(p)
                self.assertEqual(tx, p.r[4])
                self.assertEqual(p.r[6], a.getlabel('stack')+startaddr-4)

    def _make_updown(self, taddr, uaddr, kaddr, uphysdata=0o200000):
        # Makes the instruction blocks required for the mmu_updown tests.
        # This is separated out so (as described below) it becomes possible
        # to execute this in isolation just to generate the instructions
        # and use them in other simulators (e.g., SIMH)
        #
        # Returns three tuples:
        #  (taddr, t), (uaddr, u), (k addr, k)
        # where the t/u/k elements are the instruction blocks.

        # the kernel stack will start at 8K and work its way down.
        # This is fixed/required in the code. The taddr and kaddr must also
        # be in that first 8K and MUST leave space for the kernel stack.
        # The kernel stack is arbitrarily given at least 256 bytes
        # (in reality it doesn't come close to using that)
        APR0_end = 0o20000
        if APR0_end - max(taddr, kaddr) < 256:
            raise ValueError("not enough room for kernel stack")

        kernel_stack = APR0_end

        # there are some variable locations needed in the code, they
        # are allocated from the "stack", like this:

        kernel_stack -= 2
        saved_r5 = kernel_stack

        cn = self.usefulconstants()

        # Two tests - up and down.
        #
        # In both tests, KERNEL I space page 0 is mapped to physical 0
        # and KERNEL I space page 7 is mapped to the I/O page.
        # I/D separation is NOT enabled for KERNEL.
        # taddr and kaddr MUST be in this first 8K of memory (if only
        # because the mapping setup doesn't map anything else)
        #
        # USER I space is mapped to uaddr which can be any 64-byte
        # (i.e., mappable to user 0) boundary 0o20000 and beyond.
        #
        # All 64K of USER D space is mapped to 64K of physical memory
        # from uphysdata to uphysdata + 64K, but with a bizarre page
        # length scheme according to UP or DOWN phase of the test as
        # below. I/D separation is (obviously) enabled for USER.
        # All 64K of that memory is filled with sequential words such
        # that (vaddr) + vaddr = 0o123456 (where vaddr is a user D space
        # virtual address 0 .. 65534). This gives the test two ways to verify
        # the MMU map is working correctly: by where the accessibility of a
        # segment ends and by the value at the location where it ends.
        #
        # Segment pages in the PDP-11 are broken down into 32-word (64-byte)
        # units called "blocks" in the manual. There are 128 blocks in
        # each 8KB page.
        #
        # For UP direction test:
        #
        #   using ED=0 (segments grow upwards), create a user DSPACE mapping
        #   where page zero has length ("PLF") 0, page 1 has
        #   length 16, page 2 has length 32 (all measured in blocks) etc...
        #   and then check that valid addresses map correctly and
        #   invalid ones fault correctly. Note a subtle semantic of the PDP
        #   page length field: to be invalid (in an upward growing segment)
        #   the address has to be GREATER than the computed block number.
        #   Thus a length of "zero" still has 1 valid block of words.
        #
        # For DOWN:
        #   using ED=1 ("dirbit" = 0o10) segments grow downwards, with the
        #   same 0, 16, 32 .. progression (of valid "blocks") but they
        #   are at the end of the segments.
        #
        # This test can be rightly criticized as unnecessarily complex;
        # once the math has been shown correct for a few important length
        # cases to consider, the possibility of further bugs is remote.
        # Nevertheless, here it is.

        # this code will go at taddr
        tr = InstructionBlock()
        # The trap handler for MMU faults and the trap 0 / trap 1 from
        # the user code (when there is no MMU fault). It is integrated
        # into one routine.
        #
        # The user code gets to use r0 and r1, the trap handler
        # gets to use r2-r5:
        #      r2: expected good flag (initialized elsewhere)
        #      r3: determined good flag (by trap entry)
        #      r5: test table data pointer (initialized elsewhere)
        tr.label('UTrap')

        # first determine if trap0 (bad) or trap1 (good)
        tr.mov('(sp)', 'r3')       # get user PC from trap frame
        tr.mfpi('-2(r3)')          # get the trap instruction
        tr.mov('(sp)+', 'r3')      # r3 is now the trap instruction
        tr.bic(0o177400, 'r3')
        tr.cmp(1, 'r3')
        tr.beq('common')           # skip the HALT and the MMU entry point
        # this was not a "good" trap, the user code failed
        tr.halt()

        tr.label('TrapMMU')
        tr.clr('r3')            # indicate MMU fault case

        # both Utrap and TrapMMU join in common here on out
        # see if the access was good/bad as expected
        tr.label('common')
        tr.cmp('r2', 'r3')
        tr.beq('bump')          # jump over the HALT
        tr.halt()               # NOPE, something wrong!

        # the user mode code specifically avoids '(r0)+'
        # to avoid ambiguity in machine types for when the
        # autoincrement happens in the face of MMU aborts.
        # Bump r0 for the user code here accordingly:
        tr.label('bump')
        tr.add(2, 'r0')

        # see if it is time to switch to next table entry
        tr.cmp('2(r5)', 'r0')
        tr.bne('rtu')           # skip over the "time to switch" stanza

        # it is time to switch
        tr.add(4, 'r5')
        tr.mov('(r5)', 'r2')
        tr.cmp(0o666, 'r2')
        tr.bne('rtu')
        tr.halt()            # test done; success if r2 = 0o666

        tr.label('rtu')
        # next iteration of the user code loop
        tr.clr('(sp)')       # put user PC back to zero
        tr.rtt()

        # this trap handler is only used during the startup phase
        # See where the kernel code invokes the user setup code
        tr.label('trap_usersetup')
        # the kernel put a resume address onto the stack, just go there
        tr.add(4, 'sp')       # get rid of user trap frame, don't care
        tr.mov('(sp)+', 'pc')

        # user mode program: there are two parts to this
        # Starting at (user) location ZERO: the test program:
        #    read the given address: mov (r0),r1
        #    If this causes an MMU fault, it goes to the MMU trap handler
        #    If it succeeds, it then hits the trap(1) and goes to that
        #    handler. The kernel trap handler, using the test table,
        #    can then validate that the right thing happened.
        #    The code "loops" only because the kernel trap handler resets
        #    the PC to zero and bumps r0 then returns to user mode for the
        #    next iteration. (Yes, all this could have been done with mfpd
        #    but that feels like a different test than this)
        #
        # Start at (user) location labelled 'setup'
        #   user-mode code executed before the test begins to put
        #   the test pattern into memory

        u = InstructionBlock()
        # this subtract combines the access check with part1 of chksum
        u.mov(0o123456, 'r1')
        u.sub('(r0)', 'r1')
        u.cmp('r0', 'r1')
        u.beq('good')
        u.trap(0o77)            # trap 77 indicates miscompare

        u.label('good')
        u.trap(1)               # indicate good status
        # the kernel puts the PC back to zero after the good trap
        # and also bumps r0. This is how the loop loops.
        u.halt()                # never get here, this is illegal

        # this code is executed one time at startup (see kernel code)
        u.label('setup')

        # Initialize the user D space pattern that will be checked
        # by the user code. In python this was:
        #
        #   checksum = 0o123456         # arbitrary
        #   user_phys_DSPACEbase = 0o200000
        #   words = (checksum - (user_phys_DSPACEbase + o) & 0o177777
        #            for o in range(0, 65536, 2))
        #   self.loadphysmem(p, words, user_phys_DSPACEbase)
        u.clr('r0')
        u.mov(0o123456, 'r1')

        u.label('pattern')
        u.mov('r1', '(r0)+')
        u.sub(2, 'r1')
        u.tst('r0')
        u.bne('pattern')
        # the kernel code looks for this in r1 as success flag
        u.mov(0o3333, 'r1')
        u.trap(0)

        # The kernel-mode code that drives the whole test

        # this is certainly one way to allocate data variables,
        # given the limitations of the non-assembler ASM() methodology...
        #
        # The stack will start at kstackstart

        k = InstructionBlock()
        k.mov(kernel_stack, 'sp')

        # KERNEL I SPACE
        #    PAR 0 to physical 0
        #    PAR 7 to physical 760000 and 22bit not turned on
        #
        #    PDR 77406 = read/write, full length
        k.clr(k.ptr(cn.KISA0))
        k.mov(0o760000 >> 6, k.ptr(cn.KISA7))
        k.mov(0o077406, k.ptr(cn.KISD0))
        k.mov(0o077406, k.ptr(cn.KISD7))

        # USER I SPACE
        k.mov(uaddr >> 6, k.ptr(cn.UISA0))
        k.mov(0o077406, k.ptr(cn.UISD0))

        # USER D SPACE... first set it up to be simply/fully
        # accessible at its physical home, so that the pattern
        # can be set. Then come back and limit the page lengths later.

        k.mov(cn.UDSD0, 'r3')   # will walk through D0 .. D7
        # NOTE: A0 .. A7 is 040(r3)
        k.mov(uphysdata >> 6, 'r4')     # phys addr base
        k.mov(8, 'r0')
        k.label('utmp')
        k.mov('r4', '040(r3)')    # set U PAR; don't bump r3 yet
        k.add(0o200, 'r4')        # 0o200 = 8192>>6
        k.mov(0o77406, '(r3)+')   # set U PDR and bump to next
        k.sob(0, 'utmp')

        k.bis(1, k.ptr(cn.MMR3))   # enable I/D sep just for USER
        k.mov(1, k.ptr(cn.MMR0))   # turn on MMU

        # certainly could have used mtpd to set up the user mode
        # pattern but this is just another excuse to test more things
        # jump to the user 'setup' code, but first establish a handler
        # for the trap it will execute when done.
        k.mov(taddr + tr.getlabel('trap_usersetup'), '*$34')
        k.mov(0o340, '*$36')

        k.mov('pc', '-(sp)')
        # add the offset to (forward ref) back_from_u.
        k.add(k.getlabel('back_from_u', idxrel=True), '(sp)')
        k.mov(0o140340, '-(sp)')   # push user-ish PSW to K stack
        k.mov(u.getlabel('setup'), '-(sp)')   # PC for setup code
        k.rtt()

        k.label('back_from_u')
        # user code dropped this magic value into r1 on success
        k.cmp(0o3333, 'r1')
        k.beq('ok')
        k.halt()

        k.label('ok')
        # and now set the length limits on the user D space
        k.mov(cn.UDSD0, 'r3')   # will walk through D0 .. D7
        k.clr('r0')             # r0: segno*2 = (0, 2, 4, .., 14)

        k.label('PDRloop')
        k.mov('r0', 'r2')         # r2 = segno*2
        k.ash(3, 'r2')            # r2 = segno*16
        k.swab('r2')              # really (segno*16)<<8
        k.add(0o06, 'r2')         # ACF r/w segment
        k.mov('r2', '(r3)+')      # set U PDR
        k.inc('r0')               # bump r0 by two
        k.inc('r0')
        k.cmp('r0', 16)           # and loop until done all 8 segments
        k.blt('PDRloop')

        # create the test table, just push it onto the stack (yeehah!)
        k.mov(0, '-(sp)')           # this is a PAD (not really needed)
        k.mov(0o666, '-(sp)')       # this is a sentinel
        k.mov(0, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o176100, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o160000, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o154100, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o140000, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o132100, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o120000, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o110100, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o100000, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o66100, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o60000, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o44100, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o40000, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o22100, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o20000, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o100, '-(sp)')
        k.mov(1, '-(sp)')

        # the test table for the trap handler is now here:
        k.mov('sp', 'r5')
        k.mov('sp', k.ptr(saved_r5))   # so it can be recovered later

        # test starts in the region at the start of the table
        k.mov('(r5)', 'r2')

        # poke the MMU trap handler vector (250)
        k.mov(taddr + tr.getlabel('TrapMMU'), '*$250')
        k.mov(0o340, '*$252')

        # same for the "trap N" handler
        k.mov(taddr + tr.getlabel('UTrap'), '*$34')
        k.mov(0o340, '*$36')

        # ok, now ready to start the user program
        k.mov(0o140340, '-(sp)')   # push user-ish PSW to K stack
        k.clr('-(sp)')             # new user PC = 0
        k.clr('r0')                # user test expects r0 to start zero
        k.rtt()

        # this is where the DOWN test starts.
        k.label('DOWNTEST')

        # re-establish initial kernel stack
        k.mov(kernel_stack, 'sp')

        # Redo the entire test table for the down address cases
        # these were precomputed from the algorithm for setting PDRs
        k.mov(0, '-(sp)')
        k.mov(0o666, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o161700, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o160000, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o143700, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o140000, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o125700, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o120000, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o107700, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o100000, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o71700, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o60000, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o53700, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o40000, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o35700, '-(sp)')
        k.mov(0, '-(sp)')
        k.mov(0o20000, '-(sp)')
        k.mov(1, '-(sp)')
        k.mov(0o17700, '-(sp)')
        k.mov(1, '-(sp)')

        k.mov('sp', 'r5')    # r5 is where the table starts
        k.mov('sp', k.ptr(saved_r5))  # store location for re-use

        # fiddle the PDRs (PARs stay the same) for the down configuration
        k.mov(cn.UDSD0, 'r3')
        k.clr('r0')
        k.label('PARloopDOWN')
        # compute segno * 8 in r2 (r0 starts as segno*2)
        k.mov('r0', 'r2')
        k.ash(3, 'r2')
        # pln = 0o177 - (segno * 16)
        k.mov(0o177, 'r1')
        k.sub('r2', 'r1')
        k.mov('r1', 'r2')
        k.swab('r2')
        k.add(0o16, 'r2')    # the downward growing case
        k.mov('r2', '(r3)+')      # set U PDR
        k.inc('r0')
        k.inc('r0')
        k.cmp('r0', 16)
        k.blt('PARloopDOWN')

        k.clr('r2')                # the down test starts in 'bad' zone

        # ok, now ready to start the user program
        k.mov(0o140340, '-(sp)')   # push user-ish PSW to K stack
        k.clr('-(sp)')             # new user PC = 0
        k.clr('r0')                # user test expects r0 to start zero
        k.rtt()

        # Now for something extra frosty... relocate just segment 4
        # (arbitrarily chosen) of the user memory to a different
        # physical page and run the test again to ensure it still works.
        # This will make use of KERNEL A1 and A2 segments to map the
        # relocation (note: I space because no sep I/D for kernel here)
        k.label('BONUS')

        # recover the r5 stack table beginning
        k.mov(k.ptr(saved_r5), 'r5')

        # copy UDSA4 into KISA1 - mapping old segment into kernel space
        k.mov(k.ptr(cn.UDSA0 + 4*2), k.ptr(cn.KISA0 + 2))  # i.e., A1

        # the new location for this data will be physical 0o600000
        # (not a typo) which becomes 0o6000 in the PAR
        k.mov(0o6000, k.ptr(cn.KISA0 + 4))                 # i.e., A2

        # the standard PDR access/full-length/etc bits
        k.mov(0o077406, k.ptr(cn.KISD0 + 2))
        k.mov(0o077406, k.ptr(cn.KISD0 + 4))

        # count r0, source address r1, destination r2
        k.mov(4096, 'r0')
        k.mov(8192, 'r1')
        k.mov(8192*2, 'r2')
        k.mov('(r1)+', '(r2)+')
        k.literal(0o077002)            # SOB to the copy

        # switch the user page to the new mapping
        k.mov(0o6000, k.ptr(cn.UDSA0 + 4*2))

        # and the standard initialization dance
        k.clr('r2')                # the down test starts in 'bad' zone
        k.clr('r0')
        k.clr('(sp)')      # just knows the user loop starts at zero
        k.rtt()

        return (taddr, tr), (uaddr, u), (kaddr, k)

    def test_mmu_updown(self):
        # test the page length field support in both up and down directions

        # This somewhat-irresponsible magnum opus of assembly code would
        # have been much easier to write, understand, and maintain if it
        # used the PDP1170/mmu/etc methods directly and was mostly
        # written at the python level rather than elaborate machine code.
        # For example, it could have looped over calls to mmu.v2p()
        # instead of performing an elaborate dance of user mode code
        # and kernel trap handlers to analyze the same thing.
        #
        # HOWEVER, doing it as machine code allowed the same instructions
        # to be run through SIMH for cross-verification of the test itself
        # and the machine behavior. Was the juice worth the squeeze?
        # Someone else will have to decide; the deed is done.
        #
        # Note that the assembly of three code segments
        # (the trap handlers, the user code, and the "kernel") are
        # in a separate method, which is helpful for getting at the
        # instructions in the "import to SIMH" usage scenario.
        #

        cn = self.usefulconstants()
        p = self.make_pdp(memwords=256*1024)

        # On these addresses: the code isn't fully general (mostly in
        # how the MMU is set up). The kernel stack will start at 8K
        # (0o20000 physical) and work downwards. The traps and kernel
        # code can be "anywhere" so long as it is in that first 8K
        # of memory and leaves room for trap vectors at the bottom and
        # stack at the top.
        #
        # The user code can be on any 64-byte (!) physical boundary.
        # The kernel knows the virtual address for the start will be zero.
        #

        # this are all PHYSICAL addresses. The code is not fully general,
        # there are constraints: taddr and kaddr must be in the first
        # 8K of physical memory (if only because that's how the trivial
        # kernel mapping is set up). The kernel stack, which also
        # requires room for the test tables, must start at the tail end
        # of the first 8K.
        #
        # The uaddr must be on an 8K boundary
        taddr = 0o4000
        kaddr = 0o6000        # make sure enough room for the traps code
        uaddr = 0o20000

        for addr, b in self._make_updown(taddr, uaddr, kaddr):
            if addr == kaddr:
                # need to know DOWNTEST and BONUS
                downtest = kaddr + b.getlabel('DOWNTEST')
                bonus = kaddr + b.getlabel('BONUS')
            self.loadphysmem(p, b, addr)

        with self.subTest(phase="UP"):
            # finally ready to run the whole shebang!
            p.run(pc=kaddr)
            self.check16(p)

            # a halt was encountered, verify r2 is the end sentinel
            self.assertEqual(p.r[2], 0o666)

        with self.subTest(phase="DOWN"):
            # run the down test
            p.r[2] = 0            # superfluous but makes sure
            p.run(pc=downtest)
            self.check16(p)
            self.assertEqual(p.r[2], 0o666)

        with self.subTest(phase="BONUS"):
            # and the bonus test
            p.r[2] = 0            # superfluous but makes sure
            p.run(pc=bonus)
            self.check16(p)
            self.assertEqual(p.r[2], 0o666)

    def test_mmu_AWbits(self):
        cn = self.usefulconstants()
        p = self.make_pdp()

        base_address = 0o10000

        k = InstructionBlock()
        # this is silly but easy, just put the trap handler here and
        # jump over it
        k.br('L1')
        k.label('traphandler')
        k.rtt()
        k.label('L1')

        k.mov(base_address, 'sp')      # system stack just below this code

        #
        # No I/D separation turned on, so everything is mapped via I SPACE
        #  PAR 0 to physical 0
        #  PAR 1 to physical 8K
        #  PAR 2 to physical 16K
        #    ... etc ... 1:1 virtual:physical mapping up to ...
        #  PAR 7 to phys 760000 and 22bit not turned on (i.e., I/O page)
        #
        k.clr('r2')            # r2 will step by 0o200 for 8K PAR incrs
        k.mov(cn.KISA0, 'r0')  # r0 will chug through the PARs
        k.mov(7, 'r1')         # count of PARs to set
        k.label('parloop')
        k.mov('r2', '(r0)+')
        k.add(0o200, 'r2')
        k.sob('r1', 'parloop')

        # set the PAR7 to I/O page
        k.mov(0o7600, '(r0)')

        # now the PDRs
        k.mov(cn.KISD0, 'r4')
        k.mov(0o77406, '(r4)+')   # read/write, full length for PDR0
        k.mov(0o77404, 'r2')      # r/w, full length, trap on any r/w
        k.mov(6, 'r1')            # setting PDR1 .. PDR6
        k.label('pdrloop')
        k.mov('r2', '(r4)+')
        k.sob('r1', 'pdrloop')
        k.mov(0o77406, '(r4)+')   # r/w, full length for PDR7 / IO page

        # NOTE: at this point '-(r4)' will be PDR7 ... that is used below

        # set up the trap handler
        k.mov(base_address + k.getlabel('traphandler'), '*$250')
        k.mov(0o340, '*$252')

        k.mov(1, k.ptr(cn.MMR0))   # turn on MMU

        # this test code just "knows" the code is in APR0 (8K and below)
        # and makes three accesses:
        #     a READ at 12K   (APR1)
        #     a WRITE at 20K  (APR2)
        #     a READ-then-WRITE at 28K (APR3)
        #
        # then it dumps the 8 kernel PDRS onto the stack in reverse
        # (so that at the end (sp) -> PDR0
        #
        k.mov(0o20000, 'r0')       # 8K will will be the base for:
        k.mov('010000(r0)', 'r1')  # read from 12K
        k.mov(1234, '030000(r0)')  # write to 20K
        k.inc('050000(r0)')        # read-then-write to 28K

        # push (the dumb way) PDRs onto the stack for examination
        k.mov('-(r4)', '-(sp)')
        k.mov('-(r4)', '-(sp)')
        k.mov('-(r4)', '-(sp)')
        k.mov('-(r4)', '-(sp)')
        k.mov('-(r4)', '-(sp)')
        k.mov('-(r4)', '-(sp)')
        k.mov('-(r4)', '-(sp)')
        k.mov('-(r4)', '-(sp)')

        # expected:
        #   * PDR0: W only, mgmt traps not set here
        #   * PDR1 to be only A bit
        #   * PDR2 to be A and W
        #   * PDR3 to be A and W
        #   * PDR4-6 to be neither
        #   * PDR7 (not really part of the test but will be neither)

        # These expected_PDRs were obtained by running the machine
        # code in this test under SIMH.
        expected_PDRs = [0o077506,
                         0o077604,
                         0o077704,
                         0o077704,
                         0o077404,
                         0o077404,
                         0o077404,
                         0o077406]

        k.clr('r0')
        k.mov('sp', 'r1')
        for i, xpdr in enumerate(expected_PDRs):
            k.cmp(xpdr, '(r1)+')
            k.beq(f"LXX{i}")
            k.mov(i | 0o100000, 'r0')
            k.mov('-2(r1)', 'r3')
            k.br('_done')
            k.label(f"LXX{i}")

        k.label('_done')
        k.halt()

        self.loadphysmem(p, k, base_address)
        p.run(pc=base_address)
        self.check16(p)
        self.assertEqual(p.r[0], 0)

    def test_stacklim0(self):
        # verify that simply *having* an illegal SP doesn't trap
        p = self.make_pdp()
        a = InstructionBlock()
        a.clr('r0')            # will be used to verify progress

        # none of these bad stack pointers should cause
        # a YELLOW trap as they are never used as stacks...
        a.clr('sp')            # really it's already zero...
        a.inc('r0')            # show made it to here
        a.mov(1, 'sp')         # odd SP, very bad idea
        a.inc('r0')            # show made it to here
        a.mov(0o100, 'sp')     # still too low
        a.inc('r0')            # show made it to here
        a.halt()

        aa = 0o4000
        self.loadphysmem(p, a, aa)
        p.run(pc=aa)
        self.check16(p)
        self.assertEqual(p.r[0], 3)    # confirm made it all the way through

    def _stacklimcode(self, go_red=False):
        # memory usage:
        # 0o4000.. is the test code
        # 0o6000.. is the trap handler
        # 0o7000.. is the log of various values collected in the trap handler

        # r5 is used to walk through the 0o7000+ storage, it is initialized
        # in the test code and used in the trap handler

        p = self.make_pdp()

        tr = InstructionBlock()
        # record...
        tr.mov('r2', '(r5)+')              # ...separator/entry number
        tr.mov('sp', '(r5)+')              # ...the sp
        tr.mov('(sp)', '(r5)+')            # ...the trap-saved pc
        tr.mov(tr.ptr(0o177766), 'r1')     # (will be used later)
        tr.mov('r1', '(r5)+')              # ...cpu error register
        tr.mov('r2', '(r5)+')              # ...separator/entry number

        # indicate successfully completed the above, bump entry number
        tr.inc('r2')

        # but if RED trap, stop here.
        tr.bit(p.CPUERR_BITS.REDZONE, 'r1')
        tr.beq('rtt')
        tr.halt()
        tr.label('rtt')
        tr.rtt()

        tra = 0o6000
        self.loadphysmem(p, tr, tra)

        recordmagic = 0o66000
        a = InstructionBlock()
        a.mov(0o400, 'sp')
        a.mov(0o7000, 'r5')
        a.mov(recordmagic, 'r2')

        # dirty up the (to be pushed to) stack to verify writes happened
        a.mov(0o370, 'r0')
        a.mov('r0', '(r0)+')
        a.mov('r0', '(r0)+')
        a.mov('r0', '(r0)+')
        a.mov('r0', '(r0)+')

        # install the trap handler
        a.mov(tra, a.ptr(0o4))
        a.mov(0o340, a.ptr(0o6))

        loopcount = 3 if not go_red else 30   # will never get to 30
        a.mov(loopcount, 'r0')
        a.label('push')
        a.clr('-(sp)')
        a.sob('r0', 'push')
        a.halt()

        aa = 0o4000
        self.loadphysmem(p, a, aa)
        p.r[p.PC] = aa
        return p

    def test_stacklim1(self):
        # Set the stack at the top of the yellow zone and then use it.
        # This test should cause loopcount (3) YELLOW synchronous traps.
        # Behavior and expected results verified by running identical
        # machine code in SIMH

        # r5 is used to walk through the 0o7000+ storage, it is initialized
        # in the test code and used in the trap handler

        p = self._stacklimcode()
        p.run()
        self.check16(p)

        # obtained by running machine code in SIMH
        expected_7000 = [
            # MARKER       SP       PC     CPUERR    MARKER
            0o066000, 0o000372, 0o004052, 0o000010, 0o066000,
            0o066001, 0o000370, 0o004052, 0o000010, 0o066001,
            0o066002, 0o000366, 0o004052, 0o000010, 0o066002,
            0]

        recbase = 0o7000//2       # word address in phys mem
        for i, val in enumerate(expected_7000):
            with self.subTest(i=i, val=val):
                self.assertEqual(val, p.physmem[recbase + i])

    def test_stacklim_red(self):
        p = self._stacklimcode(go_red=True)
        p.run()
        self.check16(p)

        # Behavior/results verified by running machine code on SIMH;
        # however, SIMH halts the simulation on the red stack trap and
        # requires intervention to continue into the actual trap handler.
        # Doing that (i.e., "CONTINUE") leads to these same results.
        self.assertEqual(p.r[1], 0o14)      # RED|YELLOW
        self.assertEqual(p.r[2], 0o66021)   # known magic iteration marker
        self.assertEqual(p.r[6], 0)         # stack should be at zero

        # these results obtained from SIMH (running same machine code)
        expected_7000 = [
            # MARKER       SP       PC     CPUERR    MARKER
            0o066000, 0o000372, 0o004052, 0o000010, 0o066000,
            0o066001, 0o000370, 0o004052, 0o000010, 0o066001,
            0o066002, 0o000366, 0o004052, 0o000010, 0o066002,
            0o066003, 0o000364, 0o004052, 0o000010, 0o066003,
            0o066004, 0o000362, 0o004052, 0o000010, 0o066004,
            0o066005, 0o000360, 0o004052, 0o000010, 0o066005,
            0o066006, 0o000356, 0o004052, 0o000010, 0o066006,
            0o066007, 0o000354, 0o004052, 0o000010, 0o066007,
            0o066010, 0o000352, 0o004052, 0o000010, 0o066010,
            0o066011, 0o000350, 0o004052, 0o000010, 0o066011,
            0o066012, 0o000346, 0o004052, 0o000010, 0o066012,
            0o066013, 0o000344, 0o004052, 0o000010, 0o066013,
            0o066014, 0o000342, 0o004052, 0o000010, 0o066014,
            0o066015, 0o000340, 0o004052, 0o000010, 0o066015,
            0o066016, 0o000336, 0o004052, 0o000010, 0o066016,
            0o066017, 0o000334, 0o004052, 0o000010, 0o066017,
            0o066020, 0o000000, 0o004052, 0o000014, 0o066020,
            0]

        recbase = 0o7000//2       # word address in phys mem
        for i, val in enumerate(expected_7000):
            with self.subTest(i=i, val=val):
                self.assertEqual(val, p.physmem[recbase + i])

    # test loading the LDA format (absolute tape loader)
    def test_load_lda(self):
        p = self.make_pdp()
        ldabytes = [
            0, 0, 0,           # testing zero skip
            1, 0, 9, 0, 0, 8,  # address 0x800 (2k)
            1, 2, 3,           # the data
            232,               # checksum

            0, 0, 0, 0, 0,     # more zero skip testing
            1, 0, 9, 0, 3, 8,  # address 0x803 (2k)
            4, 5, 6,           # the data
            220,               # checksum
            1, 0, 9, 0, 6, 8,  # testing lack of zero skip
            7, 8, 9,
            208,
        ]

        # test two variations of the END block, with and without checksum
        # The docs say no checksum is present so it never really gets read
        # if it is there (so the harder test is if it is not there)
        endblk_with = [
            1, 0, 6, 0, 1, 1, 247
        ]

        endblk_without = [
            1, 0, 6, 0, 1, 1
        ]

        for tbytes in (ldabytes + endblk_with, ldabytes + endblk_without):
            with io.BytesIO(bytes(tbytes)) as f:

                addr = boot.load_lda_f(p, f)
                self.assertEqual(addr, 257)    # just "known" from data above
                # this range of addresses and the related data is just "known"
                # from the data bytes above
                baseaddr = 0x800
                for offset in range(9):
                    self.assertEqual(p.mmu.byteRW(baseaddr+offset), offset+1)

    def test_physrw_n(self):
        p = self.make_pdp()
        words = [1, 2, 3, 0, 0o40000, 65534]
        addr = 0o10000
        p.physRW_N(addr, len(words), words)
        for i, w in enumerate(words):
            self.assertEqual(p.physmem[(addr >> 1) + i], w)

        with self.assertRaises(PDPTraps.AddressError):
            p.physRW_N(addr+1, len(words), words)

    def test_rtt(self):
        # ensure that RTT properly filters out various bits in various modes

        p = self.make_pdp()
        kcode = 0o4000
        ucode = 0o5000
        thandlers = 0o6000
        rttcode = 0o7000
        traprec = 0o10000

        # there is ambiguity in the manuals as to whether the usermode
        # HALT is supposed to go through vector 0o4 or 0o10; SIMH seems
        # to prefer 4; more of the books seem to say 0o10. Dunno. This
        # test allows either to be ok. Note the PSW condition codes used
        # as a way to know which was which

        p.physmem[2] = thandlers
        p.physmem[3] = 0o356
        p.physmem[4] = thandlers
        p.physmem[5] = 0o357

        a = InstructionBlock()
        # the first goal is to get into user mode executing ucode
        # need a valid kernel stack
        a.mov(kcode, 'sp')         # stack just below code
        a.mov(traprec, 'r2')       # where PSW's from all traps recorded
        a.clr('r1')                # used to track progress
        a.mov(0o170000, '-(sp)')   # cm/pm user/user
        a.mov(ucode, '-(sp)')      # the userspace code
        a.rtt()
        self.loadphysmem(p, a, kcode)

        a = InstructionBlock()
        # this is the user code; the goal here is to try to execute
        # an illegal rtt and see what happens
        a.mov(ucode, 'sp')       # need a stack
        a.inc('r1')               # show that made it to here
        a.mov(0o0340, '-(sp)')
        a.mov(rttcode, '-(sp)')
        a.rtt()
        self.loadphysmem(p, a, ucode)

        # the trap handlers
        # will get here via user mode trapping HALT
        a = InstructionBlock()
        a.mov('*$177776', '-(r2)')
        a.add(0o100, 'r1')
        a.halt()
        self.loadphysmem(p, a, thandlers)

        # the rtt code the user goes to, trying to be in kernel mode
        # but it gets here still in user mode (because RTT filters that out)
        # and therefore the halt instruction will actually cause a trap
        a = InstructionBlock()
        a.add(0o1000, 'r1')
        a.halt()
        self.loadphysmem(p, a, rttcode)

        p.run(pc=kcode)
        self.assertEqual(p.r[1], 0o1101)    # all these code points reached
        self.assertEqual(p.psw_curmode, p.KERNEL)
        self.assertEqual(p.psw_prevmode, p.USER)

        # there should have been only one trap CC recorded
        self.assertEqual(p.r[2], traprec - 2)

        # and the CC should be 357 (but it is 356 in simh... investigate)
        self.assertEqual(p.physmem[p.r[2] >> 1] & 0o377, 0o357)

    def test_registerio(self):
        # on most processors the general purpose registers are not really
        # accessible this way (only accessible via console phys interface)
        # but in the emulation they are accessible so test them...
        p = self.make_pdp()
        startaddr = 0o4000

        IOPAGEBASE = 0o160000             # last 8k of the 16 bit space
        r0_addr = + p.IOPAGE_REGSETS_OFFS
        a = InstructionBlock()
        a.mov(IOPAGEBASE, 'r0')
        a.mov(f"{p.IOPAGE_REGSETS_OFFS}.(r0)", 'r1')
        a.mov(7, 'r2')                   # arbitrary; proving the next instr
        a.mov('r0', f"{p.IOPAGE_REGSETS_OFFS+2}.(r0)")  # should write into r2
        a.halt()
        self.loadphysmem(p, a, startaddr)
        p.run(pc=startaddr)
        self.check16(p)
        self.assertEqual(p.r[0], p.r[1])
        self.assertEqual(p.r[0], p.r[2])

    def test_kl11_bytewrite(self):
        # Test for
        #    https://github.com/outofmbufs/python-pdp11-emulator/issues/14
        # byte writes to KL11 transmit buffer need to work.
        p = self.make_pdp()
        p.associate_device(KL11(p.ub), 'KL')    # console
        startaddr = 0o4000
        a = InstructionBlock()
        a.clr('r0')                     # will be incremented to show success
        a.movb(13, a.ptr(0o177566))
        a.inc('r0')                     # r0 will be 1 if the movb worked
        a.halt()
        self.loadphysmem(p, a, startaddr)
        p.run(pc=startaddr)
        self.check16(p)
        self.assertEqual(p.r[0], 1)

    def test_io(self):
        # unibus I/O callback tests

        p = self.make_pdp()
        # this callback just records arguments taking advantage of closure
        cbx = SimpleNamespace(value=0, arglog=[])

        RESET_VALUE = 1234                     # arbitrary

        def callback(ioaddr, cycle, /, **kwargs):
            if len(kwargs) == 0:
                cbx.arglog.append((ioaddr, cycle))
            elif len(kwargs) == 1:
                cbx.arglog.append((ioaddr, cycle, kwargs['value']))
            else:
                raise ValueError("invalid kwargs")
            if cycle == BusCycle.READ16:
                return cbx.value
            elif cycle == BusCycle.RESET:
                cbx.value = RESET_VALUE
            elif cycle == BusCycle.WRITE16:
                cbx.value = kwargs['value']
            elif cycle == BusCycle.WRITE8:
                v = kwargs['value'] & 0xFF
                if ioaddr & 1:
                    cbx.value = (cbx.value & 0x00FF) | (v << 8)
                else:
                    cbx.value = (cbx.value & 0xFF00) | v
            else:
                assert False, "bad cycle"

        startaddr = 0o4000
        ioaddr = 0o177720        # arbitrary; in a "reserved" block fwiw
        a = InstructionBlock()
        a.mov(startaddr, 'sp')
        a.mov(ioaddr, 'r5')
        a.literal(5)                   # RESET instruction
        a.mov('(r5)', 'r0')            # expect r0 to be the RESET_VALUE
        a.mov(1, '(r5)')               # set new value to 1
        a.mov('(r5)', 'r1')            # expect r1 to be 1
        a.mov(0o400, '(r5)')           # setting a bit in the high byte
        a.movb(2, '(r5)')              # should only set the low part
        a.mov('(r5)', 'r2')            # expect r2 to be 0o402
        a.movb(0, '1(r5)')             # should only clear the high part
        a.mov('(r5)', 'r3')            # expect r3 to be 2
        a.halt()
        self.loadphysmem(p, a, startaddr)

        p.ub.register(callback, ioaddr & 8191)
        p.run(pc=startaddr)
        self.check16(p)
        # per the various comments in the test sequence above
        self.assertEqual(p.r[0], RESET_VALUE)
        self.assertEqual(p.r[1], 1)
        self.assertEqual(p.r[2], 0o402)
        self.assertEqual(p.r[3], 2)

        # the sequence of arguments expected in the log - determined
        # by inspection when the test code was created
        offs = ioaddr & 8191
        gold = (
            (offs, BusCycle.RESET),           # from the RESET instruction
            (offs, BusCycle.READ16),          # from mov (r5),r0
            (offs, BusCycle.WRITE16, 1),      # from mov $1,(r5)
            (offs, BusCycle.READ16),          # from mov (r5),r1
            (offs, BusCycle.WRITE16, 0o400),  # from mov $0400,(r5)
            (offs, BusCycle.WRITE8, 2),       # from the movb
            (offs, BusCycle.READ16),          # from mov (r5),r2
            (offs+1, BusCycle.WRITE8, 0),     # from the movb
            (offs, BusCycle.READ16),          # from mov (r5),r3
            )
        for i, t in enumerate(cbx.arglog):
            self.assertEqual(t, gold[i])

    def test_adc(self):
        p = self.make_pdp()
        a = InstructionBlock()
        a.clr('r1')
        a.mov(0o177777, 'r0')
        # NOTE WELL: INC DOES NOT SET C (!!!)
        a.add(1, 'r0')
        a.adc('r1')
        a.halt()
        self.loadphysmem(p, a, 0o10000)
        p.run(pc=0o10000)
        self.assertEqual(p.r[0], 0)
        self.assertEqual(p.r[1], 1)

    # not automatically tested
    def clocktests_basic(self):
        # test clock overhead

        # memory layout
        # 0o100     VECTOR for KW11 interrupt handler
        # 0o102     PSW for KW11 interrupt handler
        #
        # 0o10000   stack (grows down)
        # 0o10000   start address (set up clock, vectors, etc)
        # 0o20000   interrupt handler
        #
        # REGISTER USAGE
        #   r0-r3   scratch
        #   r3      count down (# of interrupts to take until HALT)
        #   r4:r5   32-bit count up performance tracker (r5 high)

        mc = InstructionBlock()
        mc_addr = 0o10000

        ih = InstructionBlock()
        ih_addr = 0o20000

        mc.mov(mc_addr, 'sp')
        mc.mov(0o340, mc.ptr(0o177776))        # PSW .. pri 7
        mc.mov(ih_addr, mc.ptr(0o100))
        mc.mov(0o340, mc.ptr(0o102))
        mc.mov(10, 'r3')
        mc.clr('r4')
        mc.clr('r5')

        # enable interrupts and start the counter loop
        mc.mov(0o100, mc.ptr(0o177546))
        mc.clr(mc.ptr(0o177776))
        mc.label('loop')
        mc.inc('r4')           # NOTE: INC does not set C (!!)
        mc.bne('loop')
        mc.inc('r5')
        mc.br('loop')

        ih.dec('r3')
        ih.beq('done')
        ih.rti()
        ih.label('done')
        ih.halt()

        # this test is a little questionable in that what it assumes
        # is that an increasing line clock frequency will correspond to
        # an increase in overhead, and the
        #
        #            self.assertTrue(thisrun < prev)
        #
        # test depends on that (i.e., fewer increments get to happen if the
        # clock interrupt is going off more often). Obviously, random
        # performance interference issues in the test environment could
        # cause a spurious failure.
        #
        prev = 0x7fffffff
        for hz in [2, 5, 10, 25, 50, 100]:
            p = self.make_pdp()
            clk = KW11HZ(p.ub, hz=hz)
            self.loadphysmem(p, mc, mc_addr)
            self.loadphysmem(p, ih, ih_addr)
            p.run(pc=mc_addr)
            thisrun = (p.r[5] << 16) | p.r[4]
            with self.subTest(hz=hz, prev=prev, thisrun=thisrun):
                self.assertTrue(thisrun < prev)
            prev = thisrun

    # not automatically tested
    def clocktests_hz(self):
        for hz in [5, 10, 25, 50, 100]:
            p = self.make_pdp()
            clk = KW11HZ(p.ub, hz=hz)

            arbitrary_delay = 1
            time.sleep(arbitrary_delay)
            stamps = list(clk.tslog)

            # However many time stamps there are, the average delay over
            # those readings should be 1/hz seconds.  It won't be that,
            # of course, but see if it is plausible.
            calculated_hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])

            # it will (presumably!) always run slow, and this is the
            # empirically-determined reasonable slop factor
            minratio = 0.8
            with self.subTest(hz=hz, calculated_hz=calculated_hz):
                self.assertTrue(calculated_hz/hz > minratio)

    def test_breakpoints1(self):
        # test the steps=N breakpoint capability

        p = self.make_pdp()

        maxtest = 100
        a = InstructionBlock()
        for i in range(maxtest):
            a.mov(i, 'r0')
        a.clr('r0')
        a.halt()

        startaddr = 0o4000
        self.loadphysmem(p, a, startaddr)

        for i in range(maxtest):
            with self.subTest(i=i):
                p.run_steps(pc=startaddr, steps=i+1)
                self.check16(p)
                self.assertEqual(p.r[0], i)

    def test_breakpoints2(self):
        # test the PCBreakpoint ('run_until') breakpoint capability

        p = self.make_pdp()

        maxtest = 100
        a = InstructionBlock()
        for i in range(maxtest):
            a.mov(i, 'r0')
            a.label(f"L{i}")
        a.clr('r0')

        startaddr = 0o4000
        self.loadphysmem(p, a, startaddr)

        for i in range(maxtest):
            with self.subTest(i=i):
                p.run_until(pc=startaddr, stoppc=startaddr+a.getlabel(f"L{i}"))
                self.check16(p)
                self.assertEqual(p.r[0], i)

    def test_breakpoints3(self):
        # test multiple breakpoints
        p = self.make_pdp()

        maxtest = 100
        a = InstructionBlock()
        for i in range(maxtest):
            a.mov(i+1, 'r0')
        a.clr('r0')
        a.halt()

        startaddr = 0o4000
        self.loadphysmem(p, a, startaddr)

        # create one MultiBreakpoint with four different Steps bkpts,
        # in this order in the MultiBreakpoint:
        #    one at 300 steps (should not fire at all)
        #    one at 75 steps
        #    one at 1 step
        #    one at 50 steps

        # each of these should fire at its "absolute" step, because they
        # are being re-used...

        # create one MultiBreakpoint of PCBreakpoints at each label
        s300 = BKP.StepsBreakpoint(steps=300)
        s75 = BKP.StepsBreakpoint(steps=75)
        s1 = BKP.StepsBreakpoint(steps=1)
        s50 = BKP.StepsBreakpoint(steps=50)

        mbp = BKP.MultiBreakpoint(s300, s75, s1, s50)

        # this test just knows there are four of them, code the easy/dumb way:
        p.r[p.PC] = startaddr
        p.run(breakpoint=mbp)
        self.check16(p)

        # this should have fired after 1 instruction...
        self.assertEqual(p.r[0], 1)

        # the next two similar
        p.run(breakpoint=mbp)
        self.check16(p)
        self.assertEqual(p.r[0], 50)
        p.run(breakpoint=mbp)
        self.check16(p)
        self.assertEqual(p.r[0], 75)

        # the last one will complete because of the HALT
        p.run(breakpoint=mbp)
        self.check16(p)
        self.assertEqual(p.r[0], 0)

        # this is really an internal detail, but test it as a way
        # to make sure s1, etc all were continuing to be called
        self.assertTrue(s1.togo < 0)
        self.assertTrue(s50.togo < 0)
        self.assertTrue(s75.togo < 0)

    def test_bkplog(self):
        # test the instruction logger "breakpoint"

        fnamebase = f"pdptestlog-{hex(id(object()))}"
        fname = fnamebase + ".log"

        try:
            os.remove(fname)
        except FileNotFoundError:
            pass

        p = PDP1170(logger=fnamebase, loglevel='DEBUG')
        # the point of this program is just to create N log
        # entries (when executed) that can be verified
        a = InstructionBlock()
        a.mov('r0', 'r0')
        a.mov('r0', 'r1')
        a.mov('r0', 'r2')
        a.mov('r0', 'r3')
        a.mov('r0', 'r4')
        a.mov('r0', 'r5')
        a.mov('r1', 'r0')
        a.mov('r1', 'r1')
        a.mov('r1', 'r2')
        a.mov('r1', 'r3')
        a.mov('r1', 'r4')
        a.mov('r1', 'r5')
        a.halt()

        instloc = 0o4000
        self.loadphysmem(p, a, instloc)
        p.run(pc=instloc, breakpoint=BKP.Logger())
        self.check16(p)

        # This is a probably-too-fragile attempt to see if each of the above
        # instructions made it into the logfile, in order. While trying to
        # accommodate possibility of other logging lines getting in there.
        # There was, of course, a better way, but this worked...
        with open(fname, 'r') as logf:
            raninto_EOF = False
            for inst in a:
                try:
                    while f":: {oct(inst)}" not in next(logf):
                        pass
                except StopIteration:
                    raninto_EOF = True
            self.assertFalse(raninto_EOF)

        os.remove(fname)

    def test_lookbackbp(self):
        p = self.make_pdp()

        # dynamically determine (within reason!) # of default lookbacks
        maxguess = 5000      # if it's more than this... meh
        curguess = 1
        startaddr = 0o4000

        while curguess < maxguess:
            a = InstructionBlock()
            for i in range(curguess):
                a.mov(i, 'r0')
            a.halt()
            self.loadphysmem(p, a, startaddr)
            bp = BKP.Lookback()
            p.run(pc=startaddr, breakpoint=bp)
            self.check16(p)
            # if current == first, there was 1 lookback, that's 1
            # But also the halt instruction takes up on; hence +2
            n = (p.r[0] - bp.states[0][1]['R0']) + 2
            if n < curguess:
                default_lookbacks = n
                break
            curguess += 1

        maxtest = default_lookbacks + 1
        a = InstructionBlock()
        for i in range(maxtest):
            a.mov(i, 'r0')
        a.clr('r0')
        a.halt()
        self.loadphysmem(p, a, startaddr)

        for i in range(maxtest):
            bp = BKP.Lookback(BKP.StepsBreakpoint(steps=i+1))
            bp7 = BKP.Lookback(BKP.StepsBreakpoint(steps=i+1), lookbacks=7)
            with self.subTest(i=i):
                p.run(pc=startaddr, breakpoint=bp)
                self.check16(p)
                p.run(pc=startaddr, breakpoint=bp7)
                self.check16(p)
                self.assertEqual(p.r[0], i)
                if i+1 <= default_lookbacks:
                    self.assertEqual(len(bp.states), i+1)
                else:
                    self.assertEqual(len(bp.states), default_lookbacks)
                self.assertEqual(len(bp7.states), min(i+1, 7))

    def test_jmp10(self):
        """Test of JMP (R0) instruction (mode 0o10)"""
        p = self.make_pdp()

        instloc = 0o10000
        a = InstructionBlock()
        a.clr('r2')
        a.jmp('(r0)')         # test driver code will set R0 to ...
        a.inc('r2')           # various
        a.inc('r2')           # ... different
        a.inc('r2')           # ...... locations
        a.inc('r2')           # ........  among these
        a.halt()

        self.loadphysmem(p, a, instloc)

        for offs, r2 in ((4, 4), (6, 3), (8, 2), (10, 1), (12, 0)):
            p.r[0] = instloc + offs
            p.run(pc=instloc)
            self.check16(p)
            with self.subTest(offs=offs):
                self.assertEqual(p.r[2], r2)

    def test_jmp67(self):
        """Test of JMP offs(PC) instruction (mode 0o67)"""
        # this is really more of a test of InstructionBLock jmp offset
        # calculations than it is a test of jmp itself
        p = self.make_pdp()

        a = InstructionBlock()
        a.clr('r0')
        a.clr('r1')
        a.clr('r2')
        a.jmp('X2')

        a.label('X0')
        a.inc('r0')
        a.add(2, 'r1')
        a.add(3, 'r2')
        a.jmp('X1')

        # never executed
        a.clr('r0')
        a.clr('r1')
        a.clr('r2')

        a.label('X1')
        a.inc('r1')
        a.add('r1', 'r2')
        a.add('r2', 'r0')
        a.halt()

        a.label('X2')
        a.inc('r2')
        a.jmp('X0')
        a.halt()

        instloc = 0o4000
        self.loadphysmem(p, a, instloc)

        p.run(pc=instloc)
        self.check16(p)

        # results by hand-computation but also cross verified in SIMH
        self.assertEqual(p.r[0], 8)
        self.assertEqual(p.r[1], 3)
        self.assertEqual(p.r[2], 7)

    def test_jsrco(self):
        """Another special case of the JSR instruction is JSR
        PC, @(SP) + which exchanges the top element of
        the processor stack and the contents of the program
        counter. Use of this instruction allows two
        routines to swap program control and resume operation
        when recalled where they left off. Such routines
        are called 'co-routines.'
        """
        p = self.make_pdp()
        a = InstructionBlock()
        # XXX TBD XXX

    def test_ubmap(self):
        p = self.make_pdp()

        ubmaps = self.ioaddr(p, p.ub.UBMAP_OFFS)

        # code paraphrased from UNIX startup, creates a mapping pattern
        # that the rest of the code expects (and fiddles upper bits)
        # So ... test that.
        for i in range(0, 62, 2):
            p.mmu.wordRW(ubmaps + (2 * i), (i << 12) & 0o177777)
            p.mmu.wordRW(ubmaps + (2 * (i + 1)), 0)

        # XXX there is no real test yet because the UBMAPs
        #     are all just dummied up right now

    # this is not a unit test, invoke it using timeit etc
    def speed_test_setup(self, instwords, /, *,
                         loopcount=200, usemmu=True, hz=0):
        """Set up a test run of the instwords (1 word or 2)
        Returns tuple: p, pc, per .. see speed_test_run()

        If len(instwords) is 1, make a loop of 49 instructions and one SOB
        If len == 2 (i.e., an instruction + operand), 24 instructions + SOB
        """

        # use loglevel WARNING to avoid a zillion start/halt logs
        p, pc = self.u64mapped_pdp(loglevel='WARNING')

        # the returned pdp is loaded with instructions for setting up
        # the mmu; only do them if that's what is wanted
        # NOTE: the test code is run in USER mode.

        if usemmu:
            if hz:
                clk = KW11HZ(p.ub, hz=hz)
                p.associate_device(clk, 'KW')

            p.run(pc=pc)             # set up all those mappings
            usermode_base = 0o10000  # phys 0o210000 maps here in USER mode
            user_physloc = 0o210000

            if hz:
                ihloc = 0o4000
                # the interrupt handler itself
                ih = InstructionBlock()
                ih.rti()
                for a2, w in enumerate(ih):
                    p.mmu.wordRW(ihloc + (2 * a2), w)

                # set up the vector
                p.mmu.wordRW(0o100, ihloc)
                p.mmu.wordRW(0o102, 0o340)

                # start the clock!
                p.mmu.wordRW(0o177546, 0o100)
        else:
            if hz:
                raise ValueError("Use of KW11 requires MMU")
            user_physloc = 0o20000
            usermode_base = user_physloc

        # kernel startup code at 0o5000 (NOTE: ih was at 0o4000)
        kloc = 0o5000

        # this is the tiny kernel code used to set up and start
        # each major iteration of the user mode timing code. The one-time
        # overhead of these few instructions is irrelevant in the timing test.

        k = InstructionBlock()
        k.mov(0o20000, 'sp')           # establish proper kernel stack
        k.mov(0o140000, '-(sp)')       # USER mode, spl 0
        k.mov(usermode_base, '-(sp)')  # pc start for loop/USER code
        # these environmental "knowns" are available for the test inst
        k.mov(0o1000, 'r5')            # usable writeable addr
        k.clr('r0')
        k.clr('r1')
        k.rtt()                        # off to the races!

        for a2, w in enumerate(k):
            p.mmu.wordRW(kloc + (2 * a2), w)

        if len(instwords) == 1:
            per = 49
        elif len(instwords) == 2:
            per = 24
        else:
            raise ValueError("instwords must be a list of length 1 or 2")

        # The test timing loop: N instructions and an SOB
        # NOTE: The instructions must not exxceed SOB branch reach
        a = InstructionBlock()
        a.mov(loopcount, 'r4')
        a.label('LOOP')
        for i in range(per):
            for w in instwords:
                a.literal(w)
        a.sob('r4', 'LOOP')
        a.halt()

        for a2, w in enumerate(a):
            p.physRW(user_physloc + (2 * a2), w)

        # per+1 will be 50 (len(instwords) == 1) or 25 ( == 2)
        return p, kloc, per+1

    def speed_test_run(self, p, instloc):
        """See speed_test_setup"""
        p.psw = 0
        p.run(pc=instloc)


if __name__ == "__main__":
    import argparse
    import timeit

    def asminst(s):
        try:
            return [asmint(s)]
        except ValueError:        # if, e.g., 'MOV r0,r1' instead of '010001'
            pass

        # all of these things can raise exceptions if the string is
        # ill-formatted, and that's perfectly fine (causes arg rejection)
        mnem, s2 = s.split()
        a = InstructionBlock()
        asmmeth = getattr(a, mnem.lower())
        asmmeth(*s2.split(','))
        return list(a)

    def asmint(s):
        digits = '0123456789'
        if s.startswith('0o'):
            base = 8
            s = s[2:]
            digits = '01234567'
        elif s.startswith('0x'):
            base = 16
            s = s[2:]
            digits += 'abcdef'
        elif s[-1] == '.':
            base = 10
            s = s[:-1]
        else:
            base = 8
        place = 1
        v = 0
        for d in reversed(s.lower()):
            dv = digits.index(d)           # ValueError for bad digits
            v += (dv * place)
            place *= base
        return v

    movr1r0 = [0o010100]

    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--performance', action="store_true")
    parser.add_argument('--clocktest', action="store_true")
    parser.add_argument('-i', '--instruction', default=movr1r0, type=asminst,
                        help="Test instruction, in octal. E.g.: 010203")
    parser.add_argument('--nommu', action="store_true")
    parser.add_argument('--hz', type=int, default=0)
    parser.add_argument('tests', nargs="*")
    args = parser.parse_args()

    if args.performance:
        # If the instruction is 1 word the loop will be 49 instructions
        # and one SOB, considered as 50 instructions.
        # If the instruction is 2 words it will be 24 instructions + SOB,
        # considered as 25 instructions.

        # the goal is to execute inst 1M times. The loop executes 'per'
        # instructions and 1 sob; Want to drive "number=" up more than
        # loopcount, so use
        #    loopcount=20     ... means 20*per instructions
        #    number=1000000/(loopcount*per)
        #
        # number will be 1000 for 1 instruction word, or 2000 for 2.

        t = TestMethods()
        mmu = not args.nommu
        instwords = args.instruction
        loopcount = 20
        p, pc, per = t.speed_test_setup(
            instwords, loopcount=loopcount, usemmu=mmu, hz=args.hz)

        number = 1000000 // (loopcount * per)
        ta = timeit.repeat(stmt='t.speed_test_run(p, pc)',
                           number=number, globals=globals(), repeat=50)
        tnsec = round(1000 * min(*ta), 1)
        ws = list(map(lambda w: f"{oct(w)[2:]:0>6s}", args.instruction))
        print(f"Instruction {ws} took {tnsec} nsecs")
    else:
        argv = None
        if args.clocktest:
            argv = [parser.prog]
            tests = [s for s in dir(TestMethods)
                     if s.startswith('clocktests_')]
            argv = [parser.prog] + ['TestMethods.' + s for s in tests]
            if tests:
                print(f"Tests to run: {tests}")
        unittest.main(argv=argv)
