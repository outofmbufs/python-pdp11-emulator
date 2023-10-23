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
from branches import BRANCH_CODES
from pdptraps import PDPTraps
import unittest
import random
import os
import hashlib

from pdpasmhelper import PDP11InstructionAssembler as ASM


class TestMethods(unittest.TestCase):

    PDPLOGLEVEL = 'WARNING'

    # used to create various instances, collects all the options
    # detail into this one place... mostly this is about loglevel
    @classmethod
    def make_pdp(cls):
        return PDP1170(loglevel=cls.PDPLOGLEVEL)

    @staticmethod
    def ioaddr(p, offs):
        """Given a within-IO-page IO offset, return an IO addr."""
        return (offs + p.mmu.iopage_base) & 0o177777

    # convenience routine to load word values into physical memory
    @staticmethod
    def loadphysmem(p, words, addr):
        for a, w in enumerate(words, start=(addr >> 1)):
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

        with ASM() as a:
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

    # these tests end up testing other stuff too of course, including MMU
    def test_mfpi(self):

        tvecs = []

        for result, r1tval in ((0o33333, 2), (0o22222, 0)):
            # r1=r1tval, mfpi (r1) -> r0; expect r0 = result
            with ASM() as a:
                a.mov(r1tval, 'r1')
                a.mfpi('(r1)')
                a.mov('(sp)+', 'r0')
            tvecs.append((result, list(a)),)

        for result, insts in tvecs:
            with self.subTest(result=result, insts=insts):
                p, pc = self.simplemapped_pdp(postmmu=insts)
                p.run(pc=pc)
                self.assertEqual(p.r[0], result)

    def test_mfpxsp(self):
        cn = self.usefulconstants()

        with ASM() as u:
            u.mov('r2', 'r6')
            u.trap(0)

        with ASM() as premmu:
            ts = premmu                    # just for brevity...
            ts.mov(0o14000, ts.ptr(0o34))  # set vector 034 to 14000
            ts.clr(ts.ptr(0o36))           # PSW for trap - zero work
            ts.mov(0o20000, 'r0')          # mov #20000,r0

            for uinst in u:
                ts.mov(uinst, '(r0)+')
            ts.mov(0o123456, 'r2')         # mov #123456,r2
            ts.mov(0o140340, '-(sp)')      # push user-ish PSW to K stack
            ts.clr('-(sp)')                # new user PC = 0

        with ASM() as postmmu:
            postmmu.literal(6)             # RTT - goes to user mode, addr 0

        p, pc = self.simplemapped_pdp(premmu=premmu, postmmu=postmmu)

        # put the trap handler at 14000 as expected
        with ASM() as th:
            th.mfpd('sp')
            th.mov('(sp)+', 'r3')
            th.halt()
        self.loadphysmem(p, th, 0o14000)
        p.run(pc=pc)
        self.assertEqual(p.r[2], p.r[3])

    def test_mtpi(self):
        cn = self.usefulconstants()

        with ASM() as ts:
            ts.mov(0o1717, '-(sp)')        # pushing 0o1717
            ts.mtpi(ts.ptr(0o02))          # and MTPI it to user location 2
            ts.clr(ts.ptr(cn.MMR0))        # turn MMU back off
            ts.mov(ts.ptr(0o20002), 'r0')  # r0 = (020002)

        tvecs = ((0o1717, ts),)

        for r0result, insts in tvecs:
            with self.subTest(r0result=r0result, insts=insts):
                p, pc = self.simplemapped_pdp(postmmu=insts)
                p.run(pc=pc)
                self.assertEqual(p.r[0], r0result)

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
            with ASM() as a:
                getattr(a, addsub)('r0', 'r1')
                a.halt()
            self.loadphysmem(p, a, loc)

        for r0, r1, added, a_nzvc, subbed, s_nzvc in testvecs:
            with self.subTest(r0=r0, r1=r1, op="add"):
                p.r[0] = r0
                p.r[1] = r1
                p.run(pc=add_loc)
                self.assertEqual(p.r[1], added)
                if a_nzvc is not None:
                    self.assertEqual(p.psw & 0o17, a_nzvc)

            with self.subTest(r0=r0, r1=r1, op="sub"):
                p.r[0] = r0
                p.r[1] = r1
                p.run(pc=sub_loc)
                self.assertEqual(p.r[1], subbed)
                if s_nzvc is not None:
                    self.assertEqual(p.psw & 0o17, s_nzvc)

    # test BNE (and, implicitly, INC/DEC)
    def test_bne(self):
        p = self.make_pdp()
        loopcount = 0o1000

        with ASM() as a:
            # Program is:
            #         MOV loopcount,R1
            #         CLR R0
            #   LOOP: INC R0
            #         DEC R1
            #         BNE LOOP
            #         HALT
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
        self.assertEqual(p.r[0], loopcount)
        self.assertEqual(p.r[1], 0)

    # test BEQ and BNE (BNE was also tested in test_bne)
    def test_eqne(self):
        p = self.make_pdp()

        goodval = 0o4321            # arbitrary, not zero
        with ASM() as a:
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
        self.assertEqual(p.r[1], goodval)

    # create the instruction sequence shared by test_cc and test_ucc
    def _cc_unscc(self, br1, br2):
        with ASM() as a:
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
                self.assertEqual(p.r[0], 65534)

    def test_ash1(self):
        # this code sequence taken from Unix startup, it's not really
        # much of a test.
        with ASM() as a:
            a.mov(0o0122451, 'r2')
            neg6 = -6 & 0xFFFF
            a.ash(neg6, 'r2')
            a.bic(0o0176000, 'r2')
            a.halt()

        p = self.make_pdp()
        instloc = 0o4000
        self.loadphysmem(p, a, instloc)
        p.run(pc=instloc)
        self.assertEqual(p.r[2], 0o1224)

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

            with self.subTest(offset=offset):
                self.assertEqual(p.r[0], 17)
                self.assertEqual(p.r[1], expected_R1)
            expected_R1 = (expected_R1 - 1) & 0o177777

    def test_div(self):
        # test the div instruction
        # The 32-bit int in R and R|1 is divided by the src operand

        p = self.make_pdp()

        with ASM() as a:
            # The test cases will be X / Y:
            #    X : 1, 255, 4096, 10017, 32767, 32768, 32769
            #        and then those same values with 690000 added to them
            #    Y : -50 .. 50 but skipping 0 and using a large number
            #
            # The code is written this way so that the resulting ASM()
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
        with ASM() as handler:
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
            with ASM() as a:
                a.trap(i)          # TRAP #i
                a.mov('r3', 'r1')  # MOV R3,R1 just to show RTT worked
                a.halt()

            self.loadphysmem(p, a, 0o30000)
            p.run(pc=0o30000)
            self.assertEqual(p.r[3], p.r[1])

            # because the machine code did MOVB, values over 127 get
            # sign extended, so take that into consideration
            if i > 127:
                trapexpected = 0xFF00 | i
            else:
                trapexpected = i
            self.assertEqual(p.r[1], trapexpected)

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
        with ASM() as tr:
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

        with ASM() as u:
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

        with ASM() as k:
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
            # add the offset to (forward ref) back_from_u
            k.add(k.getlabel('back_from_u'), '(sp)')

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
        p = self.make_pdp()

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

            # a halt was encountered, verify r2 is the end sentinel
            self.assertEqual(p.r[2], 0o666)

        with self.subTest(phase="DOWN"):
            # run the down test
            p.r[2] = 0            # superfluous but makes sure
            p.run(pc=downtest)
            self.assertEqual(p.r[2], 0o666)

        with self.subTest(phase="BONUS"):
            # and the bonus test
            p.r[2] = 0            # superfluous but makes sure
            p.run(pc=bonus)
            self.assertEqual(p.r[2], 0o666)

    def test_mmu_AWbits(self):
        cn = self.usefulconstants()
        p = self.make_pdp()

        base_address = 0o10000

        with ASM() as k:
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
        self.assertEqual(p.r[0], 0)

    def test_stacklim0(self):
        # verify that simply *having* an illegal SP doesn't trap
        p = self.make_pdp()
        with ASM() as a:
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
        self.assertEqual(p.r[0], 3)    # confirm made it all the way through

    def _stacklimcode(self, go_red=False):
        # memory usage:
        # 0o4000.. is the test code
        # 0o6000.. is the trap handler
        # 0o7000.. is the log of various values collected in the trap handler

        # r5 is used to walk through the 0o7000+ storage, it is initialized
        # in the test code and used in the trap handler

        p = self.make_pdp()

        with ASM() as tr:
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
        with ASM() as a:
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

    def test_breakpoints1(self):
        # test the steps=N breakpoint capability

        p = self.make_pdp()

        maxtest = 100
        with ASM() as a:
            for i in range(maxtest):
                a.mov(i, 'r0')
            a.clr('r0')
            a.halt()

        startaddr = 0o4000
        self.loadphysmem(p, a, startaddr)

        for i in range(maxtest):
            with self.subTest(i=i):
                p.run_steps(pc=startaddr, steps=i+1)
                self.assertEqual(p.r[0], i)

    def test_breakpoints2(self):
        # test the PCBreakpoint ('run_until') breakpoint capability

        p = self.make_pdp()

        maxtest = 100
        with ASM() as a:
            for i in range(maxtest):
                a.mov(i, 'r0')
                a.label(f"L{i}")
            a.clr('r0')
            a.halt()

        startaddr = 0o4000
        self.loadphysmem(p, a, startaddr)

        for i in range(maxtest):
            with self.subTest(i=i):
                p.run_until(pc=startaddr, stoppc=startaddr+a.getlabel(f"L{i}"))
                self.assertEqual(p.r[0], i)

    def test_breakpoints3(self):
        # test multiple breakpoints
        p = self.make_pdp()

        maxtest = 100
        with ASM() as a:
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

        # this should have fired after 1 instruction...
        self.assertEqual(p.r[0], 1)

        # the next two similar
        p.run(breakpoint=mbp)
        self.assertEqual(p.r[0], 50)
        p.run(breakpoint=mbp)
        self.assertEqual(p.r[0], 75)

        # the last one will complete because of the HALT
        p.run(breakpoint=mbp)
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
        with ASM() as a:
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
            with ASM() as a:
                for i in range(curguess):
                    a.mov(i, 'r0')
                a.halt()
            self.loadphysmem(p, a, startaddr)
            bp = BKP.Lookback()
            p.run(pc=startaddr, breakpoint=bp)
            # if current == first, there was 1 lookback, that's 1
            # But also the halt instruction takes up on; hence +2
            n = (p.r[0] - bp.states[0][1]['R0']) + 2
            if n < curguess:
                default_lookbacks = n
                break
            curguess += 1

        maxtest = default_lookbacks + 1
        with ASM() as a:
            for i in range(maxtest):
                a.mov(i, 'r0')
            a.clr('r0')
            a.halt()

        for i in range(maxtest):
            bp = BKP.Lookback(BKP.StepsBreakpoint(steps=i+1))
            bp7 = BKP.Lookback(BKP.StepsBreakpoint(steps=i+1), lookbacks=7)
            with self.subTest(i=i):
                p.run(pc=startaddr, breakpoint=bp)
                p.run(pc=startaddr, breakpoint=bp7)
                self.assertEqual(p.r[0], i)
                if i+1 <= default_lookbacks:
                    self.assertEqual(len(bp.states), i+1)
                else:
                    self.assertEqual(len(bp.states), default_lookbacks)
                self.assertEqual(len(bp7.states), min(i+1, 7))

    def test_jmp(self):
        """In many ways more of a test of ASM module labels..."""
        p = self.make_pdp()

        with ASM() as a:
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

        print("\n")
        print(list(map(oct, a)))

        instloc = 0o4000
        self.loadphysmem(p, a, instloc)
        p.run(pc=instloc)
        print("\n", p.machinestate())

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
        with ASM() as a:
            pass

    def test_ubmap(self):
        p = self.make_pdp()

        ubmaps = self.ioaddr(p, p.ub.UBMAP_OFFS)

        # code paraphrased from UNIX startup, creates a mapping pattern
        # that the rest of the code expects (and fiddles upper bits)
        # So ... test that.
        for i in range(0, 62, 2):
            p.mmu.wordRW(ubmaps + (2 * i), i << 12 & 0o1777777)
            p.mmu.wordRW(ubmaps + (2 * (i + 1)), 0)

        # XXX there is no real test yet because the UBMAPs
        #     are all just dummied up right now

    # this is not a unit test, invoke it using timeit etc
    def speed_test_setup(self, *, loopcount=200, mmu=True, inst=None):
        """Set up a test run of 50*loopcount inst instructions.

        Returns tuple: p, pc
        """

        p, pc = self.simplemapped_pdp()

        # the returned pdp is loaded with instructions for setting up
        # the mmu; only do them if that's what is wanted
        #
        # NOTE: the test code is run in USER mode Because Reasons
        # (was experimenting with virtual caches and this was helpful).
        # The test code will run at (virtual/user) 0 when the MMU is
        # enabled, or its physical location (0o20000) when off.

        user_physloc = 0o20000
        if mmu:
            p.run(pc=pc)         # set up all those mappings
            usermode_base = 0    # physical 0o20000 maps here in USER mode
        else:
            usermode_base = user_physloc

        # by default the instruction being timed will be MOV R1,R0
        # but other instructions could be used. MUST ONLY BE ONE WORD
        if inst is None:
            inst = 0o010100

        # this is the tiny kernel code used to set up and start
        # each iteration of the user mode timing code. It slightly
        # distorts the per-instruction overhead of course. C'est la vie.
        with ASM() as k:
            k.mov(0o20000, 'sp')           # establish proper kernel stack
            k.mov(0o140340, '-(sp)')       # USER mode, no interrupts
            k.mov(usermode_base, '-(sp)')  # pc start for loop/USER code
            k.rtt()                        # off to the races!

        kloc = 0o4000
        for a2, w in enumerate(k):
            p.mmu.wordRW(kloc + (2 * a2), w)

        # The test timing loop... 49 "inst" instructions
        # and an SOB for looping (so 50 overall instructions per loop)
        with ASM() as a:
            a.mov(loopcount, 'r4')
            a.label('LOOP')
            for i in range(49):
                a.literal(inst)
            a.sob('r4', 'LOOP')
            a.halt()

        for a2, w in enumerate(a):
            p.physRW(user_physloc + (2 * a2), w)

        return p, kloc

    def speed_test_run(self, p, instloc):
        """See speed_test_setup"""
        p.run(pc=instloc)


if __name__ == "__main__":
    import argparse
    import timeit

    movr1r0 = 0o010100

    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--performance', action="store_true")
    parser.add_argument('-i', '--instruction', default=movr1r0, type=int)
    parser.add_argument('--nommu', action="store_true")
    parser.add_argument('--clr', action="store_true")
    args = parser.parse_args()

    if args.performance:
        # the goal is to execute inst 1M times. The loop executes 49 inst
        # instructions and 1 sob (which taken together are considered as 50).
        # Want to drive "number=" up more than loopcount, so use
        #    loopcount=20     ... means "1000" inst instructions
        #    number=1000       ... do that 1000 times, for 1M instructions

        # simple way to test CLR instruction vs default MOV.
        # The CLR instruction is not optimized the way MOV is so
        # this shows the difference.
        if args.clr:
            args.instruction = 0o005000

        t = TestMethods()
        mmu = not args.nommu
        inst = args.instruction
        p, pc = t.speed_test_setup(loopcount=20, inst=inst, mmu=mmu)
        ta = timeit.repeat(stmt='t.speed_test_run(p, pc)',
                           number=1000, globals=globals(), repeat=50)
        tnsec = round(1000 * min(*ta), 1)
        if args.instruction == movr1r0:
            instr = 'MOV R1,R0'
        elif (args.instruction & 0o177770) == 0o005000:
            instr = f'CLR R{args.instruction & 7}'
        else:
            instr = oct(args.instruction)
        print(f"Instruction {instr} took {tnsec} nsecs")
    else:
        unittest.main()
