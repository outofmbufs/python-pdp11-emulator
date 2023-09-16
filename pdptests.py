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

from machine import PDP1170
from branches import BRANCH_CODES
from pdptraps import PDPTraps
import unittest
import random

from pdpasmhelper import PDP11InstructionAssembler as ASM


class TestMethods(unittest.TestCase):

    PDPLOGLEVEL = 'INFO'

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
        self.loadphysmem(p, a.instructions(), instloc)
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
            tvecs.append((result, a.instructions()))

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
        user_mode_instructions = u.instructions()

        with ASM() as premmu:
            ts = premmu                    # just for brevity...
            ts.mov(0o14000, ts.ptr(0o34))  # set vector 034 to 14000
            ts.clr(ts.ptr(0o36))           # PSW for trap - zero work
            ts.mov(0o20000, 'r0')          # mov #20000,r0

            for uinst in user_mode_instructions:
                ts.mov(uinst, '(r0)+')
            ts.mov(0o123456, 'r2')         # mov #123456,r2
            ts.mov(0o140340, '-(sp)')      # push user-ish PSW to K stack
            ts.clr('-(sp)')                # new user PC = 0

        with ASM() as postmmu:
            postmmu.literal(6)             # RTT - goes to user mode, addr 0

        p, pc = self.simplemapped_pdp(premmu=premmu.instructions(),
                                      postmmu=postmmu.instructions())

        # put the trap handler at 14000 as expected
        with ASM() as th:
            th.mfpd('sp')
            th.mov('(sp)+', 'r3')
            th.halt()
        self.loadphysmem(p, th.instructions(), 0o14000)
        p.run(pc=pc)
        self.assertEqual(p.r[2], p.r[3])

    def test_mtpi(self):
        cn = self.usefulconstants()

        with ASM() as ts:
            ts.mov(0o1717, '-(sp)')        # pushing 0o1717
            ts.mtpi(ts.ptr(0o02))          # and MTPI it to user location 2
            ts.clr(ts.ptr(cn.MMR0))        # turn MMU back off
            ts.mov(ts.ptr(0o20002), 'r0')  # r0 = (020002)

        tvecs = ((0o1717, ts.instructions()),)

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
            for offs, inst in enumerate(a.instructions()):
                p.physmem[(loc >> 1) + offs] = inst

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
        self.loadphysmem(p, a.instructions(), instloc)

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
            a.beq(+1)
            a.halt()                # stop here if BEQ fails
            a.literal(0o000257)     # 1f: CCC .. clear all the condition codes
            a.bne(+1)
            a.halt()                # stop here if BNE fails
            a.mov(goodval, 'r1')    # indicate success
            a.halt()

        instloc = 0o4000
        self.loadphysmem(p, a.instructions(), instloc)
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
        return a.instructions()

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
            a.mov(0o0122451, 'r2')           # mov #122451,R2
            a.literal(0o072200, 0o0177772)   # ash -6,R2
            a.bic(0o0176000, 'r2')           # bic #0176000,R2
            a.halt()

        p = self.make_pdp()
        instloc = 0o4000
        self.loadphysmem(p, a.instructions(), instloc)
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
        for a, w in enumerate(insts, start=(baseloc >> 1)):
            p.physmem[a] = w

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

        self.loadphysmem(p, handler.instructions(), 0o10000)

        # just bash a stack pointer directly in
        p.r[6] = 0o20000       # 8K and working down

        for i in range(256):
            with ASM() as a:
                a.trap(i)          # TRAP #i
                a.mov('r3', 'r1')  # MOV R3,R1 just to show RTT worked
                a.halt()

            self.loadphysmem(p, a.instructions(), 0o30000)
            p.run(pc=0o30000)
            self.assertEqual(p.r[3], p.r[1])

            # because the machine code did MOVB, values over 127 get
            # sign extended, so take that into consideration
            if i > 127:
                trapexpected = 0xFF00 | i
            else:
                trapexpected = i
            self.assertEqual(p.r[1], trapexpected)

    def test_mmu_updown(self):
        # test the page length field support in both up and down directions

        # XXX whether it was wise to code this test as a magnum opus
        #     of assembler prowess is well open to debate. On the plus
        #     side, it certainly exercises a bunch of features besides
        #     just testing the MMU page length functionality.

        cn = self.usefulconstants()
        p = self.make_pdp()

        # Two tests - up and down.
        #
        # In both tests, KERNEL I space page 0 is mapped to physical 0
        # and KERNEL I space page 7 is mapped to the I/O page.
        # I/D separation is NOT enabled for KERNEL.
        #
        # USER I space is mapped to 0o20000.
        # All 64K of USER D space is mapped to 64K of physical memory
        # from 0o200000 (not a typo) to 0o400000 (not a typo), but with
        # a bizarre page length scheme according to UP or DOWN phase of
        # the test as below. I/D separation is (obviously) enabled for USER.
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

        # these instructions do initialization common to both up/down cases
        kernel_addr = 0o6000      # arbitrary start for all this
        traps_addr = 0o4000       # make sure enough room before kernel_addr

        # this code will go at traps_addr
        with ASM() as tr:
            # The trap handler for MMU faults and the trap 0 / trap 1 from
            # the user code (when there is no MMU fault). It is integrated
            # into one routine.
            #
            # The user code gets to use r0 and r1, the trap handler
            # gets to use r2-r5:
            #      r2: expected good flag (initialized elsewhere)
            #      r3: determined good flag (by trap entry)
            #      r5: TESTTABLE pointer (initialized elsewhere)
            tr.label('UTrap')

            # first determine if trap0 (bad) or trap1 (good)
            tr.mov('(sp)', 'r3')       # get user PC from trap frame
            tr.mfpi('-2(r3)')          # get the trap instruction
            tr.mov('(sp)+', 'r3')      # r3 is now the trap instruction
            tr.bic(0o177400, 'r3')
            tr.cmp(1, 'r3')
            tr.beq(1)
            # this was not a "good" trap, the user code failed
            tr.halt()

            tr.br(1)                # skip over the MMU entry point

            tr.label('TrapMMU')
            tr.clr('r3')            # indicate MMU fault case

            # see if the access was good/bad as expected
            tr.cmp('r2', 'r3')
            tr.beq(1)               # jump over the HALT
            tr.halt()               # NOPE, something wrong!

            # see if it is time to switch to next table entry
            tr.add(2, 'r0')      # didn't rely on (r0)+ vs MMU semantic
            tr.cmp('2(r5)', 'r0')
            tr.bne(7)               # skip over the "time to switch" stanza

            # it is time to switch
            tr.add(4, 'r5')
            tr.mov('(r5)', 'r2')
            tr.cmp(0o666, 'r2')
            tr.bne(1)
            tr.halt()            # test done; success if r2 = 0o666

            # next iteration of the user code loop
            tr.clr('(sp)')       # put user PC back to zero
            tr.rtt()

        # this code goes at kernel_addr
        with ASM() as a:
            a.mov(0o20000, 'sp')      # start system stack at 8k
            # KERNEL I SPACE
            #    PAR 0 to physical 0
            #    PAR 7 to physical 760000 and 22bit not turned on
            #
            #    PDR 77406 = read/write, full length
            a.clr(a.ptr(cn.KISA0))
            a.mov(0o760000 >> 6, a.ptr(cn.KISA7))
            a.mov(0o077406, a.ptr(cn.KISD0))
            a.mov(0o077406, a.ptr(cn.KISD7))

            # USER I SPACE
            a.mov(0o20000 >> 6, a.ptr(cn.UISA0))
            a.mov(0o077406, a.ptr(cn.UISD0))

            # USER D SPACE going UP...
            a.mov(cn.UDSD0, 'r3')   # will walk through D0 .. D7
            # NOTE: A0 .. A7 is 040(r3)
            a.clr('r0')             # r0: segno*2 = (0, 2, 4, .., 14)
            a.mov(0o2000, 'r4')     # phys addr base (0o200000>>6)

            a.label('PARloop')

            a.mov('r4', '040(r3)')    # set U PAR; don't bump r3 yet
            a.add(0o200, 'r4')        # 0o200 = 8192>>6

            a.mov('r0', 'r2')         # r2 = segno*2
            a.ash(3, 'r2')            # r2 = segno*16
            a.swab('r2')              # really (segno*16)<<8
            a.add(0o06, 'r2')         # ACF r/w segment
            a.mov('r2', '(r3)+')      # set U PDR
            a.inc('r0')               # bump r0 by two
            a.inc('r0')
            a.cmp('r0', 16)           # and loop until done all 8 segments
            a.blt('PARloop')

            a.bis(1, a.ptr(cn.MMR3))   # enable I/D sep just for USER
            a.mov(1, a.ptr(cn.MMR0))   # turn on MMU

            # create the test table, just push it onto the stack (yeehah!)
            a.mov(0, '-(sp)')           # this is a PAD (not really needed)
            a.mov(0o666, '-(sp)')       # this is a sentinel
            a.mov(0o176100, '-(sp)')
            a.mov(1, '-(sp)')
            a.mov(0o160000, '-(sp)')
            a.mov(0, '-(sp)')
            a.mov(0o154100, '-(sp)')
            a.mov(1, '-(sp)')
            a.mov(0o140000, '-(sp)')
            a.mov(0, '-(sp)')
            a.mov(0o132100, '-(sp)')
            a.mov(1, '-(sp)')
            a.mov(0o120000, '-(sp)')
            a.mov(0, '-(sp)')
            a.mov(0o110100, '-(sp)')
            a.mov(1, '-(sp)')
            a.mov(0o100000, '-(sp)')
            a.mov(0, '-(sp)')
            a.mov(0o66100, '-(sp)')
            a.mov(1, '-(sp)')
            a.mov(0o60000, '-(sp)')
            a.mov(0, '-(sp)')
            a.mov(0o44100, '-(sp)')
            a.mov(1, '-(sp)')
            a.mov(0o40000, '-(sp)')
            a.mov(0, '-(sp)')
            a.mov(0o22100, '-(sp)')
            a.mov(1, '-(sp)')
            a.mov(0o20000, '-(sp)')
            a.mov(0, '-(sp)')
            a.mov(0o100, '-(sp)')
            a.mov(1, '-(sp)')

            # the test table for the trap handler is now here:
            a.mov('sp', 'r5')
            # test starts in the region at the start of the table
            a.mov('(r5)', 'r2')

            # ok, now ready to start the user program
            a.mov(0o140340, '-(sp)')   # push user-ish PSW to K stack
            a.clr('-(sp)')             # new user PC = 0
            a.clr('r0')                # user test expects r0 to start zero
            a.rtt()

            # these instructions will be used to switch over
            # to the DOWN phase of the test. Similar to the UP but
            # don't have to do the PARs (they stay the same) and the
            # pln calculations are different.

            a.label('DOWN')
            a.mov(cn.UDSD0, 'r3')
            a.clr('r0')
            a.label('PARloopDOWN')
            # compute segno * 8 in r2 (r0 starts as segno*2)
            a.mov('r0', 'r2')
            a.ash(3, 'r2')
            # pln = 0o177 - (segno * 16)
            a.mov(0o177, 'r1')
            a.sub('r2', 'r1')
            a.mov('r1', 'r2')
            a.swab('r2')
            a.add(0o16, 'r2')    # the downward growing case
            a.mov('r2', '(r3)+')      # set U PDR
            a.inc('r0')
            a.inc('r0')
            a.cmp('r0', 16)
            a.blt('PARloopDOWN')

            # this halt will be right before the first run of user mode test
            a.halt()

            a.clr('r0')        # initial loop condition
            a.clr('(sp)')      # just knows the user loop starts at zero
            a.rtt()

            # Now for something extra frosty... relocate just segment 4
            # (arbitrarily chosen) of the user memory to a different
            # physical page and run the test again to ensure it still works.
            # This will make use of KERNEL A1 and A2 segments to map the
            # relocation (note: I space because no sep I/D for kernel here)
            a.label('BONUS')

            # copy UDSA4 into KISA1 - mapping old segment into kernel space
            a.mov(a.ptr(cn.UDSA0 + 4*2), a.ptr(cn.KISA0 + 2))  # i.e., A1

            # the new location for this data will be physical 0o600000
            # (not a typo) which becomes 0o6000 in the PAR
            a.mov(0o6000, a.ptr(cn.KISA0 + 4))                 # i.e., A2

            # the standard PDR access/full-length/etc bits
            a.mov(0o077406, a.ptr(cn.KISD0 + 2))
            a.mov(0o077406, a.ptr(cn.KISD0 + 4))

            # count r0, source address r1, destination r2
            a.mov(4096, 'r0')
            a.mov(8192, 'r1')
            a.mov(8192*2, 'r2')
            a.mov('(r1)+', '(r2)+')
            a.literal(0o077002)            # SOB to the copy

            # switch the user page to the new mapping
            a.mov(0o6000, a.ptr(cn.UDSA0 + 4*2))

            # and the standard initialization/resume dance
            a.halt()
            a.clr('r0')
            a.clr('(sp)')      # just knows the user loop starts at zero
            a.rtt()

        # poke the trap handler vector (250)
        pcps = [traps_addr + (tr.labels['TrapMMU'] * 2), 0o340]
        self.loadphysmem(p, pcps, 0o250)

        # same for the "trap N" handler
        pcps = [traps_addr + (tr.labels['UTrap'] * 2), 0o340]
        self.loadphysmem(p, pcps, 0o34)

        # all those trap instructions
        self.loadphysmem(p, tr.instructions(), traps_addr)

        # all those kernel instructions
        self.loadphysmem(p, a.instructions(), kernel_addr)

        # user mode program:
        #    read the given address: mov (r0),r1
        #    If this causes an MMU fault, it goes to the MMU trap handler
        #    If it succeeds, it then hits the trap(1) and goes to that
        #    handler. The kernel trap handler, using the test table,
        #    can then validate that the right thing happened.
        #    The code "loops" only because the kernel trap handler resets
        #    the PC to zero and bumps r0 then returns to user mode for the
        #    next iteration. (Yes, all this could have been done with mfpd
        #    but that feels like a different test than this)

        user_phys_ISPACEaddr = 0o20000
        with ASM() as u:
            # this subtract combines the access check with part1 of chksum
            u.mov(0o123456, 'r1')
            u.sub('(r0)', 'r1')
            u.cmp('r0', 'r1')
            u.beq(1)
            u.trap(0o77)
            u.trap(1)               # indicate good status
            u.halt()                # never get here, this is illegal
        self.loadphysmem(p, u.instructions(), user_phys_ISPACEaddr)

        # set the physical memory that will be mapped to user D
        # space to this pattern so the test can verify the mapping
        checksum = 0o123456         # arbitrary
        user_phys_DSPACEbase = 0o200000
        words = (checksum - (user_phys_DSPACEbase + o) & 0o177777
                 for o in range(0, 65536, 2))
        self.loadphysmem(p, words, user_phys_DSPACEbase)

        # finally ready to run the whole shebang!
        p.run(pc=kernel_addr)

        # a halt was encountered, verify r2 is the end sentinel
        self.assertEqual(p.r[2], 0o666)
        return

        # this will be used for both up/down testing, based on goodf
        def _test(goodf):
            previous_good = True

            for segno in range(8):
                for o in range(4096):
                    p.run()            # picks up at rtt pc
                    physval = (checksum -
                               ((segno * 8192) + (o * 2))) & 0o177777
                    if goodf(segno, o*2):
                        r5_expected = 0o42
                        r1_expected = physval
                        if not previous_good:
                            print(f"bad to good at r0={oct(p.r[0])}, "
                                  f"addr = {oct(segno*8192 + (2*o))}")
                            previous_good = True
                    else:
                        r5_expected = 0o666
                        r1_expected = user_noval
                        if previous_good:
                            print(f"good to bad at r0={oct(p.r[0])}, "
                                  f"addr = {oct(segno*8192 + (2*o))}")
                            previous_good = False

                    self.assertEqual(p.r[1], r1_expected)
                    self.assertEqual(p.r[5], r5_expected)

        # run the UP test:
        _test(lambda _segno, _o: _o <= (63 + ((_segno * 64) * 16)))

        # run the code to convert over to DOWN MMU format, and then the test
        p.run(pc=kernel_addr + (a.labels['DOWN'] * 2))
        _test(lambda _segno, _o: _o >= 8192 - (64 + ((_segno * 64) * 16)))

        # last but not least, the BONUS test
        p.run(pc=kernel_addr + (a.labels['BONUS'] * 2))
        _test(lambda _segno, _o: _o >= 8192 - (64 + ((_segno * 64) * 16)))

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
    def speed_test_setup(self, *, loopcount=10000, mmu=True, inst=None):

        p, pc = self.simplemapped_pdp()

        # the returned pdp is loaded with instructions for setting up
        # the mmu; only do them if that's what is wanted
        if mmu:
            p.run(pc=pc)

        # by default the instruction being timed will be MOV R1,R0
        # but other instructions could be used. MUST ONLY BE ONE WORD
        if inst is None:
            inst = 0o010100

        # now load the test timing loop... 9 MOV R1,R0 instructions
        # and an SOB for looping (so 10 instructions per loop)

        insts = (0o012704, loopcount,        # loopcount into R4
                 inst,
                 inst,
                 inst,
                 inst,
                 inst,
                 inst,
                 inst,
                 inst,
                 inst,

                 0o077412,      # SOB R4 back to first inst
                 0)             # HALT

        instloc = 0o4000
        for a2, w in enumerate(insts):
            p.mmu.wordRW(instloc + (2 * a2), w)
        return p, instloc

    def speed_test_run(self, p, instloc):
        p.run(pc=instloc)


if __name__ == "__main__":
    unittest.main()
