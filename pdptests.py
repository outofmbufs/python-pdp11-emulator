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
from pdptraps import PDPTraps
import unittest
import random

from pdpasmhelper import PDP11InstructionAssembler as ASM

class TestMethods(unittest.TestCase):

    PDPLOGLEVEL = 'INFO'

    # DISCLAIMER ABOUT TEST CODING PHILOSOPHY:
    #   For the most part, actual PDP-11 machine code is created and
    #   used to establish the test conditions, as this provides additional
    #   (albeit haphazard) testing of the functionality. Occasionally it's
    #   just too much hassle to do that and the pdp object is manipulated
    #   directly via methods/attributes to establish conditions.
    #   There's no rhyme or reason in picking the approach for a given test.

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

        # Kernel data space PDR registers
        ns.KDSD0 = ns.KISD0 + 0o20

        # Kernel instruction space PAR registers
        ns.KISA0 = ns.KDSD0 + 0o20

        # Kernel data space PAR registers
        ns.KDSA0 = ns.KISA0 + 0o20

        # User mode similar
        ns.UISD0 = cls.ioaddr(p, p.mmu.APR_USER_OFFS)
        ns.UDSD0 = ns.UISD0 + 0o20
        ns.UISA0 = ns.UDSD0 + 0o20
        ns.UDSA0 = ns.UISA0 + 0o20

        ns.MMR0 = cls.ioaddr(p, p.mmu.MMR0_OFFS)

        return ns

    #
    # Create and return a test machine with a simple memory mapping:
    #    Kernel Instruction space seg 0 points to physical 0
    #    Kernel Data space segment 0 also points to physical 0
    #    User instruction space seg 0 points to physical 0o20000
    #    User Data space seg 0 points to physical 0o40000
    # and turns on the MMU
    #

    def simplemapped_pdp(self, p=None, addons=[]):
        if p is None:
            p = self.make_pdp()

        asm = ASM()

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

        asm.startblock()
        asm.mov(0o20000, 'sp')           # start system stack at 8k 

        # write the constants as described above
        asm.mov(0o22222, asm.ptr(0o20000)) 
        asm.mov(0o33333, asm.ptr(0o20002)) 
        asm.mov(0o44444, asm.ptr(0o40000)) 

        # point both kernel seg 0 PARs to physical zero
        asm.clr(asm.ptr(cn.KISA0)) 
        asm.clr(asm.ptr(cn.KDSA0)) 

        # kernel seg 7 D space PAR to I/O page (at 22-bit location)
        asm.mov(0o017760000 >> 6, asm.ptr(cn.KDSA0 + (7 * 2))) 

        # user I seg 0 to 0o20000, user D seg 0 to 0o40000
        asm.mov(0o20000 >> 6, asm.ptr(cn.UISA0)) 
        asm.mov(0o40000 >> 6, asm.ptr(cn.UDSA0)) 

        # set the PDRs for segment zero
        asm.mov(0o077406, 'r3') 
        # 77406 = PDR<2:0> = ACF = 0o110 = read/write
        #         PLF<14:8> =0o0774 = full length (128*64 bytes = 8K)

        asm.mov('r3', asm.ptr(cn.KISD0)) 
        asm.mov('r3', asm.ptr(cn.KDSD0)) 
        asm.mov('r3', asm.ptr(cn.UISD0)) 
        asm.mov('r3', asm.ptr(cn.UDSD0)) 

        # PDR for segment 7
        asm.mov('r3', asm.ptr(cn.KDSD0 + (7 * 2))) 

        # set previous mode to USER, keeping current mode KERNEL, pri 7
        asm.mov((p.KERNEL << 14) | (p.USER << 12) | (7 << 5),
                asm.ptr(self.ioaddr(p, p.PS_OFFS)))

        # turn on 22-bit mode, unibus mapping, and I/D sep for k & u
        asm.mov(0o000065, asm.ptr(self.ioaddr(p, p.mmu.MMR3_OFFS)))

        # turn on relocation mode ... yeehah! (MMR0 known zero here)
        asm.inc(asm.ptr(self.ioaddr(p, p.mmu.MMR0_OFFS)))


        asm.addtoblock(addons)
        asm.halt()
        setup_instructions = asm.endblock()

        instloc = 0o4000             # 2K
        
        self.loadphysmem(p, setup_instructions, instloc)
        return p, instloc

    # these tests end up testing a other stuff too of course, including MMU
    def test_mfpi(self):
        # ((r0, ..., rN) results, (instructions)), ...
        tvecs = (

            # r1=2, mfpi (r1) -> r0; expect r0 = 33333
            ((0o33333,), (0o012701, 0o02, 0o006511, 0o012600)),

            # r1=0, mfpi (r1) -> r0; expect r0 = 22222
            ((0o22222,), (0o012701, 0o00, 0o006511, 0o012600)),
            )

        for rslts, insts in tvecs:
            with self.subTest(rslts=rslts, insts=insts):
                p, pc = self.simplemapped_pdp(addons=insts)
                p.run(pc=pc)
                for rN, v in enumerate(rslts):
                    self.assertEqual(p.r[rN], v)

    def test_mfpxsp(self):
        cn = self.usefulconstants()
        insts = (
            # gotta turn mapping back off for these...
            0o005037, cn.MMR0,         # CLR MMR0
            0o012737, 0o14000, 0o34,   # mov $14000,*#34
            0o005037, 0o36,            # clear *#36 .. perfectly fine PSW

            0o012700, 0o20000,         # mov #20000,r0
            0o012720, 0o010206,       # put into user 0: mov r2,r6
            0o012720, 0o104400,       # put into user 2: trap 0

            0o012702, 0o123456,       # put 123456 into R2
            0o012746, 0o140340,       # push user-ish PSW onto kernel stack
            0o005046,                 # new user PC == 0
            0o005237, cn.MMR0,        # back on with the mapping!

            0o000006,                 # RTT -- goes to user mode, addr 0
        )

        p, pc = self.simplemapped_pdp(addons=insts)

        # put the trap handler at 14000 as expected
        traph = (
            0o106506,     # mfpd sp
            0o012603,     # pop stack into r3
            0
        )

        self.loadphysmem(p, traph, 0o14000)
        p.instlog = True
        p.run(pc=pc)
        self.assertEqual(p.r[2], p.r[3])

    def test_mtpi(self):
        # need an instance just for the constants, meh
        px = self.make_pdp()
        tvecs = (
            ((0o1717,), (0o012746, 0o1717, 0o006637, 0o02,
                         # turn MMU back off (!)
                         0o005037, self.ioaddr(px, px.mmu.MMR0_OFFS),
                         0o013700, 0o20002)),
            )
        for rslts, insts in tvecs:
            with self.subTest(rslts=rslts, insts=insts):
                p, pc = self.simplemapped_pdp(addons=insts)
                p.run(pc=pc)
                for rN, v in enumerate(rslts):
                    self.assertEqual(p.r[rN], v)

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

        p.physmem[add_loc >> 1] = 0o060001    # ADD R0,R1
        p.physmem[(add_loc >> 1) + 1] = 0
        p.physmem[sub_loc >> 1] = 0o160001    # SUB R0,R1
        p.physmem[(sub_loc >> 1) + 1] = 0

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

    def test_bne(self):
        p = self.make_pdp()
        loopcount = 0o1000
        insts = (
            # Program is:
            #         MOV loopcount,R1
            #         CLR R0
            #   LOOP: INC R0
            #         DEC R1
            #         BNE LOOP
            #         HALT
            0o012701, loopcount, 0o005000, 0o005200, 0o005301, 0o001375, 0)

        instloc = 0o4000
        self.loadphysmem(p, insts, instloc)

        p.run(pc=instloc)
        self.assertEqual(p.r[0], loopcount)
        self.assertEqual(p.r[1], 0)

    def test_cc(self):
        # various condition code tests
        p = self.make_pdp()
        insts = (
            # program is:
            #       CLR R0
            #       BEQ 1f
            #       HALT
            #    1: CCC
            #       BNE 1f
            #       HALT
            #    1: DEC R0

            #       MOV @#05000,R1      ; see discussion below
            #       MOV @#05002,R2      ; see discussion below
            #       CMP R1,R2
            #       BLE 1f
            #       HALT
            #    1: DEC R0
            #       CMP R2,R1
            #       BGT 1f
            #       HALT
            #    1: DEC R0
            #       HALT
            #
            # and the program will poke various test cases into locations
            # 5000 and 5002, with the proviso that 5000 is always the lesser.
            #
            # Given that, after running the program R0 should be 65553

            0o005000, 0o101401, 0o0, 0o000257, 0o001001, 0, 0o005300,

            # MOV @#5000 etc
            0o013701, 0o5000, 0o013702, 0o5002,

            # CMP R1,R2 BLE
            0o020102, 0o003401, 0, 0o005300,

            # CMP R2,R1 BGT
            0o020201, 0o003001, 0, 0o005300,

            0)

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
                self.assertEqual(p.r[0], 65533)

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
                self.assertEqual(p.r[0], 65533)

    def test_unscc(self):
        # more stuff like test_cc but specifically testing unsigned Bxx codes
        p = self.make_pdp()
        insts = (
            # program is:
            #       CLR R0
            #       MOV @#05000,R1      ; see discussion below
            #       MOV @#05002,R2      ; see discussion below
            #       CMP R1,R2
            #       BCS 1f              ; BCS same as BLO
            #       HALT
            #    1: DEC R0
            #       CMP R2,R1
            #       BHI 1f
            #       HALT
            #    1: DEC R0
            #       HALT
            #
            # test values in 5000,5002 .. unsigned and 5002 always higher
            #
            # Given that, after running the program R0 should be 65534

            0o005000,

            # MOV @#5000 etc
            0o013701, 0o5000, 0o013702, 0o5002,

            # CMP R1,R2 BCS
            0o020102, 0o103401, 0, 0o005300,

            # CMP R2,R1 BHI
            0o020201, 0o101001, 0, 0o005300,

            0)

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
        insts = (0o012702, 0o0122451,      # mov #122451,R2
                 0o072227, 0o0177772,      # ash -6,R2
                 0o042702, 0o0176000,      # bic #0176000,R2
                 0)                        # R2 should be 1224
        p = self.make_pdp()
        instloc = 0o4000
        self.loadphysmem(p, insts, instloc)
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
        handler = (
            # the saved PC is at the top of the stack ... get it
            0o011600,              # MOV (SP),R0
            # get the low byte of the instruction which is the trap code
            # note that the PC points after the TRAP instruction so
            # MOVB -2(R0),R3
            0o116003, 0o177776,
            # RTT
            6)
        self.loadphysmem(p, handler, 0o10000)

        # just bash a stack pointer directly in
        p.r[6] = 0o20000       # 8K and working down

        for i in range(256):
            insts = (
                0o104400 | i,      # TRAP #i
                0o010301,          # MOV R3,R1 just to show RTT worked
                0)
            self.loadphysmem(p, insts, 0o30000)
            p.run(pc=0o30000)
            self.assertEqual(p.r[3], p.r[1])

            # because the machine code did MOVB, values over 127 get
            # sign extended, so take that into consideration
            if i > 127:
                trapexpected = 0xFF00 | i
            else:
                trapexpected = i
            self.assertEqual(p.r[1], trapexpected)

    # test_mmu_1 .. test_mmu_N .. a variety of MMU tests.
    #
    # Any of the other tests that use simplemapped_pdp() implicitly
    # test some aspects of the MMU but these are more targeted tests.
    # NOTE: it's a lot easier to test via the methods than via writing
    #       elaborate PDP-11 machine code so that's what these do.

    def test_mmu_1(self):
        # test the page length field support
        p = self.make_pdp()

        # using ED=0 (segments grow upwards), create a (bizarre!)
        # user DSPACE mapping where the the first segment has length 0,
        # the second has 16, the third has 32 ... etc and then check
        # that that valid addresses map correctly and invalid ones fault
        # correctly. NOTE that there are subtle semantics to the so-called
        # "page length field" ... in a page that grows upwards, a plf of
        # zero means that to be INVALID the block number has to be greater
        # than zero (therefore "zero" length really means 64 bytes of
        # validity) and there is a similar off-by-one semantic to ED=1
        # downward pages. The test understands this.

        cn = self.usefulconstants()
        for segno in range(8):
            p.mmu.wordRW(cn.UDSA0 + (segno*2), (8192 * segno) >> 6)
            pln = segno * 16
            p.mmu.wordRW(cn.UDSD0 + (segno*2), (pln << 8) | 0o06)

        # enable user I/D separation
        p.mmu.MMR3 |= 0o01

        # turn on the MMU!
        p.mmu.MMR0 = 1

        for segno in range(8):
            basea = segno * 8192
            maxvalidoffset = 63 + ((segno * 64) * 16)
            for o in range(8192):
                if o <= maxvalidoffset:
                    _ = p.mmu.v2p(basea + o, p.USER, p.mmu.DSPACE,
                                  p.mmu.CYCLE.READ)
                else:
                    with self.assertRaises(PDPTraps.MMU):
                        _ = p.mmu.v2p(basea + o, p.USER, p.mmu.DSPACE,
                                      p.mmu.CYCLE.READ)

    def test_mmu_2(self):
        # same test as _1 but with ED=1 so segments grow downwards
        # test the page length field support
        p = self.make_pdp()

        cn = self.usefulconstants()
        for segno in range(8):
            p.mmu.wordRW(cn.UDSA0 + (segno*2), (8192 * segno) >> 6)
            pln = 0o177 - (segno * 16)
            p.mmu.wordRW(cn.UDSD0 + (segno*2), (pln << 8) | 0o16)

        # enable user I/D separation
        p.mmu.MMR3 |= 0o01

        # turn on the MMU!
        p.mmu.MMR0 = 1

        for segno in range(8):
            basea = segno * 8192
            minvalidoffset = 8192 - (64 + ((segno * 64) * 16))
            for o in range(8192):
                if o >= minvalidoffset:
                    _ = p.mmu.v2p(basea + o, p.USER, p.mmu.DSPACE,
                                  p.mmu.CYCLE.READ)
                else:
                    with self.assertRaises(PDPTraps.MMU):
                        _ = p.mmu.v2p(basea + o, p.USER, p.mmu.DSPACE,
                                      p.mmu.CYCLE.READ)

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
