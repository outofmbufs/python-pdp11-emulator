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

from pdptraps import PDPTraps

from op000 import op000_dispatcher
from branches import branches


def op00_4_jsr(cpu, inst):
    Rn = (inst & 0o700) >> 6
    if Rn == cpu.SP:
        raise PDPTraps.ReservedInstruction

    # according to the PDP11 handbook...
    #     R7 is the only register that can be used for
    #     both the link and destination, the other GPRs cannot.
    #     Thus, if the link is R5, any register except R5 can be used
    #     for one destination field.
    #
    # Does the PDP11 Trap if this is violated, or is it just "undefined"??
    if Rn != cpu.PC and Rn == (inst & 0o07):
        raise PDPTraps.ReservedInstruction

    # Note that the computed b6 operand address IS the new PC value.
    # In other words, JSR PC,(R0) means that the contents of R0 are
    # the subroutine address. This is one level of indirection less
    # than ordinary instructions. Hence the justEA for operandx().
    # Corollary: JSR PC, R0 is illegal (instructions cannot reside
    # in the registers themselves)

    tmp = cpu.operandx(inst & 0o77, justEA=True)

    # NOTE: no condition code modifications

    # cpu.logger.debug(f"JSR to {oct(tmp)} from {oct(cpu.r[cpu.PC])}")
    cpu.mmu.MMR1mod(0o366)           # the encoding for -2 on sp
    cpu.stackpush(cpu.r[Rn])
    cpu.r[Rn] = cpu.r[cpu.PC]        # this could be a no-op if Rn == 7
    cpu.r[cpu.PC] = tmp


def op00_50_clr(cpu, inst, opsize=2):
    """CLR(B) (determined by opsize). Clear destination."""

    cpu.psw_n = cpu.psw_v = cpu.psw_c = 0
    cpu.psw_z = 1

    dstb6 = (inst & 0o77)
    # optimize the common register case
    if opsize == 2 and dstb6 < 8:
        cpu.r[dstb6] = 0
    else:
        cpu.operandx(dstb6, 0, opsize=opsize)


def op00_51_com(cpu, inst, opsize=2):
    """COM(B) (determined by opsize). 1's complement destination."""
    val, xb6 = cpu.operandx(inst & 0o77, opsize=opsize, rmw=True)
    # Have to be careful about python infinite length integers
    # For example, ~0xFFFF == -65536 whereas the desired result is zero.
    # Hence the explicit masking
    val = (~val) & cpu.MASK816[opsize]

    cpu.psw_n = True if val & cpu.SIGN816[opsize] else False
    cpu.psw_z = (val == 0)
    cpu.psw_v = 0
    cpu.psw_c = 1

    cpu.operandx(xb6, val, opsize=opsize)


def op00_52_inc(cpu, inst, opsize=2):
    """INC(B) (determined by opsize). Increment destination."""
    val, xb6 = cpu.operandx(inst & 0o77, opsize=opsize, rmw=True)
    newval = (val + 1) & cpu.MASK816[opsize]

    cpu.psw_n = True if newval & cpu.SIGN816[opsize] else False
    cpu.psw_z = (newval == 0)
    cpu.psw_v = (newval == cpu.SIGN816)
    # C bit not affected
    cpu.operandx(xb6, newval, opsize=opsize)


def op00_53_dec(cpu, inst, opsize=2):
    """DEC(B) (determined by opsize). Decrement destination."""
    val, xb6 = cpu.operandx(inst & 0o77, opsize=opsize, rmw=True)
    newval = (val - 1) & cpu.MASK816[opsize]

    cpu.psw_n = True if newval & cpu.SIGN816[opsize] else False
    cpu.psw_z = (newval == 0)
    cpu.psw_v = (val == cpu.SIGN816[opsize])
    # C bit not affected

    cpu.operandx(xb6, newval, opsize=opsize)


def op00_54_neg(cpu, inst, opsize=2):
    """NEG(B) (determined by opsize). Negate the destination."""

    val, xb6 = cpu.operandx(inst & 0o77, opsize=opsize, rmw=True)
    newval = (-val) & cpu.MASK816[opsize]

    cpu.psw_n = True if newval & cpu.SIGN816[opsize] else False
    cpu.psw_z = (newval == 0)
    cpu.psw_v = (val == newval)    # happens at the maximum negative value
    cpu.psw_c = (newval != 0)

    cpu.operandx(xb6, newval, opsize=opsize)


def op00_55_adc(cpu, inst, opsize=2):
    """ADC(B) (determined by opsize). Add carry."""
    # NOTE: "add carry" (not "add with carry")
    val, xb6 = cpu.operandx(inst & 0o77, opsize=opsize, rmw=True)
    if cpu.psw_c:
        oldsign = val & cpu.SIGN816[opsize]
        val = (val + 1) & cpu.MASK816[opsize]
        cpu.psw_v = (val & cpu.SIGN816[opsize]) and not oldsign
        cpu.psw_c = (val == 0)     # because this is the NEW (+1'd) val
    else:
        cpu.psw_v = 0
        cpu.psw_c = 0

    cpu.psw_n = True if val & cpu.SIGN816[opsize] else False
    cpu.psw_z = (val == 0)

    cpu.operandx(xb6, val, opsize=opsize)


def op00_56_sbc(cpu, inst, opsize=2):
    """SBC(B) (determined by opsize). Subtract carry."""
    # NOTE: "subtract carry" (not "subtract with carry/borrow")
    val, xb6 = cpu.operandx(inst & 0o77, opsize=opsize, rmw=True)
    if cpu.psw_c:
        oldsign = val & cpu.SIGN816[opsize]
        val = (val - 1) & cpu.MASK816[opsize]
        cpu.psw_v = oldsign and not (val & cpu.SIGN816[opsize])
        cpu.psw_c = (val == cpu.MASK816[opsize])  # bcs this is the (-1'd) val
    else:
        cpu.psw_v = 0
        cpu.psw_c = 0

    cpu.psw_n = True if val & cpu.SIGN816[opsize] else False
    cpu.psw_z = (val == 0)

    cpu.operandx(xb6, val, opsize=opsize)


def op00_57_tst(cpu, inst, opsize=2):
    """TST(B) (determined by opsize). Test destination."""
    dst = inst & 0o77
    val = cpu.operandx(dst, opsize=opsize)
    cpu.psw_n = True if val & cpu.SIGN816[opsize] else False
    cpu.psw_z = (val == 0)
    cpu.psw_v = 0
    cpu.psw_c = 0


def op00_60_ror(cpu, inst, opsize=2):
    """ROR(B) - rotate one bit right."""
    val, xb6 = cpu.operandx(inst & 0o77, opsize=opsize, rmw=True)
    signmask = cpu.SIGN816[opsize]
    vc = signmask if cpu.psw_c else 0
    cpu.psw_c, val = (val & 1), ((val >> 1) | vc) & cpu.MASK816[opsize]

    cpu.psw_n = True if val & signmask else False
    cpu.psw_z = (val == 0)
    cpu.psw_v = cpu.psw_n ^ cpu.psw_c

    cpu.operandx(xb6, val, opsize=opsize)


def op00_61_rol(cpu, inst, opsize=2):
    """ROL(B) - rotate one bit left."""
    val, xb6 = cpu.operandx(inst & 0o77, opsize=opsize, rmw=True)
    signmask = cpu.SIGN816[opsize]
    vc = 1 if cpu.psw_c else 0
    cpu.psw_c, val = True if (val & signmask) else False, ((val << 1) | vc) & cpu.MASK816[opsize]

    cpu.psw_n = True if val & signmask else False
    cpu.psw_z = (val == 0)
    cpu.psw_v = cpu.psw_n ^ cpu.psw_c

    cpu.operandx(xb6, val, opsize=opsize)


def op00_62_asr(cpu, inst, opsize=2):
    """ASR(B) - arithmetic shift right one bit."""
    val, xb6 = cpu.operandx(inst & 0o77, opsize=opsize, rmw=True)
    signbit = (val & cpu.SIGN816[opsize])
    cpu.psw_c = (val & 1)
    val >>= 1
    val |= signbit
    cpu.psw_n = True if val & cpu.SIGN816[opsize] else False
    cpu.psw_z = (val == 0)
    cpu.psw_v = cpu.psw_n ^ cpu.psw_c
    cpu.operandx(xb6, val, opsize=opsize)


def op00_63_asl(cpu, inst, opsize=2):
    """ASL(B) - arithmetic shift left one bit."""
    val, xb6 = cpu.operandx(inst & 0o77, opsize=opsize, rmw=True)
    cpu.psw_c = True if val & cpu.SIGN816[opsize] else False
    val = (val << 1) & cpu.MASK816[opsize]
    cpu.psw_n = True if (val & cpu.SIGN816[opsize]) else False
    cpu.psw_z = (val == 0)
    cpu.psw_v = cpu.psw_n ^ cpu.psw_c
    cpu.operandx(xb6, val, opsize=opsize)


def op00_64_mark(cpu, inst):
    # this instruction is... what it is. Note: if I/D separation
    # is enabled, the stack must be in BOTH D and I space, as control
    # will be transfered to the stack (typically) for this instruction.
    nn = inst & 0o77
    cpu.r[cpu.SP] = (cpu.r[cpu.PC] + (2 * nn)) & cpu.MASK16
    cpu.r[cpu.PC] = cpu.r[5]
    cpu.r[5] = cpu.stackpop()


def op00_65_mfpi(cpu, inst, opsize=2):
    """MFPI - move from previous instruction space.

    The "opsize" -- which really is just the top bit of the instruction,
    encodes whether this is mfpi or mfpd:
         opsize = 2 mfpi    (top bit was 0)
         opsize = 1 mfpd    (top bit was 1)
    """

    # There are some wonky special semantics. In user mode if prevmode
    # is USER (which it always is in Unix) then this refers to DSPACE
    # (despite the MFPI name) to protect the notion of "execute only" I space

    prvm = cpu.psw_prevmode
    curm = cpu.psw_curmode
    if prvm == cpu.USER and (curm == prvm):
        space = cpu.mmu.DSPACE
    else:
        space = (cpu.mmu.DSPACE, cpu.mmu.ISPACE)[opsize - 1]

    # MFPx SP is a special case, it means get the other SP register.
    if (inst & 0o77) == 6 and (prvm != curm):
        pival = cpu.stackpointers[prvm]
    else:
        pival = cpu.operandx(inst & 0o77, altmode=prvm, altspace=space)
    cpu.psw_n = pival & cpu.MASK16
    cpu.psw_z = (pival == 0)
    cpu.psw_v = 0
    cpu.stackpush(pival)


def op00_66_mtpi(cpu, inst, opsize=2):
    """MTPI - move to previous instruction space.

    The "opsize" encodes whether this is mtpi or mtpd:
         opsize = 2 mtpi
         opsize = 1 mtpd
    """

    # there are some wonky semantics ... this instruction is NOT restricted
    # to privileged modes and is potentially a path to writing into
    # a privileged space (!!). Unix (and probably all others) deals with
    # this by ensuring psw_prevmode is also USER when in USER mode.

    targetspace = (cpu.mmu.DSPACE, cpu.mmu.ISPACE)[opsize - 1]
    w = cpu.stackpop()

    cpu.psw_n = w & cpu.MASK16
    cpu.psw_z = (w == 0)
    cpu.psw_v = 0

    prvm = cpu.psw_prevmode
    curm = cpu.psw_curmode
    # note the special case that MTPx SP writes the other mode's SP register
    if (inst & 0o77) == 6 and (prvm != curm):
        cpu.stackpointers[prvm] = w
    else:
        cpu.operandx(inst & 0o077, w, altmode=prvm, altspace=targetspace)


def op00_67_sxt(cpu, inst):
    if cpu.psw_n:
        val = cpu.MASK16
    else:
        val = 0
    cpu.psw_z = not cpu.psw_n
    cpu.psw_v = 0
    cpu.operandx(inst & 0o0077, val)


ops56tab = (
    op00_50_clr,
    op00_51_com,
    op00_52_inc,
    op00_53_dec,
    op00_54_neg,
    op00_55_adc,
    op00_56_sbc,
    op00_57_tst,
    op00_60_ror,
    op00_61_rol,
    op00_62_asr,
    op00_63_asl,
    op00_64_mark,
    op00_65_mfpi,         # note: "byte" variant is really MFPD
    op00_66_mtpi,         # note: "byte" variant is really MTPD
    op00_67_sxt)


op00_dispatch_table = (
    op000_dispatcher,
    branches,
    branches,
    branches,
    op00_4_jsr,
    lambda cpu, inst: ops56tab[((inst & 0o7700) >> 6) - 0o50](cpu, inst),
    lambda cpu, inst: ops56tab[((inst & 0o7700) >> 6) - 0o50](cpu, inst),
    None)
