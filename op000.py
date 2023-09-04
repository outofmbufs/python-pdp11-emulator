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

# ... and even further down the op decode rabbit hole we go!
# These are the decodes for opcodes starting 0o000
from branches import branch


def op_halt(cpu, inst):
    if cpu.psw_curmode != cpu.KERNEL:
        # strange trap, but that's what it says
        raise PDPTraps.AddressError(cpu.CPUERR_BITS.ILLHALT)
    cpu.halted = True


def op_reset(cpu, inst):
    cpu.logger.debug("RESET INSTRUCTION")
    cpu.ub.resetbus()


def op_wait(cpu, inst):
    cpu.logger.debug("WAIT")
    cpu.ub.intmgr.waitstate(cpu.psw_pri)    # will pend until an interrupt


def op_rtt(cpu, inst):
    cpu.r[cpu.PC] = cpu.stackpop()
    cpu.psw = cpu.stackpop()


def op_02xx(cpu, inst):
    x5 = (inst & 0o70)
    if x5 == 0o00:
        op_rts(cpu, inst)
    elif x5 == 0o30:
        op_spl(cpu, inst)
    elif x5 >= 0o40:
        op_xcc(cpu, inst)
    else:
        raise PDPTraps.ReservedInstruction


def op_spl(cpu, inst):
    """SPL; note that this is a no-op (!) not a trap in non-kernel mode."""
    if cpu.psw_curmode == cpu.KERNEL:
        cpu.psw = (cpu.psw & ~ (0o07 << 5)) | ((inst & 0o07) << 5)


def op_rts(cpu, inst):
    Rn = (inst & 0o07)
    cpu.r[cpu.PC] = cpu.r[Rn]        # will be a no-op for RTS PC
    cpu.r[Rn] = cpu.stackpop()


def op_jmp(cpu, inst):
    # same justEA/operand non-indirection discussion as in JSR (see)
    cpu.r[cpu.PC] = cpu.operandx(inst & 0o77, justEA=True)


def op_swab(cpu, inst):
    """SWAB swap bytes."""
    val, xb6 = cpu.operandx(inst & 0o77, rmw=True)

    val = ((val >> 8) & cpu.MASK8) | ((val & cpu.MASK8) << 8)
    cpu.psw_n = val & cpu.SIGN16

    # note this screwy definition, per the handbook
    cpu.psw_z = ((val & cpu.MASK8) == 0)
    cpu.psw_v = 0
    cpu.psw_c = 0

    cpu.operandx(xb6, val)


def op_xcc(cpu, inst):
    """XCC - all variations of set/clear condition codes."""

    setclr = inst & 0o020                 # set it or clear it
    if inst & 0o10:
        cpu.psw_n = setclr
    if inst & 0o04:
        cpu.psw_z = setclr
    if inst & 0o02:
        cpu.psw_v = setclr
    if inst & 0o01:
        cpu.psw_c = setclr


def op000_dispatcher(cpu, inst):
    match (inst & 0o0700):
        case 0o0000:
            if inst == 0:
                op_halt(cpu, inst)
            elif inst == 6 or inst == 2:    # RTI and RTT are identical!!
                op_rtt(cpu, inst)
            elif inst == 1:
                op_wait(cpu, inst)
            elif inst == 5:
                op_reset(cpu, inst)

        case 0o0100:
            op_jmp(cpu, inst)

        case 0o0200:
            op_02xx(cpu, inst)

        case 0o0300:
            op_swab(cpu, inst)

        # note that 2 bits of the branch offset sneak into this match
        case 0o0400 | 0o0500 | 0o0600 | 0o0700:
            branch(cpu, inst, lambda n, z, v, c: True)
