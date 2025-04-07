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

# op07 instructions


# 6 simple instructions appear in the op07 space: 070 .. 074 and 077.
# In these the first operand is restricted to being only a register
# (because the three bits usually used for 'mode' are part of
# these opcodes). The destination is still a full 6-bit specification.
#
# op075 is used for floating point instruction encoding (FP instructions
#       are also found in other nooks and crannies)
#
# op076 is the commercial instruction set
#

from pdptraps import PDPTraps


def op070_mul(cpu, inst):
    dstreg = (inst & 0o000700) >> 6
    r = cpu.r[dstreg]
    src = cpu.operandx(inst & 0o077)

    # unlike add/subtract, need to explicitly treat as signed.
    # The right results require sign extending both 16 bit operands to
    # 32 bits, multiplying them, then taking the bottom 32 bits of the result.
    # It may not be obvious why this works; see google.
    if r >= 32768:
        r |= 0xFFFF0000
    if src >= 32768:
        src |= 0xFFFF0000
    m = (src * r) & 0xFFFFFFFF

    # the result is stored:
    #    high 16 bits in dstreg
    #    low 16 bits in dstreg|1
    # and if dstreg is odd ONLY the low 16 bits are stored
    # This just stores both but in careful order
    cpu.r[dstreg] = (m >> 16) & 0xFFFF
    cpu.r[dstreg | 1] = m & 0xFFFF

    cpu.psw_n = m & 0x80000000
    if cpu.psw_n:
        cpu.psw_c = (m < 0xFFFF8000)
    else:
        cpu.psw_c = (m >= 32768)
    cpu.psw_z = (m == 0)
    cpu.psw_v = 0


def op071_div(cpu, inst):
    dstreg = (inst & 0o000700) >> 6
    if (dstreg & 1):
        raise PDPTraps.ReservedInstruction     # dstreg must be even
    dividend = (cpu.r[dstreg] << 16) | cpu.r[dstreg | 1]
    divisor = cpu.operandx(inst & 0o077)
    if divisor == 0:
        cpu.psw_n = 0
        cpu.psw_z = 1
        cpu.psw_v = 1
        cpu.psw_c = 1
    elif divisor == 0o177777 and dividend == 0x80000000:
        # maxneg / -1 == too big
        cpu.psw_n = 0
        cpu.psw_z = 0
        cpu.psw_v = 1
        cpu.psw_c = 0
    else:
        # convert both numbers to positive equivalents
        # and track sign info manually
        if dividend & cpu.SIGN32:
            dividend = 4*1024*1024*1024 - dividend
            ddendposneg = -1
        else:
            ddendposneg = 1
        posneg = ddendposneg
        if divisor & cpu.SIGN16:
            divisor = 65536 - divisor
            posneg = -posneg
        q, rem = divmod(dividend, divisor)
        q *= posneg
        if q > 32767 or q < -32768:
            cpu.psw_n = (q < 0)
            cpu.psw_z = 0
            cpu.psw_v = 1
            cpu.psw_c = 0
        else:
            if ddendposneg < 0:
                rem = -rem
            cpu.psw_n = (q < 0)
            cpu.psw_z = (q == 0)
            cpu.psw_v = 0
            cpu.psw_c = 0

            cpu.r[dstreg] = q & cpu.MASK16
            cpu.r[dstreg | 1] = rem & cpu.MASK16


def op072_ash(cpu, inst):
    dstreg = (inst & 0o000700) >> 6
    r = cpu.r[dstreg]
    shift = cpu.operandx(inst & 0o077) & 0o077

    r = _shifter(cpu, r, shift, opsize=2)

    cpu.r[dstreg] = r


def op073_ashc(cpu, inst):
    dstreg = (inst & 0o000700) >> 6
    r = (cpu.r[dstreg] << 16) | cpu.r[dstreg | 1]
    shift = cpu.operandx(inst & 0o077) & 0o077

    r = _shifter(cpu, r, shift, opsize=4)

    cpu.r[dstreg] = (r >> 16) & cpu.MASK16
    cpu.r[dstreg | 1] = r & cpu.MASK16


# this is the heart of ash and ashc.
def _shifter(cpu, value, shift, *, opsize):
    """Returns shifted value and sets condition codes."""

    # Implementation relies on unlimited size python ints (at least 64-bits)

    if shift < 0 or shift > 63 or (opsize not in (2, 4)):
        raise ValueError(f"Internal ASH/ASHC error, {shift=} {opsize=}")

    signmask = cpu.SIGN16
    if opsize == 4:
        signmask = cpu.SIGN16 << 16

    original_signbit = bool(value & signmask)

    if shift == 0:
        overflow = 0
        cbit = 0                   # per 1981 PDP11 Processor Handbook
    elif shift > 31:
        # RIGHT SHIFT

        if original_signbit:
            # add 1 bits up top to preserve negativity for up to 32 bit shift
            value |= 0xFFFFFFFF0000 if opsize == 2 else 0xFFFFFFFF00000000

        # right shift by 1 less, to capture bottom bit for C
        value >>= (63 - shift)  # yes 63, see ^^^^^^^^^^^^^^^
        cbit = (value & 1)
        value >>= 1
        overflow = False
    else:
        # LEFT SHIFT

        # these are the upper bits (beyond the 16 or 32 bit result)
        # that must stay the same as the original sign bit or there was
        # an overflow. Note the signbit is included in this mask
        upperbits = (((1 << shift) - 1) << (opsize*8)) | signmask

        # shift by 1 less to capture cbit
        value <<= (shift - 1)
        cbit = value & signmask
        value <<= 1

        if original_signbit:
            overflow = ((value & upperbits) != upperbits)
        else:
            overflow = (value & upperbits)

    value &= (signmask | (signmask - 1))
    cpu.psw_n = (value & signmask)
    cpu.psw_z = (value == 0)
    cpu.psw_v = overflow
    cpu.psw_c = cbit

    return value


def op074_xor(cpu, inst):
    srcreg = (inst & 0o000700) >> 6
    r = cpu.r[srcreg]

    s, xb6 = cpu.operandx(inst & 0o077, rmw=True)

    r ^= s

    cpu.psw_n = (r & cpu.SIGN16)
    cpu.psw_z = (r == 0)
    cpu.psw_v = 0

    cpu.operandx(xb6, r)


def op077_sob(cpu, inst):
    srcreg = (inst & 0o000700) >> 6
    r = (cpu.r[srcreg] - 1) & 0xffff

    if r != 0:
        # technically if this instruction occurs low enough in memory
        # this PC subtraction could wrap, so be technically correct & mask
        cpu.r[cpu.PC] = (cpu.r[cpu.PC] - 2 * (inst & 0o077)) & cpu.MASK16

    cpu.r[srcreg] = r


# dispatch on the next digit after the 07 part...
op07_dispatch_table = (
    op070_mul,
    op071_div,
    op072_ash,
    op073_ashc,
    op074_xor,
    None,        # various float instructions, not implemented
    None,        # CIS instructions, not implmented
    op077_sob)
