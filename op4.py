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

#
# TOP LEVEL OP CODE DISPATCH AND INSTRUCTION IMPLEMENTATION
#
# NOTES:
#
#  DISPATCH
#     2-operand instructions are implemented here and are dispatched
#     off the top-4 bits of the instruction (hence "op4" name).
#
#     The other instructions are encoded into the 0o00, 0o07, and 0o10
#     portions of this top-4 bit address space. They are dispatched
#     via d3dispatch and respective tables from other modules.
#
#  BYTE vs WORD operations:
#     Most of them come in two flavors - word and byte, with the
#     top-bit distinguishing. This is communicated to the implementation
#     functions via "opsize=1" or "opsize=2" when a single function can
#     implement both variations. Note that MOV/MOVB are specifically
#     optimized, separately, for performance.
#
#  PSW updates:
#     All instructions must be careful to do their final result writes
#     AFTER setting the PSW, because the PSW is addressible via memory
#     (a write to unibus 777776) and such a write is supposed to override
#     the otherwise-native instruction CC results.
#

# dispatchers to next level for 00, 07, and 10 instructions:
from op00 import op00_dispatch_table
from op07 import op07_dispatch_table
from op10 import op10_dispatch_table
from pdptraps import PDPTraps


def d3dispatcher(d3table, cpu, inst):
    opf = d3table[(inst & 0o7000) >> 9]
    if opf is None:
        raise PDPTraps.ReservedInstruction
    opf(cpu, inst)


# This is ALWAYS a 16-bit MOV
def op01_mov(cpu, inst):
    """MOV src,dst -- always 16 bits"""

    # avoid call to the more-general operandx for mode 0, direct register.
    # This optimization is a substantial speed up for register MOVs.
    if (inst & 0o7000) == 0:
        val = cpu.r[(inst & 0o700) >> 6]
    else:
        val = cpu.operandx((inst & 0o7700) >> 6)

    cpu.psw_v = 0              # per manual; V is cleared
    cpu.psw_z = (val == 0)
    cpu.psw_n = (val > 32767)

    # same optimization on the write side.
    if (inst & 0o70) == 0:
        cpu.r[(inst & 0o07)] = val
    else:
        cpu.operandx(inst & 0o0077, val)


# This is ALWAYS an 8-bit MOVB
def op11_movb(cpu, inst):
    """MOVB src,dst -- always 8 bits"""

    # avoid call to the more-general operandx for mode 0, direct register.
    # This optimization is a substantial speed up for register MOVs.
    if (inst & 0o7000) == 0:
        val = cpu.r[(inst & 0o700) >> 6] & 0o377
    else:
        val = cpu.operandx((inst & 0o7700) >> 6, opsize=1)

    cpu.psw_v = 0
    cpu.psw_z = (val == 0)
    cpu.psw_n = True if val & 0o200 else False

    # avoid call to the more-general operandx for mode 0, direct register
    # not only as an optimization, but because unlike other byte operations
    # in register-direct mode, MOVB does a sign-extend.
    dst = inst & 0o0077
    if (dst < 8):             # i.e., mode 0
        if val > 127:
            val |= 0o177400
        cpu.r[dst] = val
    else:
        cpu.operandx(dst, val, opsize=1)


def op02_cmp(cpu, inst, opsize=2):
    """CMP(B) src,dst"""
    src = cpu.operandx((inst & 0o7700) >> 6, opsize=opsize)
    dst = cpu.operandx(inst & 0o0077, opsize=opsize)

    # note: this is other order than SUB
    t = (src - dst) & cpu.MASK816[opsize]
    cpu.psw_c = (src < dst)
    signbit = cpu.SIGN816[opsize]
    cpu.psw_n = True if t & signbit else False
    cpu.psw_z = (t == 0)

    # definition of V is: operands were of opposite signs and the sign
    # of the destination was the same as the sign of the result
    src_sign = src & signbit
    dst_sign = dst & signbit
    t_sign = t & signbit
    cpu.psw_v = (dst_sign == t_sign) and (src_sign != dst_sign)


def op03_bit(cpu, inst, opsize=2):
    """BIT(B) src,dst"""
    src = cpu.operandx((inst & 0o7700) >> 6, opsize=opsize)
    dst = cpu.operandx(inst & 0o0077, opsize=opsize)
    t = dst & src

    cpu.psw_n = True if t & cpu.SIGN816[opsize] else False
    cpu.psw_z = (t == 0)
    cpu.psw_v = 0
    # cpu.logger.debug(f"BIT: {src=}, {dst=}, PSW={oct(cpu.psw)}")


def op04_bic(cpu, inst, opsize=2):
    """BIC(B) src,dst"""
    src = cpu.operandx((inst & 0o7700) >> 6, opsize=opsize)
    dst, xb6 = cpu.operandx(inst & 0o0077, opsize=opsize, rmw=True)
    dst &= ~src

    cpu.psw_n = True if dst & cpu.SIGN816[opsize] else False
    cpu.psw_z = (dst == 0)
    cpu.psw_v = 0

    cpu.operandx(xb6, dst, opsize=opsize)


def op05_bis(cpu, inst, opsize=2):
    """BIS(B) src,dst"""
    src = cpu.operandx((inst & 0o7700) >> 6, opsize=opsize)
    dst, xb6 = cpu.operandx(inst & 0o0077, opsize=opsize, rmw=True)
    dst |= src

    cpu.psw_n = True if dst & cpu.SIGN816[opsize] else False
    cpu.psw_z = (dst == 0)
    cpu.psw_v = 0

    cpu.operandx(xb6, dst, opsize=opsize)


def op06_add(cpu, inst):
    """ADD src,dst"""
    src = cpu.operandx((inst & 0o7700) >> 6)
    dst, xb6 = cpu.operandx(inst & 0o0077, rmw=True)
    t = src + dst

    cpu.psw_c = (t > cpu.MASK16)
    if cpu.psw_c:
        t &= cpu.MASK16

    cpu.psw_n = True if t & cpu.SIGN16 else False
    cpu.psw_z = (t == 0)

    # definition of V is: operands were of the same signs and the
    # sign of the result is different.
    src_sign = src & cpu.SIGN16
    dst_sign = dst & cpu.SIGN16
    t_sign = t & cpu.SIGN16
    cpu.psw_v = (dst_sign != t_sign) and (src_sign == dst_sign)

    cpu.operandx(xb6, t)


def op16_sub(cpu, inst):
    """SUB src,dst"""
    src = cpu.operandx((inst & 0o7700) >> 6)
    dst, xb6 = cpu.operandx(inst & 0o0077, rmw=True)

    t = (dst - src) & cpu.MASK16     # note: this is opposite of CMP
    cpu.psw_n = True if t & cpu.SIGN16 else False
    cpu.psw_z = (t == 0)

    # definition of V is: operands were of opposite signs and the sign
    # of the source was the same as the sign of the result
    src_sign = src & cpu.SIGN16
    dst_sign = dst & cpu.SIGN16
    t_sign = t & cpu.SIGN16
    cpu.psw_v = (src_sign == t_sign) and (src_sign != dst_sign)
    cpu.psw_c = (src > dst)

    cpu.operandx(xb6, t)


def op17_reserved(cpu, inst):
    raise PDPTraps.ReservedInstruction


op4_dispatch_table = (
    lambda c, i: d3dispatcher(op00_dispatch_table, c, i),
    op01_mov,
    op02_cmp,
    op03_bit,
    op04_bic,
    op05_bis,
    op06_add,
    lambda c, i: d3dispatcher(op07_dispatch_table, c, i),
    lambda c, i: d3dispatcher(op10_dispatch_table, c, i),
    op11_movb,                               # NOTE: optimized; not mov+lambda
    lambda c, i: op02_cmp(c, i, opsize=1),   # 12
    lambda c, i: op03_bit(c, i, opsize=1),   # 13
    lambda c, i: op04_bic(c, i, opsize=1),   # 14
    lambda c, i: op05_bis(c, i, opsize=1),   # 15
    op16_sub,
    op17_reserved)                           # 17 reserved
