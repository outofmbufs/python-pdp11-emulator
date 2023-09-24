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

# These tables are helpful to have for assembler, disassembler, etc

BRANCH_CODES = {
    'br': 0o000400,
    'bne': 0o001000,
    'beq': 0o001400,
    'bpl': 0o100000,
    'bmi': 0o100400,
    'bvc': 0o102000,
    'bvs': 0o102400,
    'bcc': 0o103000,
    'bhis': 0o103000,
    'bcs': 0o103400,
    'blo': 0o103400,
    'bge': 0o002000,
    'blt': 0o002400,
    'bgt': 0o003000,
    'ble': 0o003400,
    'bhi': 0o101000,
    'blos': 0o101400
}

# There are some duplicates and they are chosen arbitrarily by this:
BRANCH_NAMES = {v: k for k, v in BRANCH_CODES.items()}


# keyed by masked "base code" (upper byte), not shifted
_brconds = {
    # NOTE: 000400 case is handled in op000 dispatch separately
    # 0o000400: lambda n, z, v, c: True,             # BR
    0o001000: lambda n, z, v, c: not z,            # BNE
    0o001400: lambda n, z, v, c: z,                # BEQ
    0o100000: lambda n, z, v, c: not n,            # BPL
    0o100400: lambda n, z, v, c: n,                # BMI
    0o102000: lambda n, z, v, c: not v,            # BVC
    0o102400: lambda n, z, v, c: v,                # BVS
    0o103000: lambda n, z, v, c: not c,            # BCC
    0o103400: lambda n, z, v, c: c,                # BCS

    # CAUTION: Python XOR ("^") is bitwise; hence bool() != for ^
    0o002000: lambda n, z, v, c: bool(n) == bool(v),              # BGE
    0o002400: lambda n, z, v, c: bool(n) != bool(v),              # BLT
    0o003000: lambda n, z, v, c: (bool(n) == bool(v)) and not z,  # BGT
    0o003400: lambda n, z, v, c: (bool(n) != bool(v)) or z,       # BLE


    0o101000: lambda n, z, v, c: not (c or z),     # BHI
    0o101400: lambda n, z, v, c: c or z,           # BLOS

    # NOTE: These two are the same as BCC/BCS respectively
    #    0o103000: lambda n, z, v, c: not c,            # BHIS
    #    0o103400: lambda n, z, v, c: c,                # BLO
}


def branches(cpu, inst):
    branch(cpu, inst, _brconds[inst & 0o177400])


def branch(cpu, inst, condition):
    if condition(cpu.psw_n, cpu.psw_z, cpu.psw_v, cpu.psw_c):
        offset = (inst & 0o377) * 2
        if offset >= 256:      # i.e., was a negative 8-bit value
            offset -= 512
        cpu.r[7] = cpu.u16add(cpu.r[7], offset)
