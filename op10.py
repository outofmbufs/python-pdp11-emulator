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


from branches import branches
from op00 import ops56tab       # these have byte variants in op10
from pdptraps import PDPTraps


# dispatch MOST (but not all) of the 105x and 106x instructions
# to the op00 routines but with byte variation (opsize=1). But
# there are exceptions: 1064 unused, 1065 MFPD, 1066 MTPD, 1067 unused
def op156(cpu, inst):
    i56x = (inst & 0o7700) >> 6
    # 64 (mark) and 67 (sxt) do NOT have byte variants
    if i56x in (0o64, 0o67):
        raise PDPTraps.ReservedInstruction
    try:
        opf = ops56tab[i56x - 0o50]
    except IndexError:
        raise PDPTraps.ReservedInstruction
    opf(cpu, inst, opsize=1)


def op10_4_emttrap(cpu, inst):
    # bit 8 determines EMT (0) or TRAP(1)
    if (inst & 0o000400):
        raise PDPTraps.TRAP
    else:
        raise PDPTraps.EMT


op10_dispatch_table = (
    branches,
    branches,
    branches,
    branches,
    op10_4_emttrap,
    op156,
    op156,
    None)
