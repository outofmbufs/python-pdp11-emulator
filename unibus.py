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

from interrupts import InterruptManager
from mmio import MMIO

# A convenient reference for addresses:
#    https://gunkies.org/wiki/UNIBUS_Device_Addresses


class UNIBUS:
    def __init__(self, cpu):
        self.cpu = cpu
        self.mmio = MMIO(cpu)
        self.intmgr = InterruptManager()

    def resetbus(self):
        self.mmio.resetdevices()


class UNIBUS_1170(UNIBUS):
    UBMAP_OFFS = 0o10200
    UBMAP_N = 62

    def __init__(self, cpu):
        super().__init__(cpu)

        # UBAs being 32-bit (well, really 22 bit) values, they
        # are just stored natively that way and broken down
        # into 16-bit components by the mmio function as needed.
        self.ubas = [0] * (self.UBMAP_N // 2)
        self.mmio.register(self.uba_mmio, self.UBMAP_OFFS, self.UBMAP_N)

    def uba_mmio(self, addr, value=None, /):
        ubanum, hi22 = divmod(addr - self.UBMAP_OFFS, 4)
        uba22 = self.ubas[ubanum]

        self.cpu.logger.debug(f"UBA addr={oct(addr)}, {value=}")
        self.cpu.logger.debug(f"{ubanum=}, {hi22=}")

        if value is None:
            if hi22 == 0:
                return (uba22 >> 16) & 0o077
            else:
                return uba22 & 0o177777
        else:
            # the low bit is enforced to be zero
            if hi22:
                uba22 = ((value & 0o077) << 16) | (uba22 & 0o177776)
            else:
                uba22 = (uba22 & 0o17600000) | (value & 0o177776)
            self.ubas[ubanum] = uba22

    def busRW(self, ubaddr, value=None):
        pass
