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

# line frequency clock

import time
import threading


# The real clock runs at 50 or 60 cycles per second. However, the
# overhead of emulated interrupts is high - as a compromise set HZ
# to something slower than 50-60...
HZ = 25


class KW11:

    KW11_OFFS = 0o17546

    def __init__(self, ub):
        self._t = threading.Thread(
            target=self._cloop, args=(1/HZ, ub.intmgr), daemon=True)
        self.interrupts_enabled = False
        self.monbit = 1       # the manual says this starts as 1
        ub.register_simpleattr(self, 'LKS', self.KW11_OFFS, reset=True)
        self._t.start()

    # clock loop
    def _cloop(self, interval, imgr):
        while True:
            time.sleep(interval)
            self.monbit = 1
            # The loop runs forever (as does the real device) but only
            # generates interrupts if interrupts are enabled
            if self.interrupts_enabled:
                imgr.simple_irq(pri=6, vector=0o100)

    @property
    def LKS(self):
        return (int(self.monbit) << 7) | (int(self.interrupts_enabled) << 6)

    @LKS.setter
    def LKS(self, value):
        self.interrupts_enabled = bool(value & 0o100)
        self.monbit = bool(value & 0o200)
