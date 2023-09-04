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


class KW11:

    KW11_OFFS = 0o17546

    def __init__(self, ub):
        interrupt_manager = ub.intmgr
        self._t = threading.Thread(
            target=self._cloop, args=(0.05, interrupt_manager), daemon=True)
        self.running = False
        self.monbit = 0
        ub.mmio.register_simpleattr(self, 'LKS', self.KW11_OFFS, reset=True)

    # clock loop
    def _cloop(self, interval, imgr):
        while self.running:
            time.sleep(interval)
            # there are inherent races here (in the hardware too) but
            # seek to make the hazard smaller than the full interval
            # by testing self.running again here.
            if self.running:
                imgr.simple_irq(pri=6, vector=0o100)

    @property
    def LKS(self):
        return (int(self.monbit) << 7) | (int(self.running) << 6)

    @LKS.setter
    def LKS(self, value):
        if not self.running:
            if value & 0o100:
                self.running = True
                self._t.start()

        self.monbit = (value & 0o200)
        if self.running and not (value & 0o100):
            # this never happens in unix but ...
            self.running = False
            self._t.join()
