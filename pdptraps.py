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

# exceptions representing processor traps

from types import SimpleNamespace


class PDPTrap(Exception):
    vector = -1

    def __init__(self, cpuerr=0, **kwargs):

        # if specified, the cpuerr bit(s) will be OR'd into
        # the CPU Error Register when the trap is processed by go_trap
        self.cpuerr = cpuerr

        # any additional arguments that are specific per-trap info
        # simply get stored as-is
        self.trapinfo = kwargs

    def __str__(self):
        s = self.__class__.__name__ + "("
        s += f"vector={oct(self.vector)}"
        if self.cpuerr:
            s += f", cpuerr={oct(self.cpuerr)}"
        if self.trapinfo:
            s += f", {self.trapinfo=}"
        s += ")"
        return s


# rather than copy/pasta the above class, they are made this way
# It's not clear this is much better
# XXX the for/setattr loop instead of a dict() in SimpleNamespace
#     only because it seems to be more readable this way

PDPTraps = SimpleNamespace()
for __nm, __v in (
        ('AddressError', 0o004),
        ('ReservedInstruction', 0o010),
        ('BPT', 0o014),
        ('IOT', 0o20),
        ('PowerFail', 0o24),
        ('EMT', 0o30),
        ('TRAP', 0o34),
        ('Parity', 0o114),
        ('PIRQ', 0o240),
        ('FloatingPoint', 0o244),
        ('MMU', 0o250)):
    setattr(PDPTraps, __nm, type(__nm, (PDPTrap,), dict(vector=__v)))
