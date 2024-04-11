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

# Emulate a DC-11 multi-device serial adapter.


# The "emulation", such as it is right now, is entirely dummy.
# This is needed just so Unix doesn't crash trying to establish a getty
# on /dev/tty0 .. /dev/tty3 (though, alternatively, the entries in /etc/ttys
# could just have been disabled; this is more robust though).
#
# Essentially it is emulating serial cables that are never plugged in. :)
#
# Without this trivial emulation, the default configuration of unix v7 will
# crash going into multiuser mode as it expects the device to be there (i.e.
# it expects the I/O addresses to respond and crashes if they bus-fault)
#

class DC11:
    DC11_DEFAULT = 0o14000           # offset within I/O page
    DC11_NDEVS = 4                   # four devices, each is 4 16-bit registers

    def __init__(self, ub, baseaddr=DC11_DEFAULT):
        self.addr = self.DC11_DEFAULT
        self.ub = ub
        self.ub.register(None, self.DC11_DEFAULT, self.DC11_NDEVS * 4)
