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

# simulation of a KL-11 console interface
#
# Trivial TCP server that accepts connections on port 1170 (cute)
# and simply proxies character traffic back and forth.
#
import socket
import threading
import queue

from pdptraps import PDPTraps


class KL11:

    KL11_DEFAULT = 0o17560           # offset within I/O page
    BUSY = 0o400                     # probably no reason to ever set this
    RCDONE = 0o200                   # means character available in buf
    TXRDY = 0o200                    # same bit on tx side is called "RDY"
    IENABLE = 0o100
    RDRENA = 0o001

    _SHUTDOWN_SENTINEL = object()
    SERVERHOST = ''
    SERVERPORT = 1170

    def __init__(self, ub, baseaddr=KL11_DEFAULT):
        self.ub = ub
        self.addr = ub.mmio.register(self.klregs, baseaddr, 4)
        ub.mmio.devicereset_register(self.reset)

        # output characters are just queued (via tq) to the output thread
        # input characters have to undergo a more careful 1-by-1
        # dance to properly match interrupts to their arrival
        self.tq = queue.Queue()
        self.rxc = threading.Condition()

        # bits broken out of virtualized KL11 Reader Status Register
        self.rcdone = False
        self.r_ienable = False

        # reader buffer register (address: baseaddr + 2)
        self.rdrbuf = 0

        # transmit buffer status (address: baseaddr + 4)
        self.t_ienable = False

        # The socket server connection/listener
        self._t = threading.Thread(target=self._connectionserver, daemon=True)
        self._t.start()

    def reset(self, ub):
        """Called for UNIBUS resets (RESET instruction)."""
        self.rcdone = False
        self.r_ienable = False
        self.r_tenable = False

    def klregs(self, addr, value=None, /):
        match addr - self.addr:
            case 0:              # rcsr
                if value is None:
                    # *** READING ***

                    value = 0

                    if self.r_ienable:
                        value |= self.IENABLE

                    if self.rcdone:
                        value |= self.RCDONE

                else:
                    # *** WRITING ***
                    if value & self.RDRENA:
                        with self.rxc:
                            # a request to get one character, which only
                            # has to clear the rcdone bit here.
                            self.rcdone = False
                            self.rdrbuf = 0
                            self.r_ienable = (value & self.IENABLE)
                            self.rxc.notify()

            case 2 if value is None:         # rbuf
                # *** READING ***
                with self.rxc:
                    value = self.rdrbuf
                    self.rcdone = False
                    self.rxc.notify()

            # transmit buffer status (sometimes called tcsr)
            case 4:
                if value is None:
                    # *** READING ***
                    value = self.TXRDY      # always ready to send chars
                    if self.t_ienable:
                        value |= self.IENABLE
                else:
                    # *** WRITING ***
                    prev = self.t_ienable
                    self.t_ienable = (value & self.IENABLE)
                    if self.t_ienable and not prev:
                        self.ub.intmgr.simple_irq(pri=4, vector=0o64)

            # transmit buffer
            case 6 if value is not None:              # tbuf
                # *** WRITING ***
                value &= 0o177
                if (value != 0o177):
                    s = chr(value)
                    self.tq.put(s)
                if self.t_ienable:
                    self.ub.intmgr.simple_irq(pri=4, vector=0o64)
            case _:
                raise PDPTraps.AddressError

        return value

    def _connectionserver(self):
        """Server loop daemon thread for console I/O."""
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serversocket.bind((self.SERVERHOST, self.SERVERPORT))
        serversocket.listen(1)

        def _outloop(q, s):
            while True:
                try:
                    c = q.get(True, timeout=2)
                    if c is self._SHUTDOWN_SENTINEL:
                        break
                except queue.Empty:
                    pass
                else:
                    s.sendall(c.encode())

        def _inloop(s):
            while len(b := s.recv(1)) != 0:
                with self.rxc:
                    self.rxc.wait_for(lambda: not self.rcdone)
                    self.rdrbuf = ord(b)
                    self.rcdone = True
                    if self.r_ienable:
                        self.ub.intmgr.simple_irq(pri=4, vector=0o60)

        while True:
            s, addr = serversocket.accept()

            outthread = threading.Thread(target=_outloop, args=(self.tq, s))
            inthread = threading.Thread(target=_inloop, args=(s,))

            outthread.start()
            inthread.start()

            inthread.join()
            self.tq.put(self._SHUTDOWN_SENTINEL)
            outthread.join()

    # debugging tool
    def statestring(self):
        s = ""
        if self.r_ienable:
            s += " RINT"
        if self.t_ienable:
            s += " TINT"
        if self.rdrbuf:
            s += f" LC={self.rdrbuf}"
        if self.rcdone:
            s += f" RCDONE"
        return s
