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

from enum import Enum
from interrupts import InterruptManager
from pdptraps import PDPTraps, PDPTrap


BusCycle = Enum('BusCycle', ('READ16', 'WRITE16', 'WRITE8', 'RESET'))
BusWrites = {BusCycle.WRITE16, BusCycle.WRITE8}      # convenience


class UNIBUS:

    def __init__(self, cpu):
        self.cpu = cpu
        self.intmgr = InterruptManager()
        self.logger = cpu.logger
        self.mmiomap = [self.__nodev] * (self.cpu.IOPAGE_SIZE >> 1)

    def resetbus(self):
        # this isn't especially efficient but it really doesn't matter.
        # Construct a sequence of tuples: (a, f)
        #    where a is the address (really offset) in the I/O page
        #    and f is the callback
        # but only for callbacks that are not __nodev
        # and then invoke those callbacks w/BusCycle.RESET
        resets = ((x << 1, f)
                  for x, f in enumerate(self.mmiomap) if f != self.__nodev)
        for ioaddr, f in resets:
            f(ioaddr, BusCycle.RESET)

    # register() -- connect a callback function to an I/O page address
    #
    # A convenient reference for addresses:
    #    https://gunkies.org/wiki/UNIBUS_Device_Addresses
    #
    # memory-mapped I/O is just a 4K array of function callbacks, one
    # entry per each I/O WORD (even) I/O offset. Most entries, of course,
    # remain set to an initial default "non-existent address" callback.
    #
    # Callbacks must be declared like this:
    #       def f(ioaddr, cycle, /, *, value=None):
    #
    # and should contain code dispatching on cycle and (potentially) ioaddr.
    # If cycle is WRITE8 (byte) or WRITE16 (word), the 'value' argument will
    # be supplied. Otherwise 'value' is not supplied.
    #
    # If cycle is READ16 (word), the read value should be returned. Note that
    # no READ8 exists as the UNIBUS is not capable of expressing a byte read.
    #
    # The processor RESET instruction generates calls with a RESET cycle.
    #
    # The ioaddr will always be relative to the base of the I/O page:
    #            (ioaddr & 8191) == ioaddr   will always be True
    #
    # If a device's semantics are simple enough to allow synthesizing a
    # WRITE8 as a READ16 / byte-update / WRITE16 sequence it can use
    # the autobyte wrapper and omit a WRITE8 cycle handler. If the underlying
    # address is write-only the autobyte wrapper substitutes a zero value
    # for the READ16 part of the synthesis. If this is not correct, the
    # callback must handle WRITE8 itself directly.
    #
    # Callback functions that need context or other data can, of course,
    # be bound methods (automatically receiving self) or also can use
    # functools.partial() to get additional arguments passed in.
    #
    #
    # ODD I/O ADDRESSES, BYTE OPERATIONS:
    #
    # The physical UNIBUS always *reads* full 16-bit words; there is no
    # capability for a byte read at the electrical bus level.
    #
    # Oddly (haha) the bus *can* specify byte access for writes.
    # Even odder (hahaha), devices that provide full-word access at
    # odd addresses exist, the best example being the CPU itself -- though
    # as it turns out only very few models allow **CPU** access to cpu
    # registers via the Unibus (vs allowing other devices, e.g., the console
    # switches/display, to access them that way).
    #
    # There are a lot of subtleties to take into account accessing I/O
    # addresses via bytes (and possibly-odd addresses) but so be it.
    # Such byte operations in I/O space abound in real world code.
    #
    # Devices that respond to a block of related addresses can register
    # one callback to cover more than one word of Unibus address space.
    # The callback gets the ioaddr which it can further decode. Again,
    # the CPU register block is a good example of this.
    #

    def register(self, iofunc, offsetaddr, nwords=1, /):
        """register a callback for an I/O address.

        Arguments:
           iofunc     -- callback function. See documentation for signature.
           offsetaddr -- Offset within the 8K I/O page.
           nwords     -- Optional. Default=1. Number of words to span.

        iofunc may be None in which case a dummy is supplied that ignores
        all write operations and always reads as zero.
        """

        if offsetaddr >= self.cpu.IOPAGE_SIZE:
            raise ValueError(f"UNIBUS: offset too large {oct(offsetaddr)}")

        # None is a shorthand for "this is a dummy always-zero addr"
        if iofunc is None:
            iofunc = self.autobyte(self.__ignoredev)

        idx, odd = divmod(offsetaddr, 2)
        if odd != 0:
            # See discussion of odd I/O addrs in block comment elsewhere
            raise ValueError("cannot register odd (byte) address in IO space")

        for i in range(nwords):
            self.mmiomap[idx+i] = iofunc

    # this is a convenience routine for devices that want to signal things
    # such as a write to a read-only register (if they don't simply just
    # ignore them). Devices can, of course, just raise the traps themselves.
    def illegal_cycle(self, addr, /, *, cycle=BusCycle.WRITE16, msg=None):
        if msg is None:
            msg =f"Illegal cycle ({cycle}) at {oct(addr)}"
        self.cpu.logger.info(msg)
        raise PDPTraps.AddressError(cpuerr=self.cpu.CPUERR_BITS.UNIBUS_TIMEOUT)

    # the default entry for unoccupied I/O: cause an AddressError trap
    def __nodev(self, addr, cycle, /, *, value=None):
        self.illegal_cycle(addr, cycle=cycle,
                           msg=f"Non-existent I/O @ offset {oct(addr)}")

    # Devices may have simple "dummy" I/O addresses that always read zero
    # and ignore writes; See "if iofunc is None" in register() method.
    # NOTE: register() byteme-wraps this so opsize not needed.
    def __ignoredev(self, addr, cycle, /, *, value=None):
        self.cpu.logger.debug(f"dummy zero device @ {oct(addr)}, {value=}")
        return 0

    # wrap an I/O function with code that automatically handles byte writes.
    # CAUTION: Some devices may have semantics that make this ill-advised.
    #          Such devices should handle WRITE8 explicitly themselves.
    # This is essentially a decorator but typically is invoked explicitly
    # rather than by '@' syntax.
    def autobyte(self, iofunc):
        def byteme(ioaddr, cycle, /, **kwargs):
            if cycle != BusCycle.WRITE8:
                return iofunc(ioaddr, cycle, **kwargs)

            # A byte write to I/O space. Synthesize it.
            # 'value' is present (by definition) in kwargs
            value = kwargs['value']
            self.cpu.logger.debug(f"Byte write to {oct(ioaddr)} {value=}")
            try:
                wv = iofunc(ioaddr & 0o177776, BusCycle.READ16)
            except PDPTrap:     # this can happen on write-only registers
                wv = 0
            if ioaddr & 1:
                wv = (wv & 0o377) | (value << 8)
            else:
                wv = (wv & 0o177400) | value
            iofunc(ioaddr & 0o177776, BusCycle.WRITE16, value=wv)
        return byteme

    # Convenience method -- registers simple attributes (or properties) into
    # I/O space in the obvious way: Make this attr (of obj) show at this addr
    #
    # BusCycle.RESET handling: If a device wants RESET to zero the attribute,
    # it should specify reset=True. Otherwise RESET will be ignored.
    #
    # BusCycle.WRITE8 handling: autobyte() is used. If those semantics are not
    # suitable, the device must create its own i/o func instead of this.
    #
    # readonly: If True (default is False) then writes to this address will
    #           cause an AddressError trap. The device implementation can
    #           safely (if it wants to) implement the attribute directly
    #           (i.e., without using @property) and no write to the attribute
    #           will ever originate from here.
    #
    def register_simpleattr(self, obj, attrname, addr, /, *,
                            reset=False, readonly=False):
        """Create and register a handler to read/write the named attr.

        obj - the object (often "self" for the caller of this method)
        attrname - the attribute name to read/write
        addr - the I/O address to register it to

        If attrname is None, the address is registered as a dummy location
        that ignores writes and will always read as zero. This is sometimes
        useful for features that have to exist but are emulated as no-op.

        reset - if True, attrname will be set to zero on BusCycle.RESET
        readonly - if True, attrname is readonly and will never be written.
                   Write cycles to the addr will cause AddressError.
        """

        # could do this with partial, but can also do it with this nested
        # func def. One way or another need this func logic anyway.

        if attrname is None:
            def _rwattr(_, cycle, /, *, value=None):
                if cycle == BusCycle.READ16:
                    return 0
                if cycle in BusWrites and readonly:
                    self.illegal_cycle(addr, cycle)
                # all other operations just silently ignored
        else:
            def _rwattr(_, cycle, /, *, value=None):
                if cycle == BusCycle.READ16:
                    return getattr(obj, attrname)
                elif cycle in BusWrites:
                    if readonly:
                        self.illegal_cycle(addr, cycle)
                    else:         # autobyte assumed ... see register() below
                        setattr(obj, attrname, value)
                elif cycle == BusCycle.RESET:
                    if reset:
                        setattr(obj, attrname, 0)
                else:
                    assert False, f"Unknown {cycle=} in simpleattr"

        # NOTES:
        #   * it's a new defn/closure of _rwattr each time through, so the
        #     individual (per registration) addr/etc values are closure'd
        #   * Do not autobyte if readonly, simply so that the correct
        #     (unmolested) BusCycle.WRITE8 will be seen in the trap/errors
        #   * _rwattr ASSUMES autobyte() wrapper if not readonly
        if readonly:
            self.register(_rwattr, addr)
        else:
            self.register(self.autobyte(_rwattr), addr)

    def wordRW(self, ioaddr, value=None, /):
        """Read (value is None) or write the given I/O address."""
        if value is None:
            value = self.mmiomap[ioaddr >> 1](ioaddr, BusCycle.READ16)
        else:
            self.mmiomap[ioaddr >> 1](ioaddr, BusCycle.WRITE16, value=value)
        return value

    def byteRW(self, ioaddr, value=None, /):
        """UNIBUS byte R/W - only write is legal."""
        if value is None:
            raise ValueError("Unibus cannot read bytes")
        else:
            self.mmiomap[ioaddr >> 1](ioaddr, BusCycle.WRITE8, value=value)
            return None


class UNIBUS_1170(UNIBUS):
    UBMAP_OFFS = 0o10200
    UBMAP_N = 62

    def __init__(self, cpu):
        super().__init__(cpu)

        # UBAs being 32-bit (well, really 22 bit) values, they
        # are just stored natively that way and broken down
        # into 16-bit components by the mmio function as needed.
        self.ubas = [0] * (self.UBMAP_N // 2)
        self.register(
            self.autobyte(self.uba_mmio), self.UBMAP_OFFS, self.UBMAP_N)

    def uba_mmio(self, addr, cycle, /, *, value=None):
        ubanum, hi22 = divmod(addr - self.UBMAP_OFFS, 4)
        uba22 = self.ubas[ubanum]

        self.logger.debug(f"UBA addr={oct(addr)}, {value=}")
        self.logger.debug(f"{ubanum=}, {hi22=}")

        if cycle == BusCycle.READ16:
            if hi22 == 0:
                return (uba22 >> 16) & 0o077
            else:
                return uba22 & 0o177777
        elif cycle == BusCycle.WRITE16:
            # the low bit is enforced to be zero
            if hi22:
                uba22 = ((value & 0o077) << 16) | (uba22 & 0o177776)
            else:
                uba22 = (uba22 & 0o17600000) | (value & 0o177776)
            self.ubas[ubanum] = uba22
        elif cycle == BusCycle.RESET:
            pass

    def busRW(self, ubaddr, value=None):
        pass
