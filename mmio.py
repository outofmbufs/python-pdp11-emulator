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

from pdptraps import PDPTraps


class MMIO:
    """Memory Mapped I/O handling for the 8K byte I/O page."""

    #
    # memory-mapped I/O is just a 4K array of function callbacks, one
    # entry per each I/O WORD (even) I/O offset. Most entries, of course,
    # remain set to an initial default "non-existent address" callback.
    #
    # See register() for setting a callback on a specific offset or range.
    #
    # Reads and writes to any offset within the I/O page invoke the
    # callback function.  Assuming f is the callback function:
    #
    #    READS:    v = f(ioaddr)
    #              The function f must return a value 0 .. 65535
    #
    #   WRITES:    f(ioaddr, v)
    #              v will be an integer 0 .. 65535
    #              The return value is ignored.
    #
    # Callbacks must be declared like this:
    #       def f(ioaddr, value=None, /):
    #           if value is None:
    #               ... this is the read case
    #           else:
    #               ... this is the write case
    #
    # Zero, of course, is a possible and common value to write, so do not make
    # the rookie mistake of testing value for truthiness; test for *is* None.
    #
    # The ioaddr will always be relative to the base of the I/O page:
    #            (ioaddr & 8191) == ioaddr   will always be True
    #
    # Callbacks may also optionally receive a byte operation indicator,
    # but only if they register for that. See BYTE OPERATIONS, below.
    # Most callbacks will instead rely on the framework to synthesize
    # byte operations from words; see _byte_wrapper and byteme.
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
    # odd addresses exist, the best example being the CPU itself. In some
    # models the registers are available at a block of UNIBUS addresses,
    # and some of the 16-bit registers have ODD addresses.
    # For example, UNIBUS address 777707 is the PDP-11 cpu PC, as a
    # full 16-bit word, while 777706 is the PDP-11 cpu KSP.
    #
    # This creates potential for havoc if programmers use byte operations
    # on I/O addresses. Consider a typical sequence to read consecutive
    # bytes from an address in R0:
    #
    #      MOVB (R0)+,R1        # get the low byte of the word R0 points to
    #      MOVB (R0)+,R2        # get the high byte of that word
    #
    # If executed with R0 = 177706 (the KSP register virtual address
    # with a typical 16-bit mapping of the upper page to I/O space)
    # then R1 will be the low byte of KSP but the next access, which will
    # be seen by the UNIBUS as a word read on an odd address, will pull
    # the PC value (at 777707) as a word and extract the upper byte
    # from THAT (PC upper byte in R2; KSP lower byte in R1). This is
    # probably an unexpected result, which is a good argument against
    # using byte operations in I/O space. Nevertheless, byte operations
    # in I/O space abound in real world code.
    #
    # Thus:
    #  *  DEVICES THAT DIRECTLY SUPPORT BYTE-WRITE OPERATIONS:
    #     Specify "byte_writes=True" in the registration call:
    #
    #        mmio.register(somefunc, someoffset, somesize, byte_writes=True)
    #
    #     and declare the somefunc callback like this:
    #         def somefunc(ioaddr, value=None, /, *, opsize=2):
    #            ...
    #
    #     The opsize argument will be 2 for word operations and 1 for bytes.
    #     NOTE: Byte READS will never be seen; only byte WRITES.
    #
    #  *  DEVICES THAT DON'T CARE ABOUT BYTE-WRITES:
    #     The common/standard case. Let byte_writes default to False
    #     (i.e., leave it out of the registration call). The callback
    #     will be invoked with the simpler signature: f(ioaddr, v)
    #
    #  *  DEVICES THAT SUPPLY WORDS AT ODD ADDRESSES
    #     The cpu being the canonical example of this... register the
    #     I/O callback at the corresponding even address and use the ioaddr
    #     determine which address (the even or the odd) was requested.
    #
    # Devices that respond to a block of related addresses can register
    # one callback to cover more than one word of Unibus address space.
    # The callback gets the ioaddr which it can further decode. Again,
    # the CPU register block is a good example of this.
    #
    #
    # The PDP-11 RESET INSTRUCTION
    #
    # The PDP-11 RESET instruction causes the UNIBUS to be reset.
    # Devices that want to know about this should:
    #
    #      mmio.devicereset_register(resetfunc)
    #
    # And then resetfunc will be invoked as:
    #        resetfunc(mmio.ub)
    # (i.e., passed a single argument, the UNIBUS object).
    #
    # For dead-simple cases, the optional reset=True argument can be
    # supplied to register(), which will cause it to also arrange for the
    # iofunc to be called at RESET time, like this:
    #
    #       iofunc(baseaddr, 0)
    #
    # which, it should be noted, is indistinguishable from a program
    # merely setting that I/O address to zero. Note too that if iofunc
    # was registered once for an N-word block a RESET will still only call
    # it ONCE, on the baseaddr of that block.
    #
    # If this convenience, with its limitations, is insufficient for a
    # device then it must use the devicereset registration instead.
    #

    def __init__(self, cpu):
        self.cpu = cpu
        self.mmiomap = [self.__nodev] * (self.cpu.IOPAGE_SIZE >> 1)
        self.device_resets = set()

    # the default entry for unoccupied I/O: cause an AddressError trap
    def __nodev(self, addr, value=None, /):
        self.cpu.logger.info(f"Access to non-existent I/O {oct(addr)}")
        raise PDPTraps.AddressError(
            cpuerr=self.cpu.CPUERR_BITS.UNIBUS_TIMEOUT)

    # Devices may have simple "dummy" I/O addresses that always read zero
    # and ignore writes; See "if iofunc is None" in register() method.
    def __ignoredev(self, addr, value=None, /):
        self.cpu.logger.debug(f"dummy zero device @ {oct(addr)}, {value=}")
        return 0

    # register a call-back for an I/O address, which MUST be an offset
    # within the 8K I/O page (which itself may reside at three different
    # physical locations depending on configurations, thus explaining
    # why this routine deals only in the offsets).
    #
    # Variations:
    #   iofunc=None      -- implement a dummy: reads as zero, ignores writes
    #   reset=True       -- also registers iofunc to be called at RESET time
    #   byte_writes=True -- Request byte writes be sent to the iofunc
    #                       vs hidden/converted into word ops.
    #
    def register(self, iofunc, offsetaddr, nwords, *,
                 byte_writes=False, reset=False):

        if offsetaddr >= self.cpu.IOPAGE_SIZE:
            raise ValueError(f"MMIO: I/O offset too large {oct(offsetaddr)}")

        # None is a shorthand for "this is a dummy always-zero addr"
        if iofunc is None:
            iofunc = self.__ignoredev

        # register this (raw/unwrapped) iofunc for reset if so requested
        if reset:
            self.devicereset_register(lambda ub: iofunc(offsetaddr, 0))

        idx, odd = divmod(offsetaddr, 2)
        if odd != 0:
            # See discussion of odd I/O addrs in block comment elsewhere
            raise ValueError("cannot register odd (byte) address in IO space")

        if not byte_writes:
            # wrap the supplied I/O function with this code to implement
            # byte write operations automatically in terms of words.
            iofunc = self._byte_wrapper(iofunc)

        for i in range(nwords):
            self.mmiomap[idx+i] = iofunc
        return offsetaddr

    # wrap an I/O function with code that automatically handles byte writes.
    def _byte_wrapper(self, iofunc):
        def byteme(ioaddr, value=None, /, *, opsize=2):
            if (value is None) or (opsize == 2):
                return iofunc(ioaddr, value)
            else:
                # value is not None, and opsize is not 2
                # In other words: a byte write to I/O space. Synthesize it.
                self.cpu.logger.debug(f"Byte write to {oct(ioaddr)} {value=}")
                wv = self.wordRW(ioaddr)
                if ioaddr & 1:
                    wv = (wv & 0o377) | (value << 8)
                else:
                    wv = (wv & 0o177400) | value
                self.wordRW(ioaddr, wv)
        return byteme

    # Convenience method -- registers simple attributes (or properties) into
    # I/O space in the obvious way: Make this attr (of obj) show at this addr
    #
    # If a device just needs some attributes set to zero on a RESET,
    # it can specify them here with reset=True and they will be automatically
    # set to zero by reset() (no need to devicereset_register).
    def register_simpleattr(self, obj, attrname, addr, reset=False):
        """Create and register a handler to read/write the named attr.

        obj - the object (often "self" for the caller of this method)
        attrname - the attribute name to read/write
        addr - the I/O address to register it to

        If attrname is None, the address is registered as a dummy location
        that ignores writes and will always read as zero. This is sometimes
        useful for features that have to exist but are emulated as no-op.
        """

        # could do this with partial, but can also do it with this nested
        # func def. One way or another need this func logic anyway.

        def _rwattr(_, value=None, /):
            """Read/write the named attr via the I/O callback protocol."""
            if attrname is None:
                value = 0
            else:
                if value is None:
                    value = getattr(obj, attrname)
                else:
                    setattr(obj, attrname, value)
            return value

        # NOTE: Registers a different ("closure of") rwattr each time.
        self.register(_rwattr, addr, 1, reset=reset)

    # In the real hardware, the PDP-11 RESET instruction pulls a reset line
    # that all devices can see. In the emulation, devices that need to know
    # about the RESET instruction must register themselves here:
    def devicereset_register(self, func):
        """Register func to be called whenever a RESET happens."""
        self.device_resets.add(func)

    # The PDP-11 RESET instruction eventually ends up here, causing
    # a bus reset to be sent to all known registered devices.
    def resetdevices(self):
        for f in self.device_resets:
            self.cpu.logger.debug(f"RESET callback: {f}")
            f(self.cpu.ub)

    def wordRW(self, ioaddr, value=None, /):
        """Read (value is None) or write the given I/O address."""
        if value is None:
            value = self.mmiomap[ioaddr >> 1](ioaddr)
        else:
            self.mmiomap[ioaddr >> 1](ioaddr, value)
        return value

    def byteRW(self, ioaddr, value=None, /):
        """UNIBUS byte R/W - only write is legal."""
        if value is None:
            raise ValueError("Unibus cannot read bytes")
        else:
            self.mmiomap[ioaddr >> 1](ioaddr, value, opsize=1)
            return None
