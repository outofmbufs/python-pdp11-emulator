# MIT License
#
# Copyright (c) 2024 Neil Webber
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

# Emulate an RK11 controller with one or more RK05 drives


from types import SimpleNamespace
from unibus import BusCycle


class RK11:
    """RK11/RK05 emulation."""

    RKADDR_OFFS = 0o17400
    NSECT = 12               # sectors per track
    NTRAC = 2                # tracks per cyclinder
    SECTOR_SIZE = 512
    MAXDRIVES = 8            # limitation of 3-bit fields in various registers
    VECTOR = 0o220           # conceptually configuarable but no one ever does
    INTLVL = 5               # ditto

    DEFAULT_DRIVE_0_NAME = 'rk0.disk'

    RKREG_OFFS = {
        'RKDS': 0o00,        # Drive Status (read only)
        'RKER': 0o02,        # Error Register (read only)
        'RKCS': 0o04,        # Control/Status register
        'RKWC': 0o06,        # Word Count register
        'RKBA': 0o10,        # Bus Address
        'RKDA': 0o12,        # Disk Address
        # note: there is no 0o14 (might be a maintenance/diag register?)
        'RKDB': 0o16         # Data Buffer
    }

    # NOTE: This is not all the fields in the DS register. Some are multi-bit
    #       fields (sector counter, drive ident) and some are not implemented
    #       here (i.e., always zero)
    RKDS_BITS = SimpleNamespace(
        RWSRDY=0o100,        # read/write/seek ready
        DRY=0o200,           # drive ready
        SOK=0o400,           # sector counter OK ("ready")
        RK05=0o4000,         # selected drive is an RK05 and online
        HE=0o040000,         # variety of hard errors
        ERR=0o100000,        # implies something is in RKER
    )

    RKCS_BITS = SimpleNamespace(
        GO=0o000001,         # GO bit
        FN=0o000016,         # 3 bit function code - this is the mask
        A16=0o000020,        # MEX 0 (17th address bit)
        A17=0o000040,        # MEX 1 (18th address bit)
        IDE=0o000100,        # Interrupt Enable ("interrupt on done")
        RDY=0o000200,        # Control ready ("write only")
        FMT=0o002000,        # formatting - NOT IMPLEMENTED
        IBA=0o004000,        # I/O to 1 memory location - NOT IMPLEMENTED
        HE=0o040000,         # "hard" error
        ERR=0o100000,        # error (any error)
    )

    RKER_BITS = SimpleNamespace(
        WCE=0o000001,        # write-check error
        CSE=0o000002,        # read checksum error (not emulated)
        NXS=0o000040,        # non-existent sector
        NXC=0o000100,        # non-existent cylinder
        NXD=0o000200,        # non-existent disk
        TE=0o000400,         # timing error (not emulated)
        DLT=0o001000,        # data late (not emulated)
        NXM=0o002000,        # non-existent memory
        PGE=0o004000,        # programming error (see book)
        SKE=0o010000,        # seek error (not emulated)
        WLO=0o020000,        # write lockout (write to write protected disk)
        OVR=0o040000,        # operation beyond end-of-disk
        DRE=0o100000,        # drive error (not emulated)
    )

    def __init__(self, ub, *names, baseoffs=RKADDR_OFFS):
        self.addr = baseoffs
        self.ub = ub
        self.logger = ub.logger

        self.command_history = [(0, tuple())] * 100

        # if anything below errors out, need to have _diskimages for __del__
        self._diskimages = list()     # will be expanded later

        if len(names) == 0:
            names = [self.DEFAULT_DRIVE_0_NAME]
        elif len(names) > self.MAXDRIVES:           # c'mon man
            raise ValueError(f"more than {self.MAXDRIVES} drive files")

        # force it list, and to length MAXDRIVES (grow it if necessary)
        names = (list(names) + ([None] * self.MAXDRIVES))[:self.MAXDRIVES]

        self._diskimages = list(
            map(lambda nm: open(nm, 'r+b') if nm else None, names))

        self._writelocks = set()

        for attr, offs in self.RKREG_OFFS.items():
            setattr(self, attr, 0)

            ioadr = baseoffs + offs
            if attr == 'RKCS':               # RKCS has explicit handler
                ub.register(self.rw_rkcs, ioadr)
            elif attr in {'RKDS', 'RKER'}:   # these are read-only
                ub.register_simpleattr(self, attr, ioadr, readonly=True)
            else:
                ub.register_simpleattr(self, attr, ioadr)

        # a plausible initial status
        self.RKDS = (self.RKDS_BITS.RWSRDY | self.RKDS_BITS.DRY |
                     self.RKDS_BITS.SOK | self.RKDS_BITS.RK05)

    def __del__(self):
        for f in filter(None, self._diskimages):
            try:
                f.close()
            except (AttributeError, TypeError):
                pass
        self._diskimages = None

        # Pass __del__ up the inheritance tree, carefully.
        # Note that __del__ is not always defined, Because Reasons.
        getattr(super(), '__del__', lambda self: None)(self)

    @property
    def RKCS(self):
        # all ops are synchronous; therefore always "ready"
        self._rkcs |= self.RKCS_BITS.RDY

        # automatically generate the ERR bit from RKER (per spec)
        if self.RKER:
            self._rkcs |= self.RKCS_BITS.ERR
        else:
            self._rkcs &= ~self.RKCS_BITS.ERR
        return self._rkcs

    @RKCS.setter
    def RKCS(self, value):
        self.command_history.pop(-1)
        self.command_history.insert(0, (value, self.statestring()))

        self.logger.debug(f"RK: writing RKCS to {oct(value)}; "
                          f"state: {self.statestring()}")
        self._rkcs = value

        cmd = self._rkcs & self.RKCS_BITS.FN      # NOTE: not shifted
        gobit = self._rkcs & self.RKCS_BITS.GO

        # NOT IMPLEMENTED:
        #    formatting
        #    I/O that reads or writes from/to only 1 memory location
        if self._rkcs & (self.RKCS_BITS.FMT | self.RKCS_BITS.IBA):
            self.logger.info("RK: UNIMPLEMENTED")
            self.RKER = self.RKER_BITS.PGE
            gobit = 0                       # force a no-op

        match cmd, gobit:
            case 0o00, 1:                   # control reset
                # per manual "clears all bits of the seven programmable
                # registers except RKDS 01 through 11; sets RDY in RKCS
                # Why it says 01-11 and not 00-11 is a mystery; nevertheless
                # it's irrelevant as the emulated sector counter is always 0
                self.RKDS &= ~0o170000       # preserve bits 12-15
                self.RKER = 0
                self._rkcs = self.RKCS_BITS.RDY
                self.RKWC = 0
                self.RKDA = 0
                self.RKDB = 0

            case 0o02, 1:                    # write
                self.writecmd()

            case 0o04, 1:                    # read
                self.readcmd()

            case 0o06, 1:                    # write-check
                self.writecheck()

            # seek, read-check, reset all no-ops
            case (0o10 | 0o12 | 0o14), 1:
                pass

            case 0o16, 1:                    # write lock (write protect)
                self.writelock()

            case _, 0:            # anything else without the go bit is a nop
                pass

            case _:               # but with the go bit, bail out for now
                assert False, "case not reached .. cmd match??"

        # All ops are synchronous I/O in the host system, therefore the "done"
        # interrupt can be blindly generated here.
        if gobit and (self._rkcs & self.RKCS_BITS.IDE):
            self.ub.intmgr.simple_irq(self.INTLVL, self.VECTOR)

    # explicit handler for I/O to the RKCS so that WRITE8 cycles handled
    def rw_rkcs(self, addr, cycle, /, *, value=None):
        if cycle == BusCycle.WRITE8:
            value &= 0o377          # paranoia but making sure
            self.logger.debug(f"RK11: BYTE addr={oct(addr)}, "
                              f"{value=}, _rkcs={oct(self._rkcs)}")
            self.logger.debug(self.statestring())
            if addr & 1:
                self._rkcs = (value << 8) | (self._rkcs & 0o377)
            else:
                self.RKCS = (self._rkcs & 0o177400) | value
        elif cycle == BusCycle.READ16:
            return self.RKCS          # let property getter do its thing
        elif cycle == BusCycle.RESET:
            self.RKCS = 0
        elif cycle == BusCycle.WRITE16:
            self.RKCS = value         # let property setter do its thing
        else:
            assert False, "not reached or unknown cycle"
        return None

    def _get_drive_f(self, /, *, writing=False):
        drive_n = (self.RKDA >> 13) & 0o7
        drive_f = self._diskimages[drive_n]
        if drive_f is None:
            self.RKER = self.RKER_BITS.NXD
            self._rkcs |= self.RKCS_BITS.HE
        elif writing and drive_f in self._writelocks:
            self.RKER = self.RKER_BITS.WLO
            drive_f = None
        return drive_f

    def _compute_offset(self):
        # drive, cyl num, surface, sector num, which in unix v7
        # were computed and set like this:
        #    bn = bp->b_blkno;
        #    dn = minor(bp->b_dev);
        #    cn = bn/12;
        #    sn = bn%12;
        #    RKADDR->rkda = (dn<<13) | (cn<<4) | sn;
        #
        # In effect this concieves of the disk as having twice as many
        # cylinders, and one surface, which is fine. Anyhow, one way
        # or another turn that back into a byte offset in the disk image

        cn = (self.RKDA >> 4) & 0o777
        return self.SECTOR_SIZE * ((cn * self.NSECT) + (self.RKDA & 0o17))

    def readcmd(self):
        if (f := self._get_drive_f()) is None:
            return

        offs = self._compute_offset()
        self.logger.debug(f"RK READ: offs=0x{hex(offs)}, {self.RKWC=}")

        addr = self._getphysaddr()
        f.seek(offs)
        nw = (65536 - self.RKWC)
        databytes = f.read(nw*2)

        # Note conversion: from little-endian on disk to native 0 .. 65535
        self.ub.cpu.physRW_N(addr, nw, self.__b2wgen(databytes))
        self.RKWC = 0

        # update (increment) the bus address including A16/A17 overflow
        self._rkba_adjust(nw)

    def writecmd(self):
        if (f := self._get_drive_f(writing=True)) is None:
            return

        offs = self._compute_offset()
        self.logger.debug(f"RK WRITE: offs=0x{hex(offs)}, {self.RKWC=}")

        addr = self._getphysaddr()
        f.seek(offs)
        nw = (65536 - self.RKWC)

        # Words in physmem are just python integers; they have to be
        # converted into a little-endian byte stream for disk...
        databytes = bytes(self.__w2bgen(self.ub.cpu.physRW_N(addr, nw)))
        f.write(databytes)
        self.RKWC = 0

        # update (increment) the bus address including A16/A17 overflow
        self._rkba_adjust(nw)

    def _rkba_adjust(self, nw):
        """Add nw WORDS to RKBA and overflow into A16/A17 as needed."""

        addr = self._getphysaddr()
        addr += (nw * 2)
        self.RKBA = addr & 0o177777
        for m, b in ((0o200000, self.RKCS_BITS.A16),
                     (0o400000, self.RKCS_BITS.A17)):
            if addr & m:
                self._rkcs |= b
            else:
                self._rkcs &= ~b

    def __b2wgen(self, b):
        """Generate native python ints from sequence of little endian bytes"""
        g = iter(b)
        for lo in g:
            hi = next(g)
            yield lo + (hi << 8)

    def __w2bgen(self, words):
        """Generate little-endian bytes from sequence of python ints"""
        for w in words:
            yield w & 0o377
            yield (w >> 8) & 0o377

    def _getphysaddr(self):
        # low 16 bits in RKBA, and tack on A16/A17
        A16 = bool(self.RKCS & self.RKCS_BITS.A16)
        A17 = bool(self.RKCS & self.RKCS_BITS.A17)
        phys = self.RKBA | (A16 << 16) | (A17 << 17)
        self.logger.debug(f"RK: physaddr={oct(phys)}")
        return phys

    def statestring(self):
        s = "RK11:"
        for attr in self.RKREG_OFFS:
            s += f"{attr}={oct(getattr(self, attr, 0))} "
        return s
