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

# Emulate (a bare subset of) RP04..07 RM02-80 disks

from types import SimpleNamespace


class RPRM:
    RPADDR_OFFS = 0o16700

    NSECT = 22               # sectors per track
    NTRAC = 19               # tracks per cylinder
    SECTOR_SIZE = 512

    # NOTE: The key names become the attribute names. See __init__
    HPREG_OFFS = {
        'CS1': 0o00,         # control and status register
        'WC': 0o02,          # word count
        'UBA': 0o04,         # UNIBUS address
        'DA': 0o06,          # desired address
        'CS2': 0o10,         # control/status register 2
        'DS': 0o12,          # drive status
        'AS': 0o16,          # unified attention status
        'RMLA': 0o20,        # lookahead (sector under head!!)
        'OFR': 0o32,         # heads offset -- seriously, boot program??
        'DC': 0o34,          # desired cylinder
        'CC': 0o36,          # "current cylinder" and/or holding register
        'BAE': 0o50,         # address extension (pdp11/70 extra phys bits)
        }

    HPDS_BITS = SimpleNamespace(
        OFM=0o000001,               # offset mode
        VV=0o000100,                # volume valid
        DRY=0o000200,               # drive ready
        DPR=0o000400,               # drive present
        MOL=0o010000,               # medium online
        )

    HPCS1_BITS = SimpleNamespace(
        GO=0o000001,         # GO bit
        FN=0o000076,         # 5 bit function code - this is the mask
        IE=0o000100,         # Interrupt enable
        RDY=0o000200,        # Drive ready
        A16=0o000400,
        A17=0o001000,
        TRE=0o040000,
        )

    def __init__(self, ub, d0name='rp.disk', /, *dnames, baseoffs=RPADDR_OFFS):
        self.addr = baseoffs
        self.ub = ub
        self.logger = ub.logger

        self.command_history = [(0, tuple())] * 100
        self._diskimage = open(d0name, 'r+b')

        # multiple drive support is not yet implemented
        if dnames:
            raise ValueError("multiple drives not yet supported in RP")

        for attr, offs in self.HPREG_OFFS.items():
            setattr(self, attr, 0)

            # CS1 is a special case in several ways
            if attr == 'CS1':
                ub.mmio.register(self.rw_cs1, baseoffs+offs, 1,
                                 byte_writes=True, reset=True)
            else:
                # the rest are simple attributes; some as properties
                ub.mmio.register_simpleattr(self, attr, baseoffs+offs)

        # XXX obviously this is just fake for now
        self.DS = (self.HPDS_BITS.DPR | self.HPDS_BITS.MOL |
                   self.HPDS_BITS.VV | self.HPDS_BITS.DRY)

    def __del__(self):
        try:
            self._diskimage.close()
        except (AttributeError, TypeError):
            pass
        self._diskimage = None

        # Pass __del__ up the inheritance tree, carefully.
        # Note that __del__ is not always defined, Because Reasons.
        getattr(super(), '__del__', lambda self: None)(self)

    @property
    def UBA(self):
        return self._uba

    @UBA.setter
    def UBA(self, value):
        self.logger.debug(f"UBA address being set to {oct(value)}")
        self._uba = value

    @property
    def CS2(self):
        return self._cs2

    @CS2.setter
    def CS2(self, value):
        self.logger.debug(f"CS2: value={oct(value)}")
        self._cs2 = value

    @property
    def DS(self):
        return (self._ds | self.HPDS_BITS.DPR | self.HPDS_BITS.MOL |
                self.HPDS_BITS.VV | self.HPDS_BITS.DRY)

    @DS.setter
    def DS(self, value):
        self._ds = value

    @property
    def CS1(self):
        # XXX what if CS1 is just always RDY??
        self._cs1 |= self.HPCS1_BITS.RDY

        # --- XXX DEBUGGING XXX ---
        if (self._cs1 & 0x4000):
            self.logger.debug(f"RP: XXX! CS1={oct(self._cs1)}")
        self.logger.debug(f"RP: reading CS1: {oct(self._cs1)}")
        return self._cs1

    @CS1.setter
    def CS1(self, value):
        self.command_history.pop(-1)
        self.command_history.insert(0, (value, self.statestring()))

        self.logger.debug(f"RP: writing CS1 to {oct(value)}; "
                          f"state: {self.statestring()}")
        self._cs1 = value
        self.logger.debug(f"RP: CS1 set to {oct(self._cs1)}")
        if self._cs1 & 0x4000:
            self.logger.debug(f"LOOK!!!! XXX")
        if self._cs1 & self.HPCS1_BITS.RDY:
            self.AS = 1            # this is very bogus but maybe works for now

        # TRE/ERROR always cleared on next op
        if value & self.HPCS1_BITS.GO:
            self._cs1 &= ~self.HPCS1_BITS.TRE

        match value & self.HPCS1_BITS.FN, value & self.HPCS1_BITS.GO:
            case 0, go:
                self._cs1 &= ~go

            case 0o06 | 0o12 | 0o16 | 0o20 | 0o22 as fcode, go:
                self.logger.debug(f"RP: operation {oct(fcode)} ignored.")
                self.logger.debug(self.statestring())
                self._cs1 &= ~(go | fcode)
                self._cs1 |= self.HPCS1_BITS.RDY
                if self._cs1 & self.HPCS1_BITS.IE:
                    self.ub.intmgr.simple_irq(5, 0o254)

            case 0o30, 1:          # SEARCH
                self._cs1 &= ~0o31
                self._cs1 |= self.HPCS1_BITS.RDY
                self.CC = self.DC
                if self._cs1 & self.HPCS1_BITS.IE:
                    self.ub.intmgr.simple_irq(5, 0o254)

            case 0o60, 1:
                self._cs1 &= ~0o61
                self.writecmd()
                if self._cs1 & self.HPCS1_BITS.IE:
                    self.ub.intmgr.simple_irq(5, 0o254)

            case 0o70, 1:
                self._cs1 &= ~0o71
                self.readcmd()
                if self._cs1 & self.HPCS1_BITS.IE:
                    self.ub.intmgr.simple_irq(5, 0o254)

            case _, 0:            # anything else without the go bit is a nop
                pass

            case _:               # but with the go bit, bail out for now
                raise ValueError(value)

    # special function for handling writes to the CS1 attribute
    # Because byte writes to the upper byte need to be treated carefully
    def rw_cs1(self, addr, value=None, /, *, opsize=2):

        if opsize == 1:
            # by definition byte reads are impossible; this will obviously
            # bomb out if they happen somehow (it is physically impossible
            # to have a byte write on the real UNIBUS)
            value &= 0o377          # paranoia but making sure
            self.logger.debug(f"RP: BYTE addr={oct(addr)}, "
                              f"{value=}, _cs1={oct(self._cs1)}")
            self.logger.debug(self.statestring())
            if addr & 1:
                self._cs1 = (value << 8) | (self._cs1 & 0o377)
            else:
                self.CS1 = (self._cs1 & 0o177400) | value
        elif value is None:
            return self.CS1          # let property getter do its thing
        else:
            self.CS1 = value         # let property setter do its thing
        return None

    def _compute_offset(self):
        # cyl num, track num, sector num, which were written like this:
        #    HPADDR->hpdc = cn;
        #    HPADDR->hpda = (tn << 8) + sn;
        cn = self.DC
        tn = (self.DA >> 8) & 0o377
        sn = self.DA & 0o377

        # each cylinder is NSECT*NTRAC sectors
        # each track is NSECT sectors
        offs = cn * (self.NSECT * self.NTRAC)
        offs += (tn * self.NSECT)
        offs += sn
        offs *= self.SECTOR_SIZE
        return offs

    def readcmd(self):
        offs = self._compute_offset()
        self.logger.debug(f"RP READ: offs=0x{hex(offs)}, {self.WC=}")

        addr = self._getphysaddr()
        self._diskimage.seek(offs)
        nw = (65536 - self.WC)
        sector = self._diskimage.read(nw*2)

        # Note conversion: from little-endian on disk to native 0 .. 65535
        self.ub.cpu.physRW_N(addr, nw, self.__b2wgen(sector))
        self.WC = 0
        self.CS1 |= self.HPCS1_BITS.RDY

    def writecmd(self):
        offs = self._compute_offset()
        self.logger.debug(f"RP WRITE: offs=0x{hex(offs)}, {self.WC=}")

        addr = self._getphysaddr()
        self._diskimage.seek(offs)
        nw = (65536 - self.WC)

        # Words in physmem are just python integers; they have to be
        # converted into a little-endian byte stream for disk...
        sector = bytes(self.__w2bgen(self.ub.cpu.physRW_N(addr, nw)))
        self._diskimage.write(sector)
        self.WC = 0
        self.CS1 |= self.HPCS1_BITS.RDY

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
        # low 16 bits in UBA, and tack on A16/A17
        A16 = bool(self.CS1 & self.HPCS1_BITS.A16)
        A17 = bool(self.CS1 & self.HPCS1_BITS.A17)

        # but also bits may be found in bae... the assumption here is
        # if these bits are non-zero they override A16/A17 but they
        # really need to be consistent...
        if self.BAE == 0:
            A1621 = 0
        else:
            A16 = 0        # subsumed in A1621
            A17 = 0        # subsumed
            A1621 = self.BAE & 0o77

        phys = self.UBA | (A16 << 16) | (A17 << 17) | (A1621 << 16)
        self.logger.debug(f"RP: physaddr={oct(phys)}")
        return phys
        # return self.UBA | (A16 << 16) | (A17 << 17) | (A1621 << 16)

    def statestring(self):
        s = "RP XXX:"
        for attr in self.HPREG_OFFS:
            s += f"{attr}={oct(getattr(self, attr, 0))} "
        return s

    # produce a pretty-print version of a single RP history
    @staticmethod
    def rph_pps(rph):
        written = rph[0]
        s = f"CS1 <-- {oct(written)} : "
        cmd = written & 0o70
        s += {0o70: 'READ', 0o60: 'WRITE', 0o30: 'SEARCH'}.get(cmd, oct(cmd))
        if rph[0] & 1:
            s += "|GO"
        if written & 0o100:
            s += "|IE"
        if written & 0o040000:
            s += "|TRE"
        s += f"\n       {rph[1]}"
        return s
