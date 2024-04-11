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

from functools import partial
from pdptraps import PDPTraps, PDPTrap
from unibus import BusCycle
from types import SimpleNamespace
from collections import namedtuple


class MemoryMgmt:
    ISPACE = 0
    DSPACE = 1

    # I/O addresses for various registers relative to I/O page base
    # From the pdp11/70 (and others) 1981 processor handbook, Appendix A
    #
    # Each block is:
    #      I PDR (8 16-bit registers)
    #      D PDR (8 16-bit registers)
    #      I PAR (8 16-bit registers)
    #      D PAR (8 16-bit registers)
    #
    # so each block is 64 bytes total, octal 100 in size
    #
    APR_SUPER_OFFS = 0o12200          # offset within I/O page
    APR_KERNEL_OFFS = 0o12300
    APR_USER_OFFS = 0o17600           # 0o17 vs 0o12 is not a typo

    # expressed as offsets within the I/O page
    MMR0_OFFS = 0o17572
    MMR1_OFFS = 0o17574
    MMR2_OFFS = 0o17576
    MMR3_OFFS = 0o12516

    # not an Enum because ... need to do bitwise efficiently.
    MMR0_BITS = SimpleNamespace(
        ABORT_NR=0o100000, ABORT_PLENGTH=0o040000, ABORT_RDONLY=0o020000,
        TRAP_MGMT=0o010000, TRAP_ENABLE=0o001000, INST_COMPLETED=0o000200,
        RELO_ENABLE=0o000001, FREEZER_TRAPS=0o160000,
    )

    # memory control (parity, etc) is not implemented but needs to respond
    MCR_OFFS = 0o17746

    TransKey = namedtuple('TransKey', ('segno', 'mode', 'space', 'reading'))

    def __init__(self, cpu, /, *, nocache=False):

        self.cpu = cpu
        self.ub = cpu.ub

        # The "segment cache" dramatically speeds up address translation
        # for the most common MMU usage scenarios.
        #
        # To preserve correct semantics, modifications to the mapping
        # parameters must (of course) dump some, or all, of this cache.
        self.segcache = {}

        # per the architecture manual, there are six (!) sets of
        # eight 32-bit Active Page Registers (APR0 ... APR7) where
        # each APR can be thought of as a 16-bit page address register (PAR)
        # and a 16 bit page descriptor register (PDR).
        #
        # A set (of 8) APRs is selected by a combination of two PSW bits
        # for kernel/supervisor/illegal/user modes and, if I/D separation
        # is enabled, I vs D space.
        #
        # To simplify the mapping, 8 sets (instead of 6) are provided here
        # but one of them is never used because one of the four combinations
        # of the psw mode bits is illegal (enforced elsewhere).

        self.APR = [[[0, 0] for _ in range(8)] for setno in range(8)]

        self.MMR0 = 0
        self.MMR1 = 0
        self.MMR2 = 0
        self.MMR3 = 0

        self.MMR1_staged = 0
        self.nocache = nocache

        # to accelerate v2p, instead of having to look up the status of
        # I/D separation and possibly fold I/D space requests, compute
        # a shadow "unfolded" version that bakes that in.
        self._unfoldAPR()

        # register I/O space PDR/PARs: SUPER/KERNEL/USER blocks of 32 regs.
        #
        # It turns out all of this context is encoded (cleverly and
        # not entirely obviously) in the ioaddr bits, but it just seems
        # better to make it explicit here via extra args and partial():
        for mode, base in (
                (cpu.SUPERVISOR, self.APR_SUPER_OFFS),
                (cpu.KERNEL, self.APR_KERNEL_OFFS),
                (cpu.USER, self.APR_USER_OFFS)):
            # Each block is:
            #      I PDR (8 16-bit registers)
            #      D PDR (8 16-bit registers)
            #      I PAR (8 16-bit registers)
            #      D PAR (8 16-bit registers)
            for parpdr, space, offset in (
                    (1, self.ISPACE, 0),
                    (1, self.DSPACE, 16),
                    (0, self.ISPACE, 32),
                    (0, self.DSPACE, 48)):
                ioaddr = base+offset
                iofunc = partial(self.io_parpdr, parpdr, mode, space, ioaddr)
                self.ub.register(iofunc, ioaddr, 8)  # 8 words / 16 bytes

        # register the simple attrs MMR0 etc into I/O space:
        self.ub.register_simpleattr(self, 'MMR0', self.MMR0_OFFS, reset=True)
        self.ub.register_simpleattr(self, 'MMR1', self.MMR1_OFFS)
        self.ub.register_simpleattr(self, 'MMR2', self.MMR2_OFFS)
        self.ub.register_simpleattr(self, 'MMR3', self.MMR3_OFFS, reset=True)
        self.ub.register_simpleattr(self, None, self.MCR_OFFS)

    def io_parpdr(self, parpdr, mode, space, base,
                  addr, cycle, /, *, value=None):
        """I/O function for MMU PARs and PDRs.

        NOTE: parpdr/mode/space/base args provided via partial() as
              supplied at registration time; see __init__.
        """
        aprnum = (addr - base) >> 1
        aprfile = self.APR[(mode * 2) + space]
        if cycle == BusCycle.READ16:
            return aprfile[aprnum][parpdr]
        elif cycle == BusCycle.WRITE16:
            # dump any matching cache entries in both reading/writing form.
            for reading in (True, False):
                # the "space" is a dilemma because it is tied up in
                # the unfolding of I/D space separation. It's not hard
                # to figure out what to do but its also very easy to
                # just do this: nuke both I and D space cache entries.
                for xspc in (self.ISPACE, self.DSPACE):
                    if (aprnum, mode, xspc, reading) in self.segcache:
                        del self.segcache[(aprnum, mode, xspc, reading)]

            aprfile[aprnum][parpdr] = value

            # Per the handbook - the A and W bits in a PDR are reset to
            # zero when either the PAR or PDR is written.
            aprfile[aprnum][1] &= ~0o0300
            self._unfoldAPR()       # update the I/D unfolded

    @property
    def MMR0(self):
        return self._mmr0

    @MMR0.setter
    def MMR0(self, value):
        self._mmr0 = value
        self._mmu_relo_enabled = (value & self.MMR0_BITS.RELO_ENABLE)
        self._mmu_trap_enabled = (value & self.MMR0_BITS.TRAP_ENABLE)
        self._mmr12_frozen = (value & self.MMR0_BITS.FREEZER_TRAPS)
        self.segcache = {}
        self.__rebaseIO()

    # MMR1 records any autoincrement/decrement of the general purpose
    # registers, including explicit references through the PC. MMR1 is
    # cleared at the beginning of each instruction fetch. It is really
    # two subregisters each 8 bits, that record:
    #     Bits <7:3> two's complement amount changed
    #     Bits <2:0> register number (0 .. 7)
    #
    # Register set must be determined from appropriate PSW field(s)
    #
    # This is in the critical path for instruction performance, so
    # there is an optimization. At the beginning of every instruction
    # self.MMR1_staged is set to zero. Then "MMR1mod()" is used to
    # record any modifications (they still go into MMR1_staged) and
    # only when an MMU trap is generated are the staged values potentially
    # transferred into MMR1.
    #
    # This keeps the overhead down to a single self.MMR1_staged = 0
    # assignment for instructions that do not auto inc/dec and do not
    # cause MMU faults.

    def MMR1mod(self, value):
        # record the given 8-bit register modification
        if value == 0 or value > 255:        # this should never happen
            raise ValueError(f"bogus MMR1mod {value=}")

        if self.MMR1_staged == 0:
            self.MMR1_staged = value
        else:
            self.MMR1_staged |= (value << 8)

    def _MMR1commit(self):
        if not self._mmr12_frozen:
            self.MMR1 = self.MMR1_staged

    @property
    def MMR2(self):
        return self._mmr2

    @MMR2.setter
    def MMR2(self, value):
        if not self._mmr12_frozen:
            self._mmr2 = value

    @property
    def MMR3(self):
        cpu = self.cpu
        return (
            ((self._unibusmap & 1) << 5) |
            ((self._22bit & 1) << 4) |
            (int(self._Dspaces[cpu.KERNEL] == self.DSPACE) << 2) |
            (int(self._Dspaces[cpu.SUPERVISOR] == self.DSPACE) << 1) |
            (int(self._Dspaces[cpu.USER] == self.DSPACE)))

    @MMR3.setter
    def MMR3(self, value):
        self._unibusmap = (value >> 5) & 1
        self._22bit = (value >> 4) & 1
        self.segcache = {}
        self.__rebaseIO()      # because 22bit affects where it ends up

        # store which space to use for D-space lookups
        self._Dspaces = {mode: [self.ISPACE, self.DSPACE][bool(value & mask)]
                         for mode, mask in ((self.cpu.KERNEL, 4),
                                            (self.cpu.SUPERVISOR, 2),
                                            (self.cpu.USER, 1))}

        # to accelerate v2p the I/D space folding is precomputed
        self._unfoldAPR()

    def __rebaseIO(self):
        """Where oh where has my little I/O page gone?"""

        # whenver relo_enabled or _22bit change, which inconveniently
        # are in separate MMR registers, the I/O potentially moves.
        # Figure out where to put it.
        self.iopage_base = 0o160000             # end of the 16 bit space
        if self._mmu_relo_enabled:
            self.iopage_base |= (3 << 16)       # 2 more bits (18 total)
            if self._22bit:
                self.iopage_base |= (15 << 18)  # ... and 4 more

    def v2p(self, vaddr, mode, space, reading, *, invis=False):
        """Convert a 16-bit virtual address to physical.

        invis=True when being used for logging or other examinations that
              should not alter any machine state, including MMU state.

        NOTE: raises PDPTraps.MMU for untranslatable addresses
        NOTE: If not invis, updates A/W bits & sets straps as needed.
        NOTE: 'reading' MUST be True or False, not anything else.
        """

        if not self._mmu_relo_enabled:
            return vaddr

        if mode is None:                    # the normal (not mtpi etc) case
            mode = self.cpu.psw_curmode

        # the virtual address is broken down into three fields:
        #   <15:13> APF active page field. Selects the APR = par,pdr pair
        #           This is sometimes called the "segment number"
        #   <12:6> The "block number"
        #   <5:0> The displacement in block
        #
        # The block number will be added to the page address field in the par.
        # That whole thing is shifted left 6 bits and or'd with the
        # displacement  within block. All this is per the PDP11 manuals.

        segno = vaddr >> 13

        # the segment number and these other parameters form
        # a "translation key" used in several places. Unfortunately,
        # the namedtuple construction is enough overhead to matter in
        # this critical/fast path, so caching uses tuple key
        tuplexkey = (segno, mode, space, reading)

        # All this translation code takes quite some time; caching
        # dramatically improves performance.

        if self.nocache and self.segcache:
            self.segcache = {}
        try:
            xoff, validation_func = self.segcache[tuplexkey]
            if (validation_func is None) or validation_func(vaddr):
                return vaddr + xoff
        except KeyError:
            pass

        # AFTER the critical path can validate 'reading' which MUST
        # be either True or False, and not just 'truthy'. It's ok
        # to only check this after the fast path because bad values
        # can't get into the fast path (since they are filtered here).
        #
        # NOTE: This is a coding error if it happens. This is why it is
        #       not being silently corrected instead of ValueError'd.
        if reading not in (True, False):
            raise ValueError(f"Illegal value for reading: '{reading}'")

        xkey = self.TransKey(segno, mode, space, reading)

        # not cached; do the translation...

        par, pdr = self._getunfoldedapr(xkey)

        # In 22bit mode, the full 16 bits of the PAR are used.
        # In 18bit mode, the top four have to be masked off here.
        if not self._22bit:
            par &= 0o7777

        # access checks:
        #   "Aborts" (per the processor handbook) raise PDPTraps.MMU and
        #   do not return from accesschecks()
        #
        #   If there are "memory management traps" (which are to occur
        #   at the *end* of instruction execution) they are returned as
        #   bits suitable for OR'ing into cpu.straps; note that this
        #   condition also prevents caching the APR. If the mgmt trap
        #   handler modifies the APR to disable the management trap then
        #   of course future lookups will be eligible for the cache then.

        straps = self._v2p_accesschecks(pdr, vaddr, xkey, invis)

        # the actual translation...
        bn = (vaddr >> 6) & 0o177
        plf = (pdr >> 8) & 0o177
        if (pdr & 0o10) and bn < plf:
            self._raisetrap(self.MMR0_BITS.ABORT_PLENGTH, vaddr, xkey, invis)
        elif (pdr & 0o10) == 0 and bn > plf:
            self._raisetrap(self.MMR0_BITS.ABORT_PLENGTH, vaddr, xkey, invis)

        # "Attention" (not "Access") and "Written" bits updates.
        # The W bit is indeed a "memory was written" bit.
        # The A bit, however, is not "access" but rather only set when
        # there is a memory management trap (not abort) related to the access.
        #
        # Entries can only be cached if they did not cause a strap.
        # By definition, at that point (because reads/writes are separated)
        # there are no further A/W bit updates to worry about (so they
        # can be cached at that point).

        if not invis:
            self.cpu.straps |= straps
            W_update = 0o000 if reading else 0o100
            A_update = 0o200 if straps else 0o000
            AW_update = (W_update | A_update)

            if (pdr & AW_update) != AW_update:
                self._putapr(xkey, (par, pdr | AW_update))

        dib = vaddr & 0o77
        pa = ((par + bn) << 6) | dib

        # only non-trapping can be cached
        if straps == 0 and not invis:
            self._encache(tuplexkey, pdr, pa - vaddr)
        return pa

    def _encache(self, k, pdr, offs):
        # the validation function (lambdas) is constructed for cases
        # where the segment is not full-length and therefore one more
        # check has to happen even on cache hits.
        plf = (pdr >> 8) & 0o177
        if pdr & 0o10 and plf > 0:
            self.segcache[k] = (offs, lambda a: ((a >> 6) & 0o177) >= plf)
        elif (pdr & 0o10) == 0 and plf < 0o177:
            self.segcache[k] = (offs, lambda a: ((a >> 6) & 0o177) <= plf)
        else:
            self.segcache[k] = (offs, None)   # full segment

    # to accelerate v2p, instead of having to look up the status of
    # I/D separation for a given mode and possibly folding D space
    # requests into the I space APRs, this computes an "unfolded" shadow
    # version of the APRs that v2p can just use directly.
    #
    # AS AN OPTIMIZATION: when a new unfolding is needed all that happens
    # is _unfoldedAPR is set to None. No unfolding happens until later when
    # _getunfoldedapr() calls this with "now=True". The advantage of this
    # deferral is to speed up context switch code which (for example) will
    # typically do 8 back to back PAR writes; there is no reason to do
    # a unfolding 8 times instead of just once, and context switch
    # performance does matter.
    def _unfoldAPR(self, now=False):
        if not now:
            self._unfoldedAPR = None         # will be done later!
            return
        self._unfoldedAPR = [[[0, 0] for _ in range(8)] for setno in range(8)]
        for mode in (self.cpu.SUPERVISOR, self.cpu.KERNEL, self.cpu.USER):
            for space in (self.ISPACE, self.DSPACE):
                src = (mode * 2) + self._foldspaces(mode, space)
                dst = (mode * 2) + space
                for segno in range(8):
                    self._unfoldedAPR[dst][segno] = self.APR[src][segno]

    def _foldspaces(self, mode, space):
        """Folds DSPACE back into ISPACE if DSPACE not enabled for mode"""
        return space if space == self.ISPACE else self._Dspaces[mode]

    def _getunfoldedapr(self, xkey):
        nth = (xkey.mode * 2) + xkey.space
        try:
            uf = self._unfoldedAPR[nth][xkey.segno]
        except TypeError:
            self._unfoldAPR(now=True)
            uf = self._unfoldedAPR[nth][xkey.segno]
        return uf

    def _putapr(self, xkey, apr):
        """xkey should be unfolded; this puts the apr to BOTH places"""
        unfoldednth = (xkey.mode * 2) + xkey.space
        # note that a putapr can only happen after a get, so the
        # unfoldedAPR is known to be unfolded already here
        self._unfoldedAPR[unfoldednth][xkey.segno] = list(apr)
        foldednth = (xkey.mode * 2) + self._foldspaces(xkey.mode, xkey.space)
        self.APR[foldednth][xkey.segno] = list(apr)

    def _v2p_accesschecks(self, pdr, vaddr, xkey, invis):
        """Raise traps or set post-instruction traps as appropriate.

        Returns straps flags (if any required).
        """

        straps = 0

        # There are aborts and "memory management traps".
        # As the handbook says:
        #     """Aborts are used to catch "missing page faults," prevent
        #        illegal access, etc.; traps are used as an aid in
        #        gathering memory management information
        #     """
        #
        # Thus, an "abort" raises a vector 4 (AddressError) exception and
        # a "management trap" sets a cpu bit to cause a vector 250 (MMU)
        # trap at the *completion* of the instruction.
        #
        # The 7 possible access control modes (pdr & 7) are:
        #
        #    000         -- abort all accesses
        #    001         -- read-only + mgmt trap (read)
        #    010         -- read-only no mgmt traps
        #    011         -- RESERVED/ILLEGAL, abort all accesses
        #    100         -- writable + mgmt trap (any)
        #    101         -- writable + mgmt trap if write
        #    110         -- writable no mgmt traps
        #    111         -- RESERVED/ILLEGAL abort all accesses

        # Things that are not decoded in the match are accesses that
        # cause no traps or aborts. So, for example, control mode 6
        # is not in the cases; nor is control mode 5 if reading.

        reading = xkey.reading
        match pdr & 7:
            # control modes 0, 3, and 7 are always aborts
            case 0 | 3 | 7:
                self.cpu.logger.debug(f"ABORT_NR trap, regs: "
                                      f"{list(map(oct, self.cpu.r))}"
                                      f", {oct(self.cpu.psw)}"
                                      f", PDR={oct(pdr)} {reading=}")
                self._raisetrap(self.MMR0_BITS.ABORT_NR, vaddr, xkey, invis)

            # control mode 1 is an abort if writing, mgmt trap if read
            case 1 if reading:
                straps = self.cpu.STRAPBITS.MEMMGT

            case 1 | 2 if not reading:
                self._raisetrap(
                    self.MMR0_BITS.ABORT_RDONLY, vaddr, xkey, invis)

            # control mode 4 is mgmt trap on any access (read or write)
            case 4:
                straps = self.cpu.STRAPBITS.MEMMGT

            # control mode 5 is mgmt trap if WRITING
            case 5 if not reading:
                straps = self.cpu.STRAPBITS.MEMMGT

        return straps

    def wordRW(self, vaddr, value=None, /, *,
               mode=None, space=ISPACE, _invis=False):
        """Read/write a word at virtual address vaddr.

        If value is None, perform a read and return a 16-bit value
        If value is not None, perform a write; return None.
        """

        pa = self.v2p(vaddr, mode, space, value is None, invis=_invis)
        if pa >= self.iopage_base:
            return self.ub.wordRW(pa & self.cpu.IOPAGE_MASK, value)
        else:
            return self.cpu.physRW(pa, value)

    def byteRW(self, vaddr, value=None, /, mode=None, space=ISPACE):
        """Read/write a byte at virtual address vaddr.

        If value is None, perform a read and return an 8-bit value
        If value is not None, perform a write; return None.
        """

        pa = self.v2p(vaddr, mode, space, value is None)

        # Physical memory is represented as an array of 16-bit word
        # values, and byte operations are synthesized from that in
        # the obvious manner.
        #
        # However, the UNIBUS is different. At the physical electrical
        # signal level, the UNIBUS cannot perform byte reads, but CAN
        # perform byte writes.
        #
        # Given that - any byte read is synthesized from corresponding
        # word read operations, I/O or physical as appropriate.
        #
        # But byte write operations are dispatched as byte operations
        # to the unibus, while still being synthesized here for memory.

        odd = (pa & 1)

        if value is None:
            # *** READ ***
            #
            # Synthesized from containing word in the obvious way.
            # Note little-endianness.

            pa &= ~1
            if pa >= self.iopage_base:
                wv = self.ub.wordRW(pa & self.cpu.IOPAGE_MASK)
            else:
                wv = self.cpu.physRW(pa)
            return ((wv >> 8) if odd else wv) & 0o377
        else:
            # *** WRITE ***

            # This sanity check should be taken out eventually
            if (value & 0xFF) != value:
                raise ValueError(f"{value} is out of range")

            # I/O byte writes are handled by Unibus;
            # Memory byte writes are synthesized.

            if pa >= self.iopage_base:
                return self.ub.byteRW(pa & self.cpu.IOPAGE_MASK, value)
            else:
                wv = self.cpu.physRW(pa & ~1)
                if odd:
                    wv = (wv & 0o377) | (value << 8)
                else:
                    wv = (wv & 0o177400) | value
                self.cpu.physRW(pa & ~1, wv)
            return None

    # because faults will change MMR registers and CPUERR... this exists
    # so debugging/logging tools (e.g., machinestate()) can access
    # memory without disturbing MMU/cpu state if a fault is caused.
    def _invisread(self, a):
        """For debugging; read invisibly w/out modifying any MMU/CPU state."""
        return self.wordRW(a, _invis=True)

    def wordRW_KD(self, a, v=None, /):
        """Convenienence; version of wordRW for kernel/dspace."""
        return self.wordRW(a, v, mode=self.cpu.KERNEL, space=self.DSPACE)

    def _raisetrap(self, trapflag, vaddr, xkey, invis):
        """Raise an MMU trap. Commits regmods and updates reason in MMR0."""
        if not invis:
            self._MMR1commit()
            self.MMR0 |= (trapflag |
                          xkey.segno << 1 |     # bits <3:1>
                          xkey.space << 4 |     # bit 4
                          xkey.mode << 5)       # bits <6:5>

        # XXX gotta figure out how to set this for Odd Addresses and
        # T bit conditions, but otherwise Bit 7 is not set. From handbook:
        #     Bit 7 indicates that the current instruction has·been completed.
        #     It will be set to a during T bit, Parity, Odd Address, and
        #     Time Out traps and interrupts. Bit 7 is Read-Only (it cannot
        #     be written). It is initialized to a 1. Note that EMT, TRAP,
        #     BPT, and lOT do not set bit 7.
        raise PDPTraps.MMU()

    # handy for logging / debugging
    def scstr(self):
        """Return a string representation of the segment cache."""

        s = ""
        for xkey, v in self.segcache.items():
            ms = "KS!U"[xkey.mode]
            ds = "ID"[xkey.space]
            s += f"{oct(xkey.segno << 13)}:{ms}{ds}{xkey.reading} :"
            s += f" {oct(v[0])}\n"
        return s
