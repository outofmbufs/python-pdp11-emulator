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


import logging
from types import SimpleNamespace

from pdptraps import PDPTrap, PDPTraps
from mmu import MemoryMgmt
from unibus import UNIBUS, UNIBUS_1170, BusCycle
from breakpoints import StepsBreakpoint, PCBreakpoint, XInfo

from op4 import op4_dispatch_table


# A note about the various opNxxx files:
#
# Conceptually all of those are part of the PDP11 class. But having one
# monolithic/large class file seemed less than ideal. Python does not
# allow multiple files for a single class.
#
# Was there a better way? Just give in and have one huge file??
#
# The opcode parsing/dispatch starts with the top 4 bits of the opcode;
# thus the names "op4" and "op4_dispatch_table". Further decoding from
# there is as defined by the pdp11 operation code encoding tree.
#
# A note about octal
#
# Octal notation is everywhere in the world of the PDP-11; it especially
# pervades the manuals when talking about I/O page addresses. The layout
# of special purpose CPU registers (e.g., MMU registers and the like)
# is often most sensical viewed as octal. This is especially true of
# instruction format decoding, where for example the MOV instruction
# is: 0o01<src><dst> where <src> is two octal digits (3 bits mode, 3 bits
# register number) and so is <dst>.
#
# Thus MOV R2,R3 is 0o010203 ... no one wants to think of that as 0x1083
#
# Given that octal therefore appears all over in low-level processor detail,
# it seems to make the most sense to just go with it, well, (almost)
# everywhere. Thus, for example, "MASK16 = 0o177777" not "MASK16 = 0xFFFF"
#
# Complaints about this decision should be written on the back
# of an eight dollar bill and deposited appropriately.
#


class PDP11:

    # Architectural constants, generally common across the whole family

    SIGN8 = 0o200          # sign bit mask for 8 bits
    SIGN16 = 0o100000      # sign bit mask for 16 bits
    SIGN32 = 0x80000000    # for 32 bits; no one wants to see this in octal

    # index this by opsize (1 or 2) to get corresponding sign bit mask
    SIGN816 = (None, SIGN8, SIGN16)

    MASK8 = 0o377
    MASK16 = 0o177777
    MASK32 = 0xFFFFFFFF

    # index this by opsize (1 or 2) to get corresponding byte/word mask
    MASK816 = (None, MASK8, MASK16)

    # I/O page size is 8K (bytes), and equivalent mask
    IOPAGE_SIZE = 8192       # bytes
    IOPAGE_MASK = IOPAGE_SIZE - 1

    UNIBUS_MASK = 0o777777   # 18 bits
    UNIBUS_SIZE = UNIBUS_MASK + 1

    # General register(s) block(s), relative to I/O page base
    IOPAGE_REGSETS_OFFS = 0o17700
    IOPAGE_REGSET_SIZE = 0o10       # 11/70 overrides w/more registers

    # PSW modes, though not all processors implement all
    KERNEL = 0           # 00 in PSW bits
    SUPERVISOR = 1       # 01 in PSW bits
    UNDEFINED_MODE = 2   # 10 is undefined and will cause a trap
    USER = 3             # 11 in PSW bits

    # sometimes nice to use these for clarity; r6 == SP, r7 == PC
    SP = 6
    PC = 7

    # the processor status word I/O page offset
    PS_OFFS = 0o17776

    # the stack limit register I/O page offset
    STACKLIM_OFFS = 0o17774

    # the console switches (read) and LEDs (write)
    SWLEDS_OFFS = 0o17570

    # Microprogram break register - some diagnostic programs look for this
    MICROPROG_BREAK_REG = 0o17770

    # this is a superb hack for controlling the logging level for debug
    # this is in the unibus address range reserved for "testers" -- not
    # sure what that really is but this is as good a place for it as any
    LOGGING_OFFS = 0o17000

    # Lower Size Register (memory size)
    LOWERSIZE_OFFS = 0o17760

    # Upper Size Register (memory size) ... ALWAYS ZERO
    UPPERSIZE_OFFS = 0o17762

    # System ID register
    SYSTEMID_OFFS = 0o17764

    # the CPU error register and some useful bit values
    CPUERROR_OFFS = 0o17766

    # not an Enum because ... need to do bitwise efficiently.
    CPUERR_BITS = SimpleNamespace(
        REDZONE=0o004, YELLOW=0o010, UNIBUS_TIMEOUT=0o020,
        NXM=0o040, ODDADDR=0o100, ILLHALT=0o200)

    # halt types. These are not architectural, but are helpful to see.
    # They go into self.halted as "truthy" values
    HALTED_INST = 1          # halt instruction
    HALTED_VECTORS = 2       # vectors not mapped into kernel dataspace (!!)
    HALTED_STACK = 3         # fatal kernel stack condition

    # "straps" are synchronous traps. They are trap operations that occur
    # AFTER an instruction has completely executed. Examples include the
    # stack-limit violation traps, and the MMU's "management trap" which
    # allows the OS to monitor page usage without a full-on page fault.
    # [ see the distinction between "traps" and "aborts" in the MMU ]
    #
    # STRAPBITS are set by the emulation code when their condition obtains
    # and a trap should be generated at the completion of the instruction.
    # If multiple are requested in a single instruction, only the highest
    # priority will fire. These STRAPBITS values are an implementation
    # detail (presumably the PDP11 microarchitecture does something similar).
    # They are never seen outside the processor implementation.
    #
    # NOTE: mid-instruction aborts essentially are the same as a strap, but
    # abort an instruction midway rather than letting it continue. They
    # are implemented by raising a PDPTrap exception, which is caught at
    # the instruction loop top-level, and then turned into the same thing
    # as a strap -- just one that gets there mid-instruction (and highest
    # priority). See HIGHEST_ABORTTRAP in STRAPBITS and the try/except
    # in the main instruction processing loop.

    # there is no significance to the specific values, other than
    # they must be single bits and they must sort in this order
    STRAPBITS = SimpleNamespace(

        HIGHEST_ABORTTRAP=0o100000,    # absolutely MUST be the highest
        MEMMGT=0o010000,
        YELLOW=0o004000,
        PIR=0o000040)

    # this is just a handy thing to have, and this is as good a place
    # for it as anywhere else
    @staticmethod
    def u16add(a, b):
        return (a + b) & 0o177777

    def __init__(self, *,
                 physmem=None,     # default will be 512KB
                 unibus=None,      # subclasses may want to supply variant
                 logger="pdp11", loglevel='WARNING'):

        # logging is enabled by default and will go to
        # a file logger + ".log" (i.e., "pdp11.log" by default).
        # If logging is not a str instance it will just be used as is.
        # (so logging can be configured by caller that way)

        try:
            logfname = logger + ".log"
        except TypeError:
            self.logger = logger
        else:
            loglevel = logging.getLevelNamesMapping().get(loglevel, loglevel)
            logger = logging.getLogger(logger)
            if not logger.hasHandlers():     # XXX is this the right/best way?
                logger.propagate = False

                logger.setLevel(loglevel)
                logger_fh = logging.FileHandler(logfname)
                formatter = logging.Formatter(
                    '%(name)s.%(levelname)s[%(asctime)s]: %(message)s',
                    datefmt='%H%M%S')
                logger_fh.setFormatter(formatter)
                logger.addHandler(logger_fh)
            self.logger = logger

        self.logger.info(f"{self.__class__.__name__} started;"
                         f" Logging level={logging.getLevelName(loglevel)}.")

        # registers but usually get overridden by model-specific subclass
        self.r = [0] * 8
        self.ub = unibus(self) if unibus else UNIBUS(self)
        self.mmu = MemoryMgmt(self)

        # default physical memory is 256K WORDS (512KB)
        self.physmem = physmem or ([0] * (256*1024))

        # >>5 because len gives words not bytes, and -1 because manual says:
        #     defined to indicate the last addressable block of 32 words in
        #     memory (bit 0 is equivalent to bit 6 of the Physical Address).
        self.lowersize = (len(self.physmem) >> 5) - 1

        # The 16-bit view of the PSW is synthesized when read; the
        # essential parts of it are split out internally like this:
        self.psw_curmode = self.KERNEL
        self.psw_prevmode = self.KERNEL
        self.psw_regset = 0          # this is not in all processors
        self.psw_pri = 7
        self.psw_trap = 0
        self.psw_n = 0
        self.psw_z = 0
        self.psw_v = 0
        self.psw_c = 0

        # some attributes ("registers") that appear in I/O page
        for attrname, offs in (('psw', self.PS_OFFS),
                               ('stack_limit_register', self.STACKLIM_OFFS),
                               ('swleds', self.SWLEDS_OFFS),
                               ('breakreg', self.MICROPROG_BREAK_REG),
                               ('lowersize', self.LOWERSIZE_OFFS),
                               ('uppersize', self.UPPERSIZE_OFFS),
                               ('systemID', self.SYSTEMID_OFFS),
                               ('error_register', self.CPUERROR_OFFS),
                               ('logging_hack', self.LOGGING_OFFS)):
            self.ub.register_simpleattr(self, attrname, offs)

        # console switches (read) and blinken lights (write)
        self.swleds = 0
        self.error_register = 0    # CPU Error register per handbook
        self.breakreg = 0          # microprogram break register (diags)

        # NOTE: The cold machine starts out in stack limit violation.
        # (stack pointer = 0). However, the limit semantics only apply
        # during explicit stack push operations such as: mov foo,-(sp)
        # or implicit pushes during traps/interrupts.
        # Programs such as boot-loaders running from cold-start conditions
        # should establish a valid stack early in their instruction sequence.
        self.stack_limit_register = 0

        # straps: keeps track of requests for synchronous traps
        # during an instruction. Note that only one will really happen,
        # whichever is the highest priority, though some might persist
        # and recur
        #         - stack limit
        #         - mmu management traps (note: these are not aborts)
        #           ... others?
        #
        # _strapsclear contains straps to clear during/after processing
        self.straps = 0
        self._strapsclear = 0

        # start off in halted state until .run() happens
        self.halted = True

        # device instances; see add_device
        self.devices = {}

    def physRW(self, physaddr, value=None):
        """like MMU.wordRW but takes physical addresses."""

        if (physaddr & 1):
            raise PDPTraps.AddressError(cpuerr=self.CPUERR_BITS.ODDADDR)

        physaddr >>= 1          # physical mem is an array of WORDs
        try:
            if value is None:           # i.e., reading
                return self.physmem[physaddr]
            else:
                # sanity check should be taken out eventually
                if (value & 0xFFFF) != value:
                    raise ValueError(f"{value} is out of range")
                self.physmem[physaddr] = value
                return value            # generally ignored
        except IndexError:
            raise PDPTraps.AddressError(
                cpuerr=self.CPUERR_BITS.NXM) from None

    def physRW_N(self, physaddr, nwords, words=None):
        """Like physRW but for nwords at a time."""

        if (physaddr & 1):
            raise PDPTraps.AddressError(cpuerr=self.CPUERR_BITS.ODDADDR)
        physaddr >>= 1          # physical mem is an array of WORDs

        try:
            if words is None:
                return self.physmem[physaddr:physaddr+nwords]
            else:
                self.physmem[physaddr:physaddr+nwords] = words
        except IndexError:
            raise PDPTraps.AddressError(
                cpuerr=self.CPUERR_BITS.NXM) from None

    # "associate" an emulated device
    #
    # Typically a device is instantiated by passing the unibus
    # attribute ('ub') of a PDP11 instance to its __init__ function:
    #
    #       # p is a PDP11 instance
    #       # XYZ11 is a device class
    #
    #       device_instance = XYZ11(p.ub)
    #
    # The device __init__ will typically use ub.register to connect up
    # to its UNIBUS addresses. That's all that needs to happen.
    #
    # Nevertheless, on general principles, it seems like the pdp instance
    # should "know" about its devices, so ... this method.
    #
    # The typical code sequence would be:
    #        p = PDP1170()
    #        p.associate_device(XY11(p.ub), 'XY')
    #        p.associate_device(AB11(p.ub), 'AB')
    # etc., combining instantiation ("XY11(p.ub)") and name association.
    #
    # HOWEVER, note that this works just as well:
    #        p = PDP1170()
    #        _ = XY11(p.ub)
    #        _ = AB11(p.ub)
    # the devices just won't be discoverable via the devices attribute.
    # Device discovery via the devices attribute is not "architectural",
    # but may be useful for some test programs or other introspection.
    #
    def associate_device(self, dev, device_name=None, /):
        """Associate the device instance 'dev' with 'device_name'

        If device_name is None then  __class__.__name__ is used.
        """
        if device_name is None:
            device_name = dev.__class__.__name__

        try:
            self.devices[device_name].append(dev)
        except KeyError:
            self.devices[device_name] = [dev]

    # this the heart of all things related to 6-bit instruction operands.
    #    If value is not given this will be a read
    #    If value is given and not None, this will be a write
    #
    # If justEA is True, the address that would be used to access the
    # operand is returned, vs the operand itself. This is not valid for
    # register direct. See JMP/JSR for examples of how/when this happens.
    #
    # If rmw is True, this will return a tuple:
    #        value, extendedB6
    # otherwise it returns just the value (read, or written)

    def operandx(self, b6, value=None, /, *,
                 opsize=2, altmode=None, altspace=None,
                 rmw=False, justEA=False):
        """Parse a 6-bit operand and read it (value is None) or write it.

        By default the value (read, or written) is returned.
        Some instructions need the operand address, not the value
        (JSR is the best example of this). Specify justEA=True for that.
        Note that justEA=True will trap for register-direct mode.

        Some opcodes use a single addressing mode twice:
            val = read the operand
            do something to val (i.e., INC)
            write modified val to the operand
        The problem is side-effects, which are executed here for
        modes like (Rn)+ (pc-relative is also a problem)

        For this case, specify rmw=True ("read/modify/write") on the read
        call (value=None) in which case the return value will be a tuple:
                (value, EXTENDED_B6)

        and the EXTENDED_B6 should be passed back in as the "b6" for the
        write call. Callers should treat it as opaque. It is encoded to
        allow the same operand to be re-used but without side effects
        the second time.
        """

        # EXTENDED_B6 ENCODING -- it is a 32-bit value:
        #   Bits 31-24 = 0xFF or 0x00
        #     If 00: The entire value is just a native b6. The low 6 bits
        #            are a pdp11 b6 value and all other bits are zero.
        #     If FF:
        #        bits 23-8: 16-bit effective address
        #        bits 7-6: mmu.ISPACE or mmu.DSPACE value
        #        bits 5-0: 0o47 which is an illegal b6; just to avoid
        #                       looking like an optimizable case and
        #                       to catch bugs if somehow used
        #

        # NOTE: real PDP-11 implementations vary in corner cases.
        # For example:
        #     MOV R5,-(R5)
        # what value gets stored? This turns out to vary. In fact, DEC
        # documented the variations across processors. FWIW, the MACRO-11
        # assembler generates warnings for such cases.  Given all that,
        # the assumption here is that getting those tricky semantics
        # "correct to the specific processor variations" is unnecessary.

        # optimize addr mode 0 - register. 8 or 16 bits.
        # Note that in all READ cases b6 will be the newb6 (reusable)
        if (b6 & 0o70) == 0:
            if justEA:
                raise PDPTraps.AddressError

            match b6 & 0o07, value, opsize:
                case Rn, None, 2:
                    value = self.r[Rn]
                case Rn, wv, 2:
                    self.r[Rn] = wv
                case Rn, None, 1:
                    value = self.r[Rn] & 0o377
                case Rn, bv, 1:
                    # NOTE: The MOVB instruction has different semantics
                    #       than this; it is coded explicitly in op11_movb()
                    self.r[Rn] &= 0o177400
                    self.r[Rn] |= (bv & 0o377)
            return (value, b6) if rmw else value

        # harder cases
        autocrement = 0               # increment/decrement
        space = self.mmu.DSPACE       # gets changed in various cases
        extendedb6 = b6               # will be altered as necessary

        match b6 & 0xFF_0000_00, (b6 & 0o70), (b6 & 0o07):
            # (Rn) -- register deferred
            case 0, 0o10, Rn:
                addr = self.r[Rn]
                if Rn == 7:
                    space = self.mmu.ISPACE

            # both autoincrement addrmodes: (Rn)+ and @(Rn)+
            case 0, 0o20 | 0o30 as addrmode,  Rn:
                addr = self.r[Rn]
                if Rn == self.PC:
                    space = self.mmu.ISPACE
                    autocrement = 2     # regardless of opsize
                elif Rn == self.SP:
                    autocrement = 2     # regardless of opsize
                else:
                    autocrement = opsize

                if addrmode == 0o30:
                    addr = self.mmu.wordRW(addr, space=space)
                    space = self.mmu.DSPACE
                    autocrement = 2     # regardless of opsize used above
                extendedb6 = None       # force update below

            # both autodecrement addrmode, PC - NOPE.
            case 0, 0o40 | 0o50, 7:
                # ... did the pdp11 fault on this?
                raise PDPTraps.ReservedInstruction

            # both autodecrement addrmodes, not PC
            # note that bytes and -(SP) still decrement by 2
            case 0, 0o40 | 0o50 as addrmode, Rn:
                if Rn == self.SP:
                    autocrement = -2
                elif addrmode == 0o50:
                    autocrement = -2
                else:
                    autocrement = -opsize
                extendedb6 = None       # force update below
                addr = self.u16add(self.r[Rn], autocrement)
                if addrmode == 0o50:
                    addr = self.mmu.wordRW(addr, space=self.mmu.DSPACE)

            # X(Rn) and @X(Rn)
            case 0, (0o60 | 0o70) as addrmode, Rn:
                x = self.mmu.wordRW(self.r[self.PC], space=self.mmu.ISPACE)
                self.r[self.PC] = self.u16add(self.r[self.PC], 2)
                addr = self.u16add(self.r[Rn], x)
                extendedb6 = None       # force update below
                if addrmode == 0o70:
                    addr = self.mmu.wordRW(addr, space=self.mmu.DSPACE)

            case 0xFF_0000_00, _, _:
                # the address was shifted up 8 bits (to get it away
                # from the mode-0 optimization tests) and the space
                # was encoded shifted up 6 bits (again, get away from mode 0)
                addr = (b6 >> 8) & 0xFFFF
                space = (b6 >> 6) & 3
            case _:                   # should be unreachable
                raise TypeError("internal error")

        if autocrement != 0:
            # the autoincrement/decrements have to be recorded into the MMU
            # for instruction recovery if there is a page error.
            self.mmu.MMR1mod(((autocrement & 0o37) << 3) | Rn)
            self.r[Rn] = self.u16add(self.r[Rn], autocrement)
            if Rn == self.SP and autocrement < 0:
                self.redyellowcheck()   # may raise a RED zone exception

        if rmw and (value is None) and (extendedb6 is None):
            extendedb6 = 0xFF_0000_27 | (addr << 8) | (space << 6)

        # use alternate space (e.g. forced ISPACE) if requested.
        if altspace is not None:
            space = altspace

        if justEA:
            val = addr
        elif opsize == 2:
            val = self.mmu.wordRW(addr, value, mode=altmode, space=space)
        else:
            val = self.mmu.byteRW(addr, value, mode=altmode, space=space)

        return (val, extendedb6) if rmw else val

    # convenience, creates a breakpoint function to stop after N steps
    def run_steps(self, steps, *args, **kwargs):
        bpt = StepsBreakpoint(steps=steps)
        return self.run(*args, breakpoint=bpt, **kwargs)

    # convenience, creates a breakpoint to stop at the given pc,
    # optionally limited to a specific mode (KERNEL, USER, SUPERVISOR)
    def run_until(self, *args, stoppc, stopmode=None, **kwargs):
        """Run processor with breakpoint at 'stoppc'.

        if stopmode is None (default), stop at the stoppc in any mode.
        Otherwise, only stop if mode is stopmode.
        """
        bpt = PCBreakpoint(stoppc=stoppc, stopmode=stopmode)
        return self.run(*args, breakpoint=bpt, **kwargs)

    def run(self, *, pc=None, breakpoint=None):
        """The CPU main loop - run the machine!

        If pc is None (default) execution begins at the current pc; otherwise
        the pc is set to the given value first.

        If breakpoint is not None (default), it is expected to be a callable.
        It will be invoked after every instruction and if it returns True
        then a breakpoint will occur (the run() method returns).
        """

        if pc is not None:
            self.r[self.PC] = pc

        # some shorthands for convenience
        interrupt_mgr = self.ub.intmgr
        mmu = self.mmu

        abort_trap = None           # a mid-instruction abort (vs strap)
        self.halted = False

        # NOTE WELL: everything in this loop is per-instruction overhead
        while not self.halted:

            # SUBTLETY: Trap handlers expect the PC to be 2 beyond the
            #    instruction causing the trap. Hence "+2 then execute"
            thisPC = self.r[self.PC]
            self.r[self.PC] = (thisPC + 2) & 0o177777  # "could" wrap

            mmu.MMR1_staged = 0     # see discussion in go_trap
            mmu.MMR2 = thisPC       # per handbook
            inst = None             # so bkpt can know if wordRW faulted

            try:
                inst = mmu.wordRW(thisPC)
                op4_dispatch_table[inst >> 12](self, inst)
            except PDPTrap as trap:
                abort_trap = trap
                self.straps |= self.STRAPBITS.HIGHEST_ABORTTRAP

            # pri order:abort traps (encoded as a strap), straps, interrupts
            if self.straps:
                self.go_trap(self.get_synchronous_trap(abort_trap))
            elif interrupt_mgr.pri_pending > self.psw_pri:
                self.go_trap(interrupt_mgr.get_pending(self.psw_pri))

            if breakpoint and breakpoint(self, XInfo(thisPC, inst)):
                break

        # fall through to here if self.halted or breakpoont
        if self.halted:
            reason = ".run -- HALTED: {}"
        else:
            reason = ".run -- breakpoint: {}"
        self.logger.info(reason.format(self.machinestate(fmt=oct)))

    def redyellowcheck(self):
        """stack limits: possibly sets YELLOW straps, or go RED."""

        # only applies to kernel stack operations
        if self.psw_curmode != self.KERNEL:
            return

        # Note special semantic of zero which means 0o400
        # (as defined by hardware book)
        lim = self.stack_limit_register or 0o400
        if self.r[self.SP] < lim:
            if not (self.straps & self.STRAPBITS.YELLOW):
                self.logger.info(f"YELLOW ZONE, {list(map(oct, self.r))}")

            # how about red?
            if self.r[self.SP] + 32 < lim:    # uh oh - below the yellow!
                # this is a red zone trap which is immediate
                # the stack pointer is set to location 4
                # and this trap is executed
                self.r[self.SP] = 4             # !! just enough room for...
                raise PDPTraps.AddressError(cpuerr=self.CPUERR_BITS.REDZONE)
            else:
                self.straps |= self.STRAPBITS.YELLOW

    def get_synchronous_trap(self, abort_trap):
        """Return a synchronous trap, or possibly None.

        For notational convenience in the instruction loop, the
        abort_trap argument, if not None, represents a mid-instruction
        abort which is the highest priority trap and it is just returned.
        The corresponding straps bit is cleared.

        After that, finds the highest priority strap if any, and returns it.
        """

        # as described above... this is how aborts work
        if self.straps & self.STRAPBITS.HIGHEST_ABORTTRAP:
            self.straps &= ~self.STRAPBITS.HIGHEST_ABORTTRAP
            return abort_trap

        # Synchronous traps are events that are caused by an instruction
        # but happen AFTER the instruction completes. The handbook shows
        # eight of them, in this priority order (high to low)
        #
        #   HIGHEST -- Parity error
        #              Memory Management violation
        #              Stack Limit Yellow
        #              Power Failure
        #              Floating Point
        #              Program Interrupt Request
        #              Bus Request
        #   LOWEST     Trace Trap
        #
        # If there are multiple, only the highest priority will fire,
        # though some types of them are persistent (in their root cause)
        # and would therefore come back with the next instruction and
        # (potentially) fire there instead.

        # The stack limit yellow bit is a little different ... have
        # to also check for the red zone here.
        if self.straps & self.STRAPBITS.YELLOW:
            # at a minimum, it's a yellow zone fault
            self.error_register |= self.CPUERR_BITS.YELLOW

        # note that these are tested in priority order. With only two
        # cases here, if/elif seemed better than iterating a table
        if self.straps & self.STRAPBITS.MEMMGT:
            self.straps &= ~self.STRAPBITS.MEMMGT
            return PDPTraps.MMU()
        elif self.straps & self.STRAPBITS.YELLOW:   # red handled as an abort
            self._strapsclear = self.STRAPBITS.YELLOW
            return PDPTraps.AddressError(cpuerr=self.CPUERR_BITS.YELLOW)
        return None

    def go_trap(self, trap):
        """Control transfer for all types of traps, INCLUDING interrupts."""

        # it's convenient to allow trap to be None meaning "never mind"
        if trap is None:
            return

        self.logger.debug(f"TRAP: {trap}:\n{self.machinestate(fmt=oct)}")
        self.error_register |= trap.cpuerr

        # get the vector information -- always from KERNEL/DSPACE
        try:
            newpc = self.mmu.wordRW_KD(trap.vector)
            newps = self.mmu.wordRW_KD(trap.vector+2)
        except PDPTrap:
            # this is an egregious kernel programming error -- the vectors
            # are not mapped into KERNEL/DSPACE. It is a fatal halt.
            self.logger.info(f"Trap accessing trap vectors")
            self.halted = self.HALTED_VECTORS
            return

        # From the PDP11 processor book:
        #    The old PS and PC are then pushed onto the current stack
        #    as indicated by bits 15,14 of the new PS and the previous
        #    mode in effect is stored in bits 13,12 of the new PS.
        # Thus:

        # easiest to get the "previous" (currently current) mode this way:
        saved_curmode = self.psw_curmode
        saved_psw = self.psw

        # note: this (likely) switches SP and of course various psw_xxx fields
        self.psw = newps
        self.psw_prevmode = saved_curmode   # i.e., override newps<13:12>

        self._trappush(self.r[self.PC], saved_psw)

        # The error register records (accumulates) reasons (if given)
        self.error_register |= trap.cpuerr

        # if a strap is being processed, this is where it is reset
        # IMPORTANT/SUBTLE NOTE: This is after the stack pushes, so this
        # is how they avoid (re-)causing YELLOW while processing YELLOW
        self.straps &= ~self._strapsclear
        self._strapsclear = 0

        # alrighty then, can finally jump to the PC from the vector
        self.r[self.PC] = newpc

    @property
    def swleds(self):
        return 0                  # no switches implementation, yet

    @swleds.setter
    def swleds(self, v):          # writing to the lights is a no-op for now
        pass

    # technically not all -11's have stack limit support.
    # Meh, do it in base class anyway
    @property
    def stack_limit_register(self):
        return self._stklim

    @stack_limit_register.setter
    def stack_limit_register(self, v):

        # at __init__ time it's important to NOT indicate the need
        # for a stack check or else the first instruction executed
        # will fail the stack limit.
        #
        # Any other time, set the bit so the main instruction loop
        # will know it needs to examine the stack limit status.
        #
        # This could also have been fixed by initializing _stklim in
        # __init__ and not "stack_limit_register = 0" , or it could
        # have been fixed by slamming strapcheck back to false after that.
        # But this way ensures The Right Thing happens no matter what.
        # Performance is no issue in setting the stack limit obviously.
        self.logger.debug(
            f"setting stack limit to {oct(v)}, sp={oct(self.r[6])}")
        checkit = hasattr(self, '_stklim')
        self._stklim = v & 0o177400
        if checkit:
            self.redyellowcheck()

    def stackpush(self, w):
        self.r[6] = self.u16add(self.r[6], -2)
        # stacklimit checks only apply to the kernel and do not
        # apply when pushing the frame for a stacklimit fault (!)
        if self.psw_curmode == self.KERNEL:
            self.redyellowcheck()               # may raise a RED exception
        self.mmu.wordRW(self.r[6], w, space=self.mmu.DSPACE)

    def stackpop(self):
        w = self.mmu.wordRW(self.r[6], space=self.mmu.DSPACE)
        self.r[6] = self.u16add(self.r[6], 2)
        return w

    # this is special because stack limit checking is disabled
    # during pushes of trap/interrupt frames. Any other types of
    # traps during a stack trap push are a fatal CPU halt and represent
    # a serious kernel programming error (invalid kernel stack)
    def _trappush(self, pc, psw):
        try:
            # NOTE: The stack pointer is only modified if
            #       both of these succeed with no mmu/addressing traps
            self.mmu.wordRW_KD(self.u16add(self.r[self.SP], -2), psw)
            self.mmu.wordRW_KD(self.u16add(self.r[self.SP], -4), pc)
        except PDPTrap as e:
            # again this is a pretty egregious error it means the kernel
            # stack is not mapped, or the stack pointer is odd, or similar
            # very bad mistakes by the kernel code. It is a fatal halt
            self.logger.info(f"Trap ({e}) pushing trap frame onto stack")
            self.logger.info(f"Machine state: {self.machinestate(fmt=oct)}")
            self.logger.info("HALTING")
            self.halted = self.HALTED_STACK
        else:
            self.r[self.SP] = self.u16add(self.r[self.SP], -4)


class PDP1170(PDP11):

    # some 1170-specific values
    IOPAGE_REGSET_SIZE = 0o20         # 11/70 has two sets of registers

    def __init__(self, *, physmem=None, **kwargs):
        super().__init__(physmem=physmem, unibus=UNIBUS_1170, **kwargs)

        # there are two register files, though r6 and r7 are special
        self.registerfiles = [[0] * 8, [0] * 8]

        # There are four stack pointers, but only 3 are legal.
        # This can be indexed by self.KERNEL, self.SUPERVISOR, etc
        self.stackpointers = [0, 0, 0, 0]

        # The 16-bit view of the PSW is synthesized when read; the
        # essential parts of it are split out internally like this:
        self.psw_curmode = self.KERNEL
        self.psw_prevmode = self.KERNEL
        self.psw_regset = 0
        self.psw_pri = 7
        self.psw_trap = 0
        self.psw_n = 0
        self.psw_z = 0
        self.psw_v = 0
        self.psw_c = 0

        # self.r points to the current register set
        self.r = self.registerfiles[self.psw_regset]

        # how the registers appear in IOPAGE space
        self.ub.register(self._ioregsets,
                         self.IOPAGE_REGSETS_OFFS,
                         self.IOPAGE_REGSET_SIZE)

    @property
    def r_alt(self):
        """The other set of registers (the one that is not self.r)."""
        return self.registerfiles[1 - self.psw_regset]

    def _ioregsets(self, addr, cycle, /, *, value=None):
        # NOTE that the encoding of the register addresses is quite funky
        #      and includes ODD addresses (!!!)
        # [ addresses given relative to I/O page base ]
        #   REGISTER SET ZERO
        #      17700 : R0
        #      17701 : R1        -- this being at ODD address is not a typo!
        #      17702 : R2
        #      17703 : R3        -- not a typo
        #      17704 : R4
        #      17705 : R5        -- not a typo
        #      17706 : KERNEL SP
        #      17707 : PC
        #
        #   REGISTER SET ONE
        #      17710 : R0
        #      17711 : R1
        #      17712 : R2
        #      17713 : R3
        #      17714 : R4
        #      17715 : R5
        #      17716 : SUPERVISOR SP
        #      17717 : USER SP
        regset = addr & 0o10
        regnum = addr & 0o07

        # copy the stack pointer out of its r6 "cache" and dup the pc
        self._syncregs()

        #        regset             regnum       r/w
        match ((addr & 0o10) >> 3, addr & 0o07, cycle):
            case (0, 6, BusCycle.READ16):
                return self.stackpointers[self.KERNEL]
            case (0, 6, BusCycle.WRITE16):
                self.stackpointers[self.KERNEL] = value
            case (1, 6, BusCycle.READ16):
                return self.stackpointers[self.SUPERVISOR]
            case (1, 6, BusCycle.WRITE16):
                self.stackpointers[self.SUPERVISOR] = value
            case (1, 7, BusCycle.READ16):
                return self.stackpointers[self.USER]
            case (1, 7, BusCycle.WRITE16):
                self.stackpointers[self.USER] = value
            case (regset, regnum, BusCycle.READ16):
                return self.registerfiles[regset][regnum]
            case (regset, regnum, BusCycle.WRITE16):
                self.registerfiles[regset][regnum] = value
            case (regset, regnum, BusCycle.WRITE8):
                # ? is this legal.
                # Only the low-order byte would be addressible for even regs
                # (that live at even addresses) and only the high-order byte
                # would be addressible for odd registers (living at odd addrs).
                # Funky! Raising this trap seems a better idea.
                raise PDPTraps.AddressError(cpuerr=self.CPUERR_BITS.NXM)

        # if the stack pointer for the current mode was updated
        # then reestablish it as r[6]. Can just do this unconditionally
        # because syncregs copied out the active r[6] above
        self.r[6] = self.stackpointers[self.psw_curmode]

    def _syncregs(self):
        # When there is a register set change, a mode change, or when
        # the registers are being examined via their I/O addresses then
        # the "cached" stack pointer in R6 has to be synced up to its
        # real home, and the PC (R7) has to be duplicated into the other set.

        self.stackpointers[self.psw_curmode] = self.r[6]

        # sync the PC into the other register set
        self.r_alt[self.PC] = self.r[self.PC]

    @property
    def psw(self):
        # NOTE: to simplify/accelerate condition code handling during
        # instructions, the NZVC bits are broken out into individual
        # attributes, and are stored as truthy/falsey not necessarily
        # 1/0 or True/False.

        # so, to reconstitute NZVC bits ...
        NZVC = 0
        if self.psw_n:
            NZVC |= 0o10
        if self.psw_z:
            NZVC |= 0o04
        if self.psw_v:
            NZVC |= 0o02
        if self.psw_c:
            NZVC |= 0o01

        return (((self.psw_curmode & 3) << 14) |
                ((self.psw_prevmode & 3) << 12) |
                ((self.psw_regset & 1) << 11) |
                ((self.psw_pri & 7) << 5) |
                ((self.psw_trap & 1) << 4) |
                NZVC)

    # Write the ENTIRE processor word, without any privilege enforcement.
    # The lack of privilege enforcement is necessary because, e.g., that's
    # how traps get from user to kernel mode. Generally speaking, the
    # only way for user mode programs to modify the PSW is via its I/O
    # address, which (obviously) an OS should not put into user space.
    @psw.setter
    def psw(self, value):
        """Set entire PSW. NOTE: no privilege enforcement."""

        # could test if necessary but it's just easier to do this every time
        self._syncregs()           # in case any mode/regset changes

        # prevent UNDEFINED_MODE from entering the PSW (current mode)
        m = (value >> 14) & 3
        if m == self.UNDEFINED_MODE:
            raise PDPTraps.ReservedInstruction
        self.psw_curmode = m

        # prevent UNDEFINED_MODE from entering the PSW (previous mode)
        m = (value >> 12) & 3
        if m == self.UNDEFINED_MODE:
            raise PDPTraps.ReservedInstruction
        self.psw_prevmode = m

        prevregset = self.psw_regset
        self.psw_regset = (value >> 11) & 1

        newpri = (value >> 5) & 7
        self.psw_pri = newpri

        self.psw_trap = (value >> 4) & 1
        self.psw_n = (value >> 3) & 1
        self.psw_z = (value >> 2) & 1
        self.psw_v = (value >> 1) & 1
        self.psw_c = value & 1

        # set up the correct register file and install correct SP
        self.r = self.registerfiles[self.psw_regset]
        self.r[6] = self.stackpointers[self.psw_curmode]

        # the PC was already sync'd in syncregs()

    @property
    def logging_hack(self):
        return self.logger.level

    @logging_hack.setter
    def logging_hack(self, value):
        self.logger.setLevel(value)

    # this is convenient to have for debugging and logging
    def spsw(self, v=None):
        """Return string rep of a psw value."""
        if v is None:
            v = self.psw

        cm = (v >> 14) & 3
        pm = (v >> 12) & 3

        m2s = "KS!U"

        s = f"CM={m2s[cm]} PM={(m2s[pm])}"
        if v & 0o04000:
            s += " Rx=1"
        s += f" PRI={(v >> 5) & 0o07}"
        if v & 0o020:
            s += " T"
        if v & 0o017:
            s += " "
        if v & 0o010:
            s += "N"
        if v & 0o004:
            s += "Z"
        if v & 0o002:
            s += "V"
        if v & 0o001:
            s += "C"
        return s

    # logging/debugging convenience
    def machinestate(self, fmt=None):
        # machine state is arbitrarily collected into a dictionary:
        d = {}
        regnames = (* (f"R{i}" for i in range(6)), "SP", "PC")
        for i in range(8):
            d[regnames[i]] = self.r[i]

        d['PSW'] = self.psw

        # these are redundant but convenient to have broken out
        d['CURMODE'] = self.psw_curmode
        d['PREVMODE'] = self.psw_prevmode
        d['PRI'] = self.psw_pri

        for m in (0, 1, 3):
            d[("KSP", "SSP", "!X!", "USP")[m]] = self.stackpointers[m]

        for mmr in ('MMR0', 'MMR1', 'MMR2', 'MMR3'):
            d[mmr] = getattr(self.mmu, mmr)

        try:
            # _invisread so as not to disturb state (MMR/cpuerror regs etc)
            d['MMR2inst'] = self.mmu._invisread(self.mmu.MMR2)
        except PDPTrap as e:
            d['MMR2inst'] = e

        # For humans (e.g., if logging) it may be desired to have things
        # in octal or some other format. Eventually maybe "fmt" should
        # be more elaborate but as a start "if fmt is oct" then convert
        # everything into an octal string.
        if fmt is oct:
            for k, v in d.items():
                pfx = '0' if v > 7 else ''
                d[k] = pfx + oct(v)[2:]

        return d
