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


import collections

#
# BREAKPOINT FRAMEWORK
#
# Arguably this is more complex than necessary; it could have been
# (and, indeed, was originally) built into the pdp.run() method as a simple
# set of arguments for stepping N steps or stopping at a given PC and
# nothing fancier than that could be specified.
#
# However, this generalization now allows arbitrary breakpoint processing.
# The run() method will call a breakpoint after each instruction and
# pass in the machine object plus information, 'XInfo', containing
# important internal execution state of the machine:

XInfo = collections.namedtuple('XInfo', ['PC', 'instruction'])

# The Breakpoint object can use that information to cause a breakpoint.
# Custom Breakpoint objects can contain state, and anything that can
# be implemented as a per-instruction-boundary test is now possible.
# The breakpoint objects can also store logs of relevant state information
# (see, for example, Lookback).
#
# CAUTION regarding breakpoint state data and object re-use.  If a
# breakpoint object is created once but supplied to multiple pdp.run()
# method calls, it (obviously?) will carry its previous state forward.
# It is not initialized anew each call to pdp.run(). Thus, for example,
# if it is a "run N steps" breakpoint and has partially (or completely)
# counted down, the count does not start over with the second call.
# This may or may not be what was desired. Code accordingly. Note that
# run_steps() and run_until() create a new breakpoint object every time
# and avoid this issue that way.
#


class Breakpoint:
    """Base Breakpoint - a null breakpoint that never fires."""
    def __call__(self, pdp, xinfo):
        return False


class StepsBreakpoint(Breakpoint):
    """Break after the given number of instructions."""

    def __init__(self, steps):
        """steps is the number of instructions until breakpoint."""
        if steps < 1:
            raise ValueError(f"steps ({steps}) must be at least 1")
        self.togo = steps

    def __call__(self, pdp, xinfo):
        self.togo -= 1
        return self.togo == 0


class PCBreakpoint(Breakpoint):
    """Break at the given PC"""

    def __init__(self, *, stoppc, stopmode=None):
        """stoppc is the pc location to break at.
           optional stopmode restricts the breakpoint to the given
           mode (0, 1, 3 for Kernel/Supervisor/User)
        """
        self.stoppc = stoppc
        self.stopmf = lambda m: (stopmode is None) or (m == stopmode)

    def __call__(self, pdp, xinfo):
        # NOTE: xinfo.PC is the PC of the instruction just completed,
        #       whereas pdp.r[pdp.PC] is the PC of the next instruction
        #       to be executed.
        return pdp.r[pdp.PC] == self.stoppc and self.stopmf(pdp.psw_curmode)


# Fire on the Nth occurrence of the given breakpoint
class NthBreakpoint(Breakpoint):
    """Break after the nth occurrence of a given breakpoint"""

    def __init__(self, bp, nth, /):
        """Break when 'bp' has occurred 'nth' times."""
        if nth < 1:
            raise ValueError(f"nth ({nth}) must be at least 1")
        self.__nth = self.__count = nth
        self.__bp = bp

    def __call__(self, pdp, xinfo):
        if self.__bp(pdp, xinfo):
            self.__count -= 1
        return self.__count == 0


# Add lookback state to a given breakpoint.
# ALTERNATIVELY, can be used entirely by itself (bp=None), and will provide
# lookback if the run() loop terminates for any reason (e.g., a HALT).
#
class Lookback(Breakpoint):
    """Records machine state after each instruction."""

    def __init__(self, bp=None, /, lookbacks=100):
        """Augments breakpoint 'bp' with a lookback record of machine state.

        'lookbacks' (default == 100) is how far back state is kept. The
        record is FIFO (i.e., the last N states are kept)

        See states property for obtaining the lookback record.
        """
        self.__backstates = collections.deque([], lookbacks)
        self.__bp = bp or (lambda pdp, xinfo: False)

    def __call__(self, pdp, xinfo):
        self.__backstates.append((xinfo, pdp.machinestate()))
        return self.__bp(pdp, xinfo)

    @property
    def states(self):
        """Return the lookback states."""
        return list(self.__backstates)


class MultiBreakpoint(Breakpoint):
    """A Breakpoint that fires if any of the contained breakpoints fire."""

    def __init__(self, bp0, /, *bps, testall=True):
        """MultiBreakpoint(bp0, *bps, testall=True)

        If testall is true (the default) all breakpoints are tested each
        time no matter what. This is generally desired because breakpoint
        tests often have side effects (e.g., a "steps until fires" counter
        is decremented, or log entries are made).

        If not testall, breakpoints after one that fires are not tested.
        """

        self.testall = testall
        self.bkpts = [bp0] + list(bps)

    def __call__(self, pdp, xinfo):
        stophere = False
        for bp in self.bkpts:
            if bp(pdp, xinfo):
                stophere = True
                if not self.testall:
                    break
        return stophere


# Add instruction logging to a given breakpoint.
# To use: create the underlying breakpoint and then pass it into this:
#      bp = Logger(SomeOtherBreakpoint(other-args))
#
# If used with no underlying breakpoint, just logs instructions.
#      bp = Logger()
#
class Logger(Breakpoint):
    """Log every instruction. Caution: LARGE logs, slow performance."""

    def __init__(self, bp=None, /, *, logger=None):
        self.__bp = bp or (lambda pdp, info: False)
        self.__logger = logger

    def __call__(self, pdp, xinfo):
        if self.__logger is None:
            self.__logger = pdp.logger

        m_s = "KS!U"[pdp.psw_curmode]
        pc_s = oct(xinfo.PC)
        inst = xinfo.instruction
        inst_s = "None" if inst is None else oct(inst)
        self.__logger.debug(f"{pc_s}/{m_s} :: {inst_s}")
        return self.__bp(pdp, xinfo)


# Useful for ensuring the emulation implementation isn't broken in
# some way - making sure all values in physical memory are 16 bits (only).
# This will run an expensive check every nth instructions.
# Obviously, if bogus values are found it will be nearly impossible to
# figure out HOW they got in there; put some checks back into mmu word/byteRW
# functions as necessary to debug that (they are removed for performance).

class _MemChecker(Breakpoint):
    def __init__(self, nth):
        self.check_every = nth
        self.togo = nth

    def __call__(self, pdp, xinfo):
        self.togo -= 1
        if self.togo == 0:
            self.check16(pdp)
            self.togo = self.check_every
        return False

    def check16(self, pdp):
        """Verifies no illegal values ended up in physical memory or regs."""
        pdp.logger.debug("check16 checking physical memory")
        for a, w in enumerate(pdp.physmem):
            if w < 0 or w > 65535:
                raise ValueError(f"Illegal physmem value {w} @ {oct(a<<1)}")
        for r, w in enumerate(pdp.r):
            if w < 0 or w > 65535:
                raise ValueError(f"Illegal reg value {w} @ {r}")
