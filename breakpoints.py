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
# Custom Breakpoint objects can contain state, and anything that can
# be implemented as a per-instruction-boundary test is now possible.
# The breakpoint objects can also store logs of relevant state information
# (see, for example, the Lookback mixin).
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
    # the base Breakpoint class: a null breakpoint that never fires
    def __call__(self, pdp):
        return False


class StepsBreakpoint(Breakpoint):
    def __init__(self, *args, steps, **kwargs):
        self.togo = steps

    def __call__(self, pdp):
        self.togo -= 1
        return self.togo == 0


class PCBreakpoint(Breakpoint):
    def __init__(self, *, stoppc, stopmode=None):
        self.stoppc = stoppc
        self.stopmode = stopmode

    def __call__(self, pdp):
        return pdp.r[pdp.PC] == self.stoppc and (
            self.stopmode is None or pdp.psw_curmode == self.stopmode)


# Fire on the Nth occurrence of the given breakpoint
class NthBreakpoint(Breakpoint):
    def __init__(self, bp, nth, /, *args, **kwargs):
        self.__nth = self.__count = nth
        self.__bp = bp

    def __call__(self, pdp):
        if self.__bp(pdp):
            self.__count -= 1
        return self.__count == 0


# Add lookback state to a given breakpoint.
# ALTERNATIVELY, can be used entirely by itself (bp=None), and will provide
# lookback if the run() loop terminates for any reason (e.g., a HALT).
#
class Lookback(Breakpoint):

    def __init__(self, bp=None, /, *args, lookbacks=100, **kwargs):
        self.__backstates = collections.deque([], lookbacks)
        self.__bp = bp or (lambda pdp: False)

    def __call__(self, pdp):
        self.__backstates.append(pdp.machinestate())
        return self.__bp(pdp)

    @property
    def states(self):
        return list(self.__backstates)


class MultiBreakpoint(Breakpoint):
    # a breakpoint that fires if any of the contained breakpoints fire

    def __init__(self, bp0, /, *bps, testall=True):
        self.testall = testall
        self.bkpts = [bp0] + list(bps)

    def __call__(self, pdp):
        stophere = False
        for bp in self.bkpts:
            if bp(pdp):
                stophere = True
                if not self.testall:
                    break
        return stophere
