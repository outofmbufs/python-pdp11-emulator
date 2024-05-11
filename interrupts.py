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

from collections import namedtuple
import threading

from pdptraps import PDPTrap


# an interrupt is, at the cpu implementation level, just a flavor of trap.
class InterruptTrap(PDPTrap):
    def __init__(self, pri, vector):
        super().__init__()
        self.pri = pri
        self.vector = vector


# contains the details for a pending interrupt (see discussion)
PendingInterrupt = namedtuple(
    'PendingInterrupt', ('pri', 'vector', 'callback'))


# This is a delicious hack so that the console can force a processor halt.
# Notice the sneaky pri=8 (more than 0..7 in the architecture).
# This phony 'halt interrupt' is unmaskable and outside the architecture.
# Also note the (too clever?) use of the callback to invoke exit()
# Think of this as the equivalent of the HALT toggle on the physical panel

ProcessorHalt = PendingInterrupt(8, 0, exit)


# Interrupts are priority sorted by pri (duh), but (less obviously)
# two interrupts with the same pri are further priority sorted by
# vector, with lower vector being higher priority. This calculation
# encapsulates that into a single integer value, with the knowledge
# that no vectors can ever be anywhere near as high as 16384.

def _qpri(pdi):
    return (pdi.pri * 16384) + (16384 - pdi.vector)


# To cause an interrupt, a device creates a PendingInterrupt containing:
#       pri          -- priority
#     vector         -- the interrupt vector
#     callback       -- see discussion
#
# and then calls pend_interrupt() to get things going.
#
# The interrupt does not, of course, occur right away; it pends until
# the processor is willing to accept it. The processor only accepts
# interrupts at instruction boundaries, and even then only if the current
# processor priority level is below the interrupt pri.
#
# When those conditions occur, the interrupt is accepted by the processor
# (via a get_pending() method called from the processor). At that time the
# interrupt request is "granted" and the callback function, if any is
# provided, is invoked (from the cpu thread).
#
# The callback is invoked with no arguments and the return value is ignored.
# Use partial() or other python techniques if the callback function requires
# arguments for more context information (most will not).
#
# The purpose of this callback protocol is that some devices have internal
# operations they want to perform when the interrupt is acknowledged, not
# just when it is first made pending. Callbacks allow for that to happen.
# CAUTION: The callback obviously executes in a separate thread and
#          will be asynchronous to any device-internal threads.
#
# In the simplest/common cases where none of this is needed, the
# method simple_irq() bundles all this minutia up for the caller.

class InterruptManager:
    def __init__(self, cpu):
        self.pri_pending = 0
        self.requests = []
        self.condition = threading.Condition()
        self.logger = cpu.logger    # only thing needed from cpu

    def simple_irq(self, pri, vector):
        """Pend an interrupt at the given pri/vector."""
        self.pend_interrupt(PendingInterrupt(pri, vector, callback=None))

    def pend_interrupt(self, irq):
        """Pend a request for interrupt 'irq'."""
        with self.condition:
            # special case to accelerate zero-to-one common transition
            if not self.requests:
                self.requests = [irq]
                self.pri_pending = irq.pri
            else:
                # multiple identical requests are not pended
                # (it works this way in the hardware too of course --
                #  if a device has asserted the interrupt request line
                #  but that request hasn't been acknowledged/cleared by
                #  by the bus signal protocol yet, you can't assert the
                #  same interrupt line again ... it's already asserted)
                if irq not in self.requests:
                    self.requests = sorted(self.requests + [irq], key=_qpri)
                    self.pri_pending = self.requests[-1].pri
            self.condition.notify_all()

    def halt_toggle(self, msg=""):
        self.logger.info(f"HALT TOGGLE, {msg=}")
        self.pend_interrupt(ProcessorHalt)

    # called by the processor, to get one pending interrupt (if any).
    # An InterruptTrap with the highest priority is returned, IF it is
    # above the given processor priority. Else None.
    def get_pending(self, processor_pri):
        """Returns an InterruptTrap, or None."""
        with self.condition:
            try:
                if self.pri_pending > processor_pri:
                    irq = self.requests.pop()
                else:
                    return None
            except IndexError:
                return None
            else:
                if self.requests:
                    self.pri_pending = self.requests[-1].pri
                else:
                    self.pri_pending = 0

        if irq.callback:
            irq.callback()
        return InterruptTrap(irq.pri, irq.vector)

    def waitstate(self, processor_pri):
        """Sit idle until any interrupt happens."""
        with self.condition:
            if self.pri_pending > processor_pri:
                return
            self.condition.wait_for(lambda: self.pri_pending)


if __name__ == "__main__":
    import unittest
    from functools import partial

    class TestMethods(unittest.TestCase):
        def test__init__(self):
            IM = InterruptManager()

            # initial state starts with no pending interrupts
            self.assertEqual(IM.pri_pending, 0)

            # verify get_pending still "works" (returns None)
            self.assertEqual(IM.get_pending(0), None)

        def test_queue1(self):
            IM = InterruptManager()
            test_pri = 4      # arbitrary
            test_vec = 17     # arbitrary
            IM.simple_irq(test_pri, test_vec)
            self.assertEqual(IM.pri_pending, test_pri)
            iinfo = IM.get_pending(0)
            self.assertEqual(IM.pri_pending, 0)
            self.assertEqual(iinfo.pri, test_pri)
            self.assertEqual(iinfo.vector, test_vec)

        # support function for test cases, do a bunch of actions on an IM
        def _actions(self, IM, prog):
            cpupri = 0
            for action in prog:
                match action[0], action[1]:
                    case 'RQ', t:
                        IM.simple_irq(*t)
                    case 'PRI', cpupri:
                        pass
                    case 'GET', xt:
                        t = IM.get_pending(cpupri)
                        if t is None:
                            self.assertEqual(t, xt)
                        else:
                            xpri, xvec = xt
                            # If the vector position is a tuple then that
                            # means to accept anything in that tuple
                            try:
                                _ = (t.vector in xvec)
                            except TypeError:
                                pass
                            else:
                                xvec = t.vector      # i.e., it's ok
                            self.assertEqual(t.pri, xpri)
                            self.assertEqual(t.vector, xvec)
                    case 'CHK', pri:
                        self.assertEqual(IM.pri_pending, pri)

                    case _:
                        raise ValueError("bad action", action)

        def test_mixedops(self):
            testprogs = (
                # (ACTION, ACTION-INFO)
                (('RQ', (4, 44)),     # request IRQ 4
                 ('RQ', (5, 55)),     # request IRQ 5
                 ('GET', (5, 55)),    # get one, check that it is 5
                 ('CHK', 4),          # check that pri_pending is 4
                 ('RQ', (3, 33)),     # request IRQ 3
                 ('CHK', 4),          # check that pri_pending is 4
                 ('RQ', (6, 66)),     # request IRQ 6
                 ('CHK', 6),          # check that pri_pending is 6
                 ('GET', (6, 66)),    # get one, check that it is 6
                 ('CHK', 4),          # check that pri_pending is 6
                 ('GET', (4, 44)),    # get one, check that it is 4
                 ('CHK', 3),          # check that pri_pending is 3
                 ('GET', (3, 33)),    # get one, check that it is 3
                 ('CHK', 0),          # check that pri_pending is 0
                 ('GET', None),       # check that getting from empty works
                 ),

                # check priority filtering
                (('RQ', (4, 44)),           # request IRQ 4
                 ('RQ', (5, 55)),           # request IRQ 5
                 ('PRI', 7),                # spl7
                 ('GET', None),             # shouldn't see anything
                 ('PRI', 5),                # spl5
                 ('GET', None),             # still shouldn't see anything
                 ('RQ', (6, 66)),           # request IRQ 6
                 ('RQ', (7, 77)),           # request IRQ 7
                 ('RQ', (6, 666)),          # request IRQ 6
                 ('RQ', (7, 777)),          # request IRQ 7
                 ('PRI', 6),                # spl6
                 ('GET', (7, (77, 777))),   # should get one of these
                 ('GET', (7, (77, 777))),   # should get the other
                 ('GET', None),             # no more
                 ('PRI', 0),                # spl0
                 ('GET', (6, (66, 666))),   # should get one of these
                 ('GET', (6, (66, 666))),   # should get one of these
                 ('GET', (5, 55)),
                 ('GET', (4, 44)),
                 ('GET', None)),
            )
            for tp in testprogs:
                IM = InterruptManager()
                self._actions(IM, tp)

        def test_vectorcallback(self):
            def foo(d):
                d['foo'] = 1234

            foodict = {}
            pfoo = partial(foo, foodict)
            IM = InterruptManager()
            IM.pend_interrupt(PendingInterrupt(4, 888, pfoo))
            iinfo = IM.get_pending(0)
            self.assertEqual(foodict['foo'], 1234)

    unittest.main()
