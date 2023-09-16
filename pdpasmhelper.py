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

# FUNCTIONALITY DISCLAIMER:
#   This is NOT meant to recreate the entire idea of a PDP-11 assembler.
#   Rather, it is meant as an ad-hoc assistance for creating and
#   debugging small test programs, of the sort that are found in pdptest.
#   As such, the methods here are written on an "as-needed" basis and
#   are focused around helping to create hand-constructed test code.
#

from contextlib import AbstractContextManager
from branches import BRANCH_CODES
from collections import namedtuple

FwdRef = namedtuple('FwdRef', ['f', 'loc', 'name', 'block'])


class PDP11InstructionAssembler:
    B6MODES = {}
    _rnames = [(f"R{_i}", _i) for _i in range(8)] + [("SP", 6), ("PC", 7)]
    for _rn, _i in _rnames:
        B6MODES[f"{_rn}"] = _i              # register direct
        B6MODES[f"({_rn})"] = 0o10 | _i     # register indirect
        B6MODES[f"({_rn})+"] = 0o20 | _i    # autoincrement
        B6MODES[f"@({_rn})+"] = 0o30 | _i   # autoincr deferred
        B6MODES[f"-({_rn})"] = 0o40 | _i    # autodecrement
        B6MODES[f"@-({_rn})"] = 0o50 | _i   # autodecr deferred
    del _i, _rn, _rnames

    # see InstructionBlock for explanation of 'with' syntax use
    @classmethod
    def __enter__(cls):
        return InstructionBlock()

    def __exit__(self, *args, **kwargs):
        return None

    def immediate_value(self, s):
        base = 8
        if s[-1] == '.':
            base = 10
            s = s[:-1]
        val = int(s, base)

        # as a convenience, allow negative values and convert them
        if val < 0 and val >= -32768:
            val += 65536
        if val > 65535 or val < 0:
            raise ValueError(f"illegal value '{s}' = {val}")
        return val

    # this is a notational convenience to create a f'*${i].' string
    # for an operand that is an immediate deferred (i.e., numeric pointer)
    def ptr(self, i):
        return f'*${i}.'

    def operand_parser(self, operand_string, /):
        """Parse operand_string ('r1', '-(sp)', '4(r5)', $177776, etc).

        Returns: sequence: [6 bit code, additional words ...]

        Raises ValueError for syntax errors.

        Literals that should become (pc)+ (mode 0o27) must start with '$'
        They will be octal unless they end with a '.'

        Literals that are pointers and should become @(pc)+ must
        start with '*$' and will be octal unless they end with a '.'

        An integer, i, can be passed in directly; it is becomes f"${i}."
        """
        # NOTE: Not all forms implemented yet. See FUNCTIONALITY DISCLAIMER.

        # for convenience
        def valerr():
            return ValueError(f"cannot parse '{operand_string}'")

        # normalize the operand, upper case for strings, turn ints back
        # into their corresponding string (roundabout, but easiest)
        try:
            operand = operand_string.upper()
        except AttributeError:
            operand = f"${operand_string}."

        # bail out if spaces in middle, and remove spaces at ends
        s = operand.split()
        if len(s) > 1:
            raise valerr()
        operand = s[0]

        # operand now fully normalized: upper case, no spaces.

        # the first/easiest to try is to see if it is an immediate.
        # It will (must) start with either '$', or '*$' if so.
        try:
            if operand[0] == '$':
                return [0o27, self.immediate_value(operand[1:])]
            elif operand.startswith('*$'):
                return [0o37, self.immediate_value(operand[2:])]
        except ValueError:
            raise valerr() from None

        # wasn't immediate, see if it matches the precomputed modes
        try:
            return [self.B6MODES[operand]]
        except KeyError:
            pass

        # last chance: X(Rn) and @X(rn)

        # see if X(Rn) or @X(Rn)...
        if operand[0] == '@':
            mode = 0o70
            operand = operand[1:]
        else:
            mode = 0o60

        # for starters, it must contain one '(' so should split to 2
        s = operand.split('(')
        if len(s) != 2:
            raise valerr()
        idxval = self.immediate_value(s[0])

        # the back end of this, with the '(' put back on,
        # must end with ')' and must parse
        if s[1][-1] != ')':
            raise valerr()
        try:
            b6 = self.B6MODES['(' + s[1]]
        except KeyError:
            raise valerr() from None
        return [mode | (b6 & 0o07), idxval]

    # gets overridden in InstructionBlock to track generated instructions
    def _seqwords(self, seq):
        return seq

    # All 2 operand instructions end up here eventually
    def _2op(self, operation, src, dst):
        src6, *src_i = self.operand_parser(src)
        dst6, *dst_i = self.operand_parser(dst)
        return self._seqwords([operation | src6 << 6 | dst6, *src_i, *dst_i])

    # All 1 operand instructions end up here eventually
    # This also supports 0 operand "literals" (which are typically
    # instructions that have been hand-assembled another way)
    def _1op(self, operation, dst):
        """dst can be None for, essentially, a _0op."""
        if dst is None:
            dst6 = 0
            dst_i = []
        else:
            dst6, *dst_i = self.operand_parser(dst)
        return self._seqwords([operation | dst6, *dst_i])

    # XXX the instructions are not complete, this is being developed
    #     as needed for pdptests.py
    #
    # ALSO: see InstructionBlock for (primitive) branching support
    #
    def mov(self, src, dst):
        return self._2op(0o010000, src, dst)

    def movb(self, src, dst):
        return self._2op(0o110000, src, dst)

    def cmp(self, src, dst):
        return self._2op(0o020000, src, dst)

    def bic(self, src, dst):
        return self._2op(0o030000, src, dst)

    def bic(self, src, dst):
        return self._2op(0o040000, src, dst)

    def bis(self, src, dst):
        return self._2op(0o050000, src, dst)

    def add(self, src, dst):
        return self._2op(0o060000, src, dst)

    def sub(self, src, dst):
        return self._2op(0o160000, src, dst)

    def jmp(self, dst):
        return self._1op(0o000100, dst)

    def br(self, offs):
        return self.literal(0o000400 | (offs & 0o377))

    def clr(self, dst):
        return self._1op(0o005000, dst)

    def inc(self, dst):
        return self._1op(0o005200, dst)

    def dec(self, dst):
        return self._1op(0o005300, dst)

    def tst(self, dst):
        return self._1op(0o005700, dst)

    def swab(self, dst):
        return self._1op(0o000300, dst)

    def ash(self, cnt, dst):
        try:
            return self.literal(0o072000 | dst << 6, cnt)
        except TypeError:
            dstb6, *dst_i = self.operand_parser(dst)
            if dstb6 & 0o70:
                raise ValueError("ash dst must be register direct")
            return self.literal(0o072000 | dstb6 << 6, cnt)

    def halt(self):
        return self.literal(0)

    def rtt(self):
        return self.literal(6)

    def mtpi(self, dst):
        return self._1op(0o006600, dst)

    def mfpi(self, src):
        return self._1op(0o006500, src)

    def mtpd(self, dst):
        return self._1op(0o106600, dst)

    def mfpd(self, src):
        return self._1op(0o106500, src)

    def trap(self, tnum):
        return self.literal(0o104400 | tnum)

    def literal(self, inst, oprnd=None, /):
        """For hand-assembled instructions. Also allows 1 operand."""
        return self._1op(inst, oprnd)


# An InstructionBlock is a thin layer on just accumulating a sequence
# of results from calling the instruction methods.
#
# Instead of:
#   insts = (
#       a.mov('r1', 'r2'),
#       a.clr('r0'),
#          etc ...
#       )
#
# The context manager can be used to write it this way:
#
#   with ASM() as a:
#       a.mov('r1', 'r2')
#       a.clr('r0')
#        etc ...
#
# which, subject to opinion, may be notationally cleaner/clearer and also
# opens the possibility of if/for/etc full programming constructs as needed.
#
# The context manager also supports bare-bones labels, helpful for branches
#
# A list of instructions in an InstructionBlock can be obtained at any
# time via:   insts = a.instructions()
#

class InstructionBlock(PDP11InstructionAssembler, AbstractContextManager):
    def __init__(self):
        super().__init__()
        self._instblock = []
        self._labels = {}
        self._label_fwdrefs = {}

    def _seqwords(self, seq):
        """seq can be an iterable, or a naked (integer) instruction."""
        try:
            self._instblock += seq
        except TypeError:
            self._instblock += [seq]
        return seq

    def __len__(self):
        """Returns the length of the sequence in WORDS"""
        return len(self._instblock)

    def label(self, name):
        """Record the current position as 'name'."""
        curloc = len(self)
        self._labels[name] = curloc

        try:
            frefs = self._label_fwdrefs[name]
        except KeyError:
            pass
        else:
            for fref in frefs:
                fref.f(fref)
            del self._label_fwdrefs[name]

        return curloc

    def getlabel(self, name, *, callback=None):
        """Return value (loc) of name; register a callback if fwd ref."""
        try:
            return self._labels[name]
        except KeyError:
            if callback is None:
                # caller is not prepared to handle a forward reference
                raise
            # otherwise, register the callback and return None.
            # Callers prepared for forward references MUST check for None
            fref = FwdRef(f=callback, loc=len(self), name=name, block=self)
            try:
                self._label_fwdrefs[name].append(fref)
            except KeyError:
                self._label_fwdrefs[name] = [fref]
            return None

    @staticmethod
    def _neg16(x):
        """convert negative numbers in 16-bit two's complement."""
        origx = x
        if x < 0 and x >= -32768:
            x += 65536
        if x < 0 or x > 65535:
            raise ValueError(f"offset '{origx}' out of 16-bit range")
        return x

    def _label_or_offset(self, x, *, callback=None):
        """Return offset: either 'x' itself or computed from x as label.

        DOES NO VALIDATION OF SIZE OF RESULT (because different instructions
        have different requirements.
        """

        # If it's a str, treat it as a (possibly-forward-ref) label
        if isinstance(x, str):
            offs = self.getlabel(x, callback=callback)
            if offs is not None:
                # got a value - compute the delta
                x = offs - (len(self) + 1)
            else:
                return 0            # fref will be patched later

        return self._neg16(x)

    def _branchpatch(self, fref):
        fwdoffs = self.getlabel(fref.name) - (fref.loc + 1)
        block = fref.block
        block._instblock[fref.loc] |= block.bxx_offset(fwdoffs)

    # Branch instruction support only exists within a given InstructionBlock
    def bxx_offset(self, target, /):
        """Generate offset for Bxx target

        A target can be a string label or a number. Numbers are taken as-is.
        Names are looked up in the labels and offsets generated.
        """

        try:
            offs = self._label_or_offset(target, callback=self._branchpatch)

        except ValueError:
            raise ValueError(f"branch target ({target}) too far or illegal")

        # offsets come back from _label.. in 16-bit form, convert to 8
        # and make sure not too big in either direction
        if offs > 127 and offs < (65536 - 128):
            raise ValueError(f"branch target ('{target}') too far.")

        return offs & 0o377

    def bne(self, target):
        return self.literal(BRANCH_CODES['bne'] | self.bxx_offset(target))

    def blt(self, target):
        return self.literal(BRANCH_CODES['blt'] | self.bxx_offset(target))

    def beq(self, target):
        return self.literal(BRANCH_CODES['beq'] | self.bxx_offset(target))

    # overrides the base br to implement label support
    def br(self, target):
        return self.literal(0o000400 | self.bxx_offset(target))

    def sob(self, reg, target):
        offs = self._label_or_offset(target)

        # offsets are always negative and are allowed from 0 to -63
        # but they come from _label... as two's complement, so:
        if offs < 0o177701:    # (65536-63)
            raise ValueError(f"sob illegal target {target}")
        return self.literal(0o077000 | (reg << 6) | ((-offs) & 0o77))

    def instructions(self):
        return self._instblock

    # this is a convenience that allows a list of words (usually instructions)
    # to be "embedded" into an InstructionBlock with a leading jmp .+N
    # to jump over it.
    def jump_over_and_embed(self, words, /, *, name=None):
        """Embed words with leading 'jumpover'; returns offset of words

        If optional name given, creates a label for the words
        """
        if name is None:
            # an internal label name is generated instead
            name = f"__{id(object)}"

        self.jmp(f"{len(words)*2}.(pc)")
        words_offs = self.label(name)
        for w in words:
            self.literal(w)
        return words_offs

    def simh(self, *, startaddr=0o10000):
        """Generate lines of SIMH deposit commands."""

        for offs, w in enumerate(self.instructions()):
            yield f"D {oct(startaddr + (2 * offs))[2:]} {oct(w)[2:]}"


if __name__ == "__main__":
    import unittest

    ASM = PDP11InstructionAssembler

    # NOTE: these are tests of instruction ASSEMBLY not execution.
    class TestMethods(unittest.TestCase):

        def test_bne_label_distance(self):
            # this should just execute without any issue
            for i in range(127):
                with ASM() as a:
                    a.label('foo')
                    for _ in range(i):
                        a.mov('r0', 'r0')
                    a.bne('foo')

            # but this should ValueError ... branch too far
            with ASM() as a:
                a.label('foo')
                for _ in range(128):
                    a.mov('r0', 'r0')
                with self.assertRaises(ValueError):
                    a.bne('foo')

        def test_sob(self):
            for i in range(63):   # 0..62 because the sob also counts
                with self.subTest(i=i):
                    with ASM() as a:
                        a.label('foosob')
                        for _ in range(i):
                            a.mov('r0', 'r0')
                        inst = a.sob(0, 'foosob')
                        self.assertEqual(len(inst), 1)
                        self.assertEqual(inst[0] & 0o77, i+1)

    unittest.main()
