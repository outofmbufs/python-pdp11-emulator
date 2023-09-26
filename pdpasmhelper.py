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
from collections import defaultdict
from functools import partial


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

        # called in various contexts which may or may not
        # require a '$' on immediate constants; skip it if present
        if s[0] == '$':
            s = s[1:]

        # default octal unless number terminates with '.'
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

    def operand_parser(self, operand_token, /):
        """Parse operand_token ('r1', '-(sp)', '4(r5)', $177776, etc).

        Returns: sequence: [6 bit code, additional words ...]

        Raises ValueError for syntax errors.

        Literals that should become (pc)+ (mode 0o27) must start with '$'
        They will be octal unless they end with a '.'

        Literals that are pointers and should become @(pc)+ must
        start with '*$' and will be octal unless they end with a '.'

        An integer, i, can be passed in directly.
        """

        # for convenience
        cannotparse = ValueError(f"cannot parse '{operand_token}'")

        # normalize the operand, upper case for strings, turn ints back
        # into their corresponding string (roundabout, but easiest)
        try:
            operand = operand_token.upper()
        except AttributeError:
            operand = f"${operand_token}."

        # bail out if spaces in middle, and remove spaces at ends
        s = operand.split()
        if len(s) > 1:
            raise cannotparse
        operand = s[0]

        # operand now fully normalized: upper case, no spaces.

        # the first/easiest to try is to see if it is an immediate.
        # It will (must) start with either '$', or '*$' if so.
        try:
            if operand[0] == '$':
                return [0o27, self.immediate_value(operand)]
            elif operand.startswith('*$'):
                return [0o37, self.immediate_value(operand[1:])]
        except ValueError:
            raise cannotparse from None

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
            raise cannotparse
        idxval = self.immediate_value(s[0])

        # the back end of this, with the '(' put back on,
        # must end with ')' and must parse
        if s[1][-1] != ')':
            raise cannotparse
        try:
            b6 = self.B6MODES['(' + s[1]]
        except KeyError:
            raise cannotparse from None
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
            return self._seqwords([operation])
        else:
            dst6, *dst_i = self.operand_parser(dst)
            return self._seqwords([operation | dst6, *dst_i])

    # XXX the instructions are not complete, this is being developed
    #     as needed for pdptests.py
    # ALSO: see InstructionBlock for branch support

    def mov(self, src, dst):
        return self._2op(0o010000, src, dst)

    def movb(self, src, dst):
        return self._2op(0o110000, src, dst)

    def cmp(self, src, dst):
        return self._2op(0o020000, src, dst)

    def bit(self, src, dst):
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


class FwdRef:
    """Values determined by a not-yet-seen label() definition."""

    def __init__(self, name, block):
        self.loc = len(block)
        self.name = name
        self.block = block
        block._fwdrefs[name].append(self)

    def __call__(self):
        block = self.block
        # the location to be patched is in one of three places, look for it:
        for loco in (0, 1, 2):
            if block._instblock[self.loc + loco] is self:
                block._instblock[self.loc + loco] = self.transform()
                break
        else:
            raise ValueError(f"could not find FwdRef {self}")

    def __iter__(self):
        return iter([0o27, self])

    def transform(self):
        return self.block.getlabel(self.name) - (2 * self.loc)


class BranchTarget(FwdRef):
    def __init__(self, brcode, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__brcode = brcode

    @staticmethod
    def branchencode(brcode, offs, name):
        """The guts of encoding/checking a branch offset."""

        # offs is in 16-bit form as a byte offset; convert it to
        # 8-bit branch (word offset) form and make sure not too big

        if offs > 254 and offs < (65536 - 256):
            raise ValueError(f"branch target ('{name}') too far.")
        offs >>= 1
        return brcode | (offs & 0o377)

    def transform(self):
        """Called when a forward branch ref is ready to be resolved."""
        offs = self.block.getlabel(self.name) - (2 * (self.loc + 1))
        return self.branchencode(self.__brcode, offs, self.name)


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
# Use:
#
#   instlist = a.instructions()
#
# to get a list of instructions. By default, instructions() raises
# a ValueError if there are dangling forward references. This can be
# suppressed (usually for debugging) by a.instructions(allow_dangling=True)
#

class InstructionBlock(PDP11InstructionAssembler, AbstractContextManager):
    def __init__(self):
        super().__init__()
        self._instblock = []
        self._labels = {}
        self._fwdrefs = defaultdict(list)

    def _seqwords(self, seq):
        """seq can be an iterable, or a naked (integer) instruction."""
        try:
            self._instblock += seq
        except TypeError:
            self._instblock += [seq]
        return seq

    # Extend base operand_parser with ability to handle labels,
    # including forward references
    def operand_parser(self, operand_token, *args, **kwargs):
        # it's possible to get here with operand_token already
        # being a forward ref (e.g., if getlabel was used)
        if isinstance(operand_token, FwdRef):
            return operand_token
        else:
            try:
                return super().operand_parser(operand_token, *args, **kwargs)
            except ValueError as e:
                if not self._allowable_label(operand_token):
                    raise

            # falling through to here means it is a label or forward reference
            return self.getlabel(operand_token)

    def __len__(self):
        """Returns the length of the sequence in WORDS"""
        return len(self._instblock)

    def __dotandnumbers(self, w):
        """Turn '.' into 2x current offset, turn numbers into integers"""
        if isinstance(w, int):          # already an integer
            return w
        elif w in '+-':
            return w
        elif w == '.':
            return len(self) * 2
        elif w[-1] == '.':               # 12345. for example
            return int(w[:-1])
        elif self._allowable_label(w):
            return self.getlabel(w)
        else:
            return int(w, 8)

    def _allowable_label(self, s):
        if not hasattr(s, 'isalpha'):
            return False
        return ((s.upper() not in self.B6MODES) and
                (s[0].isalpha() or s[0] == '_'))

    def label(self, name, *, value='.'):
        """Record the current position, or 'value', as 'name'.

        If no value specified, it defaults to '.' which means
        the current position index, multiplied by 2 so that it
        is suitable to add to a base address. Otherwise the value
        is taken as-is, or with a trivial amount of arithmetic.

        Labels must start with a .isalpha() character and
        must not match (ignoring case) any of the tokens in B6MODES
        """

        try:
            value_tokens = value.split()
        except AttributeError:
            value_tokens = [value]

        value_tokens = [self.__dotandnumbers(w) for w in value_tokens]
        if len(value_tokens) == 3:
            if value_tokens[1] == '+':
                value_tokens = [value_tokens[0] + value_tokens[2]]
            elif value_tokens[1] == '-':
                value_tokens = [value_tokens[0] - value_tokens[2]]

        if len(value_tokens) != 1:
            raise ValueError(f"cannot parse '{value}'")

        self._labels[name] = value_tokens[0]

        # if there were any forward references to this name, process them
        for fref in self._fwdrefs[name]:
            fref()
        del self._fwdrefs[name]

        return self._labels[name]

    def getlabel(self, name, *, fwdfactory=FwdRef):
        """Return value (loc) of name, which may be a FwdRef object.

        If the label is a forward reference, the fwdfactory argument
        (default=FwdRef) will be used to create a FwdRef object placed
        into the instruction stream until resolved later. The default FwdRef
        class patches in a 16-bit value once known.  Branch (and other)
        instructions supply FwdRef subclasses via fwdfactory for customized
        encoding/processing of resolved references.

        If fwdfactory is None, forward references raise a TypeError
        """
        try:
            return self._labels[name]
        except KeyError:
            return fwdfactory(name=name, block=self)

    @staticmethod
    def _neg16(x):
        """convert negative numbers in 16-bit two's complement."""
        origx = x
        if x < 0 and x >= -32768:
            x += 65536
        if x < 0 or x > 65535:
            raise ValueError(f"offset '{origx}' out of 16-bit range")
        return x

    def _branchcommon(self, target, *, fwdfactory=None):
        """Common logic for bne, bgt, etc including unconditional br."""

        # target at this point can be:
        #    An integer -- treat directly as an offset value
        #    A string representing a direct offset - parse/use
        #    A label -- possibly forward reference or not

        try:
            if target.startswith('$'):
                x = self.immediate_value(target)
            else:
                # it's a label, which may or may not be forward ref
                x = self.getlabel(target, fwdfactory=fwdfactory)
                try:
                    x -= (2 * (len(self) + 1))
                except TypeError:     # a forward reference
                    pass
        except AttributeError:
            # it's not a string, assume it is an offset
            x = target

        # At this point it's either a number or a forward ref.
        # For numbers, complete everything now.
        # For forward refs, that work is deferred.
        try:
            x = self._neg16(x)
        except TypeError:
            pass
        return x

    # dynamically construct the methods for all the Bxx branches
    # This makes methods: beq, bne, bgt, etc
    for _bname, _code in BRANCH_CODES.items():
        def branchxx(self, target, code=_code):
            bxxfactory = partial(BranchTarget, code)
            w = self._branchcommon(target, fwdfactory=bxxfactory)
            # it's either an integer offset or a forward reference.
            # Encode integers now; forward references are encoded later
            if isinstance(w, int):
                w = BranchTarget.branchencode(code, offs=w, name=target)
            return self._seqwords([w])
        branchxx.__name__ = _bname
        setattr(PDP11InstructionAssembler, _bname, branchxx)
    del _bname, _code, branchxx

    def sob(self, reg, target):
        # the register can be a naked integer 0 .. 5 or an 'r' string
        try:
            lc = reg.lower()
        except AttributeError:
            pass
        else:
            if len(lc) == 2 and lc[0] == 'r':
                reg = int(lc[1:])

        # NOTE: forward references illegal; no fwdfactory given
        try:
            x = self._branchcommon(target)
        except (ValueError, TypeError):
            raise ValueError(f"sob '{target}' illegal target") from None

        # stricter limits on the offset size for sob:
        #   Must be between 0 and -126
        if x < 0o177602:    # (65536-126)
            raise ValueError(f"sob target ({x}) too far")

        return self.literal(0o077000 | (reg << 6) | (((-x) >> 1) & 0o77))

    def instructions(self, *, allow_dangling=False):
        # By default, it is an error to request the instructions if there
        # are unresolved forward references.
        if self._fwdrefs and not allow_dangling:
            raise ValueError(f"unresolved references: "
                             f"{list(self._fwdrefs)}")
        return self._instblock

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

        def test_labelmath_dot(self):
            with ASM()as a:
                a.mov('bozo', 'r0')
                a.label('B')
                a.label('BP2', value='. + 2')
                a.clr('r0')
                a.label('bozo')

            self.assertEqual(a.getlabel('B'), 4)
            self.assertEqual(a.getlabel('BP2'), 6)
            self.assertEqual(a.getlabel('BP2'), a.getlabel('bozo'))
            self.assertEqual(a.instructions()[1], 6)

        def test_labelmath_plus(self):
            with ASM() as a:
                a.label('L1', value=17)
                a.label('L2', value='L1 + 25.')
            self.assertEqual(a.getlabel('L2'), 42)

        def test_labelmath_minus(self):
            with ASM() as a:
                a.label('L1')
                a.clr('r0')
                a.label('L2', value='. - L1')
            self.assertEqual(a.getlabel('L2'), 2)

        def test_unresolved(self):
            with ASM() as a:
                a.br('bozo')
                a.clr('r0')
                a.mov(a.getlabel('xyzzy'), 'r0')
            with self.assertRaises(ValueError):
                foo = a.instructions()

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
