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

    # no-op here, but overridden in _Sequence to track generated instructions
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
        return self._2op(0o040000, src, dst)

    def add(self, src, dst):
        return self._2op(0o060000, src, dst)

    def sub(self, src, dst):
        return self._2op(0o160000, src, dst)

    def clr(self, dst):
        return self._1op(0o005000, dst)

    def inc(self, dst):
        return self._1op(0o005200, dst)

    def dec(self, dst):
        return self._1op(0o005300, dst)

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
# A list of instructions in an InstructionBlock can be obtained at any
# time via:   insts = a.instructions()
#

class InstructionBlock(PDP11InstructionAssembler, AbstractContextManager):
    def __init__(self):
        super().__init__()
        self._instblock = []
        self.labels = {}

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
        curoffs = len(self)
        self.labels[name] = curoffs
        return curoffs

    # Branch instruction support only exists within a given InstructionBlock
    def bxx_offset(self, name1, name2=None):
        """Generate offset appropriate to Bxx between name1 and name2.

        If name2 is None, generate offset, backwards, from current to name1.
        """

        # XXX TODO XXX make forward references possible and automate the
        #              backpatching even if that gets one step closer
        #              to slowly implementing an entire assembler...
        if name2 is None:
            # +255 not 256 because account for the Bxx instruction itself
            offs = self.labels[name1] - len(self) + 255
        else:
            raise ValueError("two name bxx_offset not yet implemented")

        offs8 = offs & 0o377
        if offs8 != offs:
            raise ValueError(f"distance to {name1} too far.")
        return offs8

    def bne(self, name):
        return self.literal(0o001000 | self.bxx_offset(name))

    def instructions(self):
        return self._instblock
