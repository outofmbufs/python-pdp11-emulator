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

class PDP11InstructionAssembler:
    def __init__(self):
        # crude but so so easy .. translate string to 6-bit operand mode
        self.B6MODES = {}

        for i in range(8):
            rnames = [f"R{i}"]
            if i == 6:
                rnames.append("SP")
            elif i == 7:
                rnames.append("PC")

            for rn in rnames:
                self.B6MODES[f"{rn}"] = i              # register direct
                self.B6MODES[f"({rn})"] = 0o10 | i     # register indirect
                self.B6MODES[f"({rn})+"] = 0o20 | i    # autoincrement
                self.B6MODES[f"@({rn})+"] = 0o30 | i   # autoincr deferred
                self.B6MODES[f"-({rn})"] = 0o40 | i    # autodecrement
                self.B6MODES[f"@-({rn})"] = 0o50 | i   # autodecr deferred
            # X(Rn) and @X(Rn) cannot be parsed this simple lookup way

    def operand_parser(self, operand_string, /):
        """Parse operand_string ('r1', '-(sp)', '4(r5)', etc).

        Returns: sequence: [6 bit code, additional words ...]

        Raises ValueError for syntax errors.

        """
        # NOTE: Not all forms implemented yet. See FUNCTIONALITY DISCLAIMER.

        # normalize the operand
        operand = operand_string.upper()
        s = operand.split()     # this also removes leading/trailing space
        if len(s) > 1:          # but if there were middle spaces... no good
            raise ValueError(f"more than one word in '{operand_string}'")
        operand = s[0]

        # operand should be fully normalized: upper case, no spaces.

        try:
            return [self.B6MODES[operand]]
        except KeyError:
            pass
        
        # for convenience
        def valerr():
            return ValueError(f"cannot parse '{operand_string}'")

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
        idxstr = s[0]

        # idxstr can only end with a digit or a '.' ... in particular
        base = 8
        if idxstr[-1] == '.':
            base = 10
            idxstr = idxstr[:-1]
        elif not idxstr[-1].isdigit():
            raise valerr()

        idxval = int(idxstr, base)

        # the back end of this, with the '(' put back on,
        # must end with ')' and must parse
        if s[1][-1] != ')':
            raise valerr()
        try:
            b6 = self.B6MODES['(' + s[1]]
        except KeyError:
            raise valerr() from None

        return [mode | (b6 & 0o07), idxval]

            
            
        
