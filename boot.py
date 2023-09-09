import time
from machine import PDP1170
from kw11 import KW11
from kl11 import KL11
from rp import RPRM


def boot_hp(p, /, *, addr=0o10000, deposit_only=False):
    """Deposit, then run, instructions to read first 1KB of drive 0 --> addr.
    RETURN VALUE:  addr if deposit_only else None

    If no 'addr' given, it defaults to something out of the way.

    If not deposit_only (default):
       * The instructions are loaded to 'addr'
       * They are executed.
       * Return value is None.
       * NOTE: The next start address depends on what those instructions do.
       *       TYPICALLY, the next start address will be zero.

    If deposit_only:
       * The instructions are loaded to 'addr'
       * 'addr' is returned.
    """

    # this is the sort of thing that would be keyed in from
    # the console switches (if the machine was not equipped
    # with a boot rom option to hold it instead)
    #
    # It is a minimalist  program, with lots of assumptions, to  read 1K
    # from block zero of drive 0 into location 0. The execution start
    # at zero is done elsewhere.
    #
    # NOTE WELL: THIS ASSUMES THE MACHINE IS IN RESET CONDITION WHICH
    #            MEANS MANY OF THE DEVICE REGISTERS ARE ASSUMED TO BE ZERO
    #
    #      MOV #176704,R0       -- note how used
    #      MOV #177000,-(R0)    -- word count - read 1K though boot really 512
    #      MOV #071,-(R0)       -- go!
    program_insts = (
        0o012700,          # MOV #0176704,R0
        0o176704,
        0o012740,          # MOV #177000,-(R0)
        0o177000,
        0o012740,          # MOV #071, -(R0)
        0o000071,
        0o0,               # HALT
    )

    for o, w in enumerate(program_insts):
        p.physRW(addr + o + o, w)

    p.r[p.PC] = addr
    if not deposit_only:
        p.run()

        # at this point in real life the user would have to set the switches
        # to deposit zero into the PC and then hit start; that takes time
        # and means the bootstrap doesn't have to have code to wait for the
        # drive to complete the read operation. Instead of adding code to
        # the "pretend this was keyed in" program above, this delay works.
        time.sleep(0.25)

    return addr if deposit_only else None


def boot_bin(p, fname, /, *, addr=0, deposit_only=False,
             little_endian=True, skipwords=8):
    """Read a binary file 'fname' into location 'addr' and execute it.
    RETURN VALUE:  addr if deposit_only else None

    NOTE: fname is in the host system, not on an emulated drive.
    If no 'addr' given, it defaults to ZERO.
    If deposit_only=True, the instructions are not executed.

    little_endian (default True) dictates the fname byte order.
    skipwords (default 8 -- a.out header) will seek that many 16-bit
    words into the file before beginning to load.
    """
    with open(fname, 'rb') as f:
        bb = f.read()

        # Two data format cases:
        #   1) little_endian (the default)
        #
        #      The file is truly a binary image of pdp11 format data
        #      and the words (pairs of bytes) in bb are in little endian
        #      order. They will be assembled accordingly and the names
        #      "low" and "hi" make sense.
        #
        #   2) not little_endian
        #
        #      Presumably the file has been byte-swapped already and
        #      the words (pairs of bytes) in it are in big endian order.
        #      They will be assembled accordingly, but the names "low"
        #      and "hi" are backwards.
        #

        xi = iter(bb)
        words = []
        for low in xi:
            hi = next(xi)
            if little_endian:
                words.append((hi << 8) | low)
            else:
                words.append((low << 8) | hi)    # see case 2) above

        for a, w in enumerate(words[skipwords:]):
            p.physmem[a] = w

    p.r[p.PC] = addr
    if not deposit_only:
        p.run()
    return addr if deposit_only else None


def boot_unix(p=None, loglevel='INFO'):

    if p is None:
        p = PDP1170(loglevel='INFO')

        p.associate_device(KW11(p.ub), 'KW')    # line clock
        p.associate_device(KL11(p.ub), 'KL')    # console
        p.associate_device(RPRM(p.ub), 'RP')    # disk drive

        # load, and execute, the key-in bootstrap
        boot_hp(p)

    print("Starting PDP11; this window is NOT THE EMULATED PDP-11 CONSOLE.")
    print("*** In another window, telnet/nc to localhost:1170 to connect.")
    print("    Terminal should be in raw mode. On a mac, this is a good way:")
    print("         (stty raw; nc localhost 1170; stty sane)")
    print("")
    print("There will be no prompt; type 'boot' in your OTHER window")
    print("")
    print("Then, at the ':' prompt, typically type: hp(0,0)unix")

    p.run(pc=0)


# USE:
#    python3 boot.py
#
# to start up unix (or whatever system is on the drive)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    if args.debug:
        boot_unix(loglevel='DEBUG')
    else:
        boot_unix()
