import time
from machine import PDP1170
from kw11 import KW11
from kl11 import KL11
from dc11 import DC11
from rp import RPRM

import breakpoints


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


def _must_read_n(f, n, zeroskip=False):
    """read exactly n bytes from f; raise exception if can't. n==0 allowed.

    If zeroskip is True (default: False), zero bytes will be discarded.
    It is not legal to zeroskip with n == 0.
    """

    if zeroskip and n == 0:
        raise ValueError("zeroskip and n == 0")

    b = bytes()
    if n > 0:
        while zeroskip and (b := f.read(1)) == b'\00':
            pass

        # b has one byte or none in it, depending on zeroskip
        b += f.read(n - len(b))
        if len(b) != n:
            raise ValueError(f"needed {n} bytes; got {len(b)}")
    return b


def _byte_phys_write(p, b, addr):
    """write b (bytes()) to addr. b may be odd length. addr may be odd."""

    # addr can be odd, length can be odd. Pick up the bookend bytes
    # as needed so physRW_N can do it as words.
    if addr & 1:
        addr -= 1
        b = bytes([p.physRW(addr) & 255]) + b
    if len(b) & 1:
        b += bytes([(p.physRW(addr+len(b)-1) >> 8) & 255])

    words = [(b[i+1] << 8) | b[i] for i in range(0, len(b), 2)]
    p.physRW_N(addr, len(words), words)


def load_lda_f(p, f):
    """Read  and load open file f as an 'absolute loader' file
    (.LDA, sometimes .BIC file). This is the same format that simh
    defines for its load (binary) command.

    Returns: the address specified in the END block (per LDA docs)

    Any file format errors or I/O errors will raise an exception.
    """

    # Archived DEC documentation says an LDA/absolute loader file may start
    # with an arbitrary number of zero bytes. SIMH says that ANY block
    # (not just the first) may have arbitrary zeros in front of it, and
    # testing reveals that SIMH indeed allows (ignores) such zeros.
    #
    # Such zeros were probably more common in the (real) paper tape days.
    # They are supported here simply because SIMH does too.
    #
    # The file is a sequence of blocks in header/data/checksum format:
    #   [HEADER]          -- 6 bytes
    #   [DATA]            -- size determined by header, can be zero length
    #   [CHECKSUM]        -- 1 byte
    #
    # The header is six individual bytes, as follows:
    #         1             -- literally, a byte with value 1
    #         0             -- "MUST" be zero (this code ignores this byte)
    #       len-lsb         -- lower 8 bits of block length
    #       len-msb         -- upper 8 bits of block length
    #       addr-lsb        -- lower 8 bits of address
    #       addr-msb        -- upper 8 bits of address
    #
    # As mentioned, runs of zeros in prior to such a header (which starts
    # with a 1) are ignored.
    #
    # The 'block length' includes the header bytes but not the checksum.
    # Thus the lengthof [DATA] is six less than the block length given.
    # Note that the [DATA] length is allowed to be odd, or zero.
    #
    # If the [DATA] length is zero (block length 6) the block is
    # an "END" block that terminates the format. The address indicated
    # is the "start address" and is returned.

    while True:
        header = _must_read_n(f, 6, zeroskip=True)
        if header[0] != 1:
            raise ValueError(f"header starts with {header[0]} not 1")

        count = (header[3] << 8) | header[2]
        addr = (header[5] << 8) | header[4]

        if count < 6:
            raise ValueError(f"header error, {count=}")

        b = _must_read_n(f, count-6)

        chksum = _must_read_n(f, 1)
        if (sum(header) + sum(b) + sum(chksum)) & 0xFF:
            raise ValueError(f"checksum mismatch, {header=}")

        if count == 6:         # END block with no data
            return addr

        _byte_phys_write(p, b, addr)

    assert False, "unreachable"


def information_screen():
    print("Starting PDP11; this window is NOT THE EMULATED PDP-11 CONSOLE.")
    print("*** In another window, telnet/nc to localhost:1170 to connect.")
    print("    Terminal should be in raw mode. On a mac, this is a good way:")
    print("         (stty raw; nc localhost 1170; stty sane)")
    print("")
    print("There will be no prompt; type 'boot' in your OTHER window")
    print("")
    print("Then, at the ':' prompt, typically type: hp(0,0)unix")
    print("")

    input("Press enter to continue")


def boot_lda(p, fname, /, *, force_run=True):
    """Load and boot an LDA/BIC/absolute-loader file.

    By default, the loaded code is started even if the start address
    given in the LDA file is odd (which is normally a flag to not start it).
    The start address is rounded down 1 to even in such cases.

    To override that behavior, specify force_run=False. The file will only
    be run if the start address is even.

    In all cases the raw start address is returned; however, if the loaded
    code is successfully started there will never be a return unless that
    code eventually halts.
    """
    with open(fname, 'rb') as f:
        addr = rawaddr = load_lda_f(p, f)

    if rawaddr & 1:
        if not force_run:
            return rawaddr
        addr = rawaddr - 1

    if addr == 0:
        addr = 0o200

    information_screen()

    p.run(pc=addr)
    return rawaddr


def make_unix_machine(*, loglevel='INFO', drivenames=[]):
    p = PDP1170(loglevel=loglevel)

    p.associate_device(KW11(p.ub), 'KW')    # line clock
    p.associate_device(KL11(p.ub), 'KL')    # console
    p.associate_device(RPRM(p.ub, *drivenames), 'RP')    # disk drive
    p.associate_device(DC11(p.ub), 'DC')    # additional serial poirts
    return p


def boot_unix(p, runoptions={}):

    # load, and execute, the key-in bootstrap
    boot_hp(p)

    information_screen()

    p.run(pc=0, **runoptions)


# USE:
#    python3 boot.py
#
# to start up unix (or whatever system is on the drive)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--drive', action='append', default=[], dest='drives')
    parser.add_argument('--instlog', action='store_true')
    parser.add_argument('--lda', action='store', default=None, dest='lda')
    args = parser.parse_args()

    pdpoptions = {'drivenames': args.drives}
    runoptions = {}
    if args.debug:
        pdpoptions['loglevel'] = 'DEBUG'
    if args.instlog:
        runoptions['breakpoint'] = breakpoints.Logger()

    p = make_unix_machine(**pdpoptions)

    if args.lda:
        boot_lda(p, args.lda)
    else:
        boot_unix(p, runoptions=runoptions)
