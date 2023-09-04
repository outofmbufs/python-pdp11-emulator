def boot_hp(p, addr=0o10000):
    # this is the sort of thing that would be keyed in from
    # the console switches (if the machine was not equipped
    # with a boot rom option to hold it instead)
    #
    # It is a minimalist  program, with lots of assumptions, to  read 1K
    # from block zero of drive 0 into location 0. The execution start
    # at zero is done elsewhere.
    #
    # NOTE WELL: THIS ASSUMES THE MACHINE IS IN RESET CONDITION WHICH
    #            MEANS MANY OF THE DEVICE REGISTERS ARE KNOWN TO BE ZERO
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

    return addr


if __name__ == "__main__":
    import time
    from machine import PDP1170
    p = PDP1170(loglevel='INFO')
    pc = boot_hp(p)
    print("starting PDP11; telnet/nc to localhost:1170 to connect to console")
    print("There will be no prompt; type 'boot' to start boot program")
    p.run(pc=pc)
    # technically need to confirm the drive is RDY, i.e., the read
    # completed, but using a delay is a lot simpler and works fine.
    # In real life, humans would have manipulated console switches to
    # start execution at location 0, which is also a source of delay. :)
    time.sleep(0.05)
    p.run(pc=0)
