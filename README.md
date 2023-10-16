# python-pdp11-emulator
Yet another PDP-11 emulator, written in Python. Can boot/run V7 Unix!

## Why, oh why??
Let's be clear - SIMH is better in every possible way. And there are also other PDP-11 emulator implementations out there, some in python. I did this as a self-educational project and because I thought it would be fun.

It was educational.

It was fun.

If you like it, great. If you think it is silly, so it goes.

## RUNNING A BARE PDP-11
Do it like this:

    % python3
    >>> from machine import PDP1170
    >>> p = PDP1170()

At this point how you jam instructions into memory is up to you.

The PDP-11 registers are in attribute `r` (e.g., `p.r[0]` to see R0).

Memory is best read/written using p.mmu.wordRW:

     value = p.mmu.wordRW(some-address)       # reads some-address
     p.mmu.wordRW(some-address, value)        # writes value to some-address

For example:

    >>> from machine import PDP1170
    >>> p = PDP1170()
    >>> p.mmu.wordRW(0o10000, 0o010203)
    4227
    >>> p.mmu.wordRW(0o10002, 0)
    0
    >>> p.r[7] = 0o10000
    >>> p.r[2] = 2
    >>> p.r
    [0, 0, 2, 0, 0, 0, 0, 4096]
    >>> p.run()
    >>> p.r
    [0, 0, 2, 2, 0, 0, 0, 4100]

showing a program: MOV R2, R3 / HALT and running it and showing the before and after (as the "2" is copied from R2 to R3)

## To boot UNIX:
* Obtain a disk image. I am using hp.disk0 from the unix7 subdirectory of the systems directory from SIMH.

* name that file rp.disk ... the name is not configurable in the code yet.

* You will need two separate shell windows. One will be the running python window, where not much happens once you get started. The other will be your emulated console. In the first window, type this:

    python3 boot.py

You may wish to examine 'boot.py' to see how all this works.

In the second window, you need to telnet to port 1170 to connect to the console emulation. It will be very helpful to be in raw terminal mode too. On the mac I do it like this:

    % (stty raw; nc localhost 1170; stty sane)

in this window there will be no prompt. Send your complaints about that to (I assume) DMR's heirs, as you are running the raw 512-byte block-zero bootstrap code at this point from unix7.

Anyhow, there is no prompt. Type:

    boot

you will then get a prompt:

    Boot
    :

If you have the SIMH hp.disk0 as your rp.disk, type "hp(0,0)unix"

I apologize for the hp/rp confusion. Perhaps I will clean this up in a future release.

You will see a single user shell prompt at this point if you are lucky!

Exit the single user shell (type a control-D) to get into multi-user mode. NOTE: The UNIX /etc/ttys file defaults to enabling four additional login lines but the emulation that is implemented (see dl11.py) simply makes them appear to the kernel as "no terminal connected". The (emulated) console is the only serial device supported so far.


Enjoy!

## Understanding the code
I'll have to write more about this later; as a first step to getting around:

* machine.py is the heart of the instruction loop and CPU basics.
* Actual machine instructions have been split off - start with op4.py and trace the opparsing/dispatch from there.
* mmu.py implements, surprisingly enough, the PDP-11 MMU system.
* mmio.py and unibus.py (mostly a stub) handle the memory mapped I/O.
* rp.py is a primitive disk emulation, good enough for unix.
* kl11.py is the console
* kw11.py is the line clock

Note that the disk operations are synchronous (not threaded/asyncio). I have tried going async and it doesn't seem to make much difference. Not sure why just yet but it works well enough in synchronous mode (in other words - when Unix commands a read, the read happens right then and no emulated instructions happen "during" the read ... then the interrupt is fired and unix continues, as if the disk were infinitely fast because no instruction cycles elapsed between GO and the interrupt).

## tests

Some (somewhat trivial) unit tests:

    python3 pdptests.py

# TODO

Known areas that need work include:
* Need to emulate more devices, especially a DL-11
* The UNIBUS address space and especially the UBA system is a stub. The disk drive is, in effect, emulated as being on the Massbus and uses its BAE register (which the unix driver sets accordingly) for the 22-bit physical address extension bits.
* No floating point instructions implemented; not sure how important they are. They were an option, so presumably all code can deal with them not being present.
* I am not at all convinced I have all the subtleties of traps, synchronous traps, aborts, interrupts, etc correct. I have it good enough for what Unix expects.
* Explore more regarding asynchronous disk I/O.
* When running unix, there's an inexplicable pause that happens fairly often. I have ruled out gc as a cause. I am suspicious this has something to do with the reduced clock rate, or a subtle semantic of interrupts/traps/etc.
