# python-pdp11-emulator
Yet another PDP-11 emulator, written in Python. Can boot/run V7 Unix!

## Why, oh why??
Let's be clear - SIMH is better in every possible way. And there are also other PDP-11 emulator implementations out there, some in python. I did this as a self-educational project and because I thought it would be fun.

It was educational.

It was fun.

If you like it, great. If you think it is silly, so it goes.

## ACKNOWLEDGEMENTS
Special thanks to Folkert van Heusden (https://github.com/folkertvanheusden) for looking at the instructions (especially the condition code results) in detail and finding many bugs.

## MORE NOTES
See NOTES file for running commentary on status, etc.

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

## KL11 Console Interface Emulation
A KL11 serial console interface device is emulated. To add this device to an instantiated machine:

    >>> from machine import PDP1170
    >>> from kl11 import KL11
    >>> p = PDP1170()
    >>> console = KL11(p.ub)

This by itself is enough to connect the device to the unibus and start it up. For introspection purposes the device can be "associated" which simply enters it into a mapping that can be seen in `p.devices` -- but there is absolutely no requirement to do this. It is the mere instantiation of the device object that starts it up and connects it to the emulated PDP11 Unibus. Doing the "association" is just an adornment that may be useful for debugging or other introspection.

That said, here's an example using the associate_device method and also adding in the line clock (KW11) emulation:

    >>> from machine import PDP1170
    >>> from kl11 import KL11
    >>> from kw11 import KW11
    >>> p = PDP1170()
    >>> p.associate_device(KL11(p.ub), 'KL')
    >>> p.associate_device(KW11(p.ub), 'KW')
    >>> p.devices
    {'KL': [<kl11.KL11 object at 0x100b774d0>],
     'KW': [<kw11.KW11 object at 0x100b77c10>]}


### socket (port 1170) vs stdin modes

As soon as the KL11 is instantiated it starts listening on port 1170 and will proxy characters back and forth between the emulated KL11 device and a (single) TCP connection to port 1170. Generally speaking the easiest way to connect to "the console" is via this type of command sequence on most unix/linux boxes:

    # DO THIS IN ANOTHER SHELL WINDOW, NOT THE PYTHON WINDOW
    % (stty raw; nc localhost 1170; stty sane)

which puts the tty in raw mode (character-at-a-time I/O, no echo) and connects. When the connection ends it restores the tty to 'sane' mode.

You can also use telnet instead of 'nc' if you have it on your system. If you use telnet, the emulation has an option to send RFC854 ("telnet options") to set no-echo mode for you. To do that change the KL11 instantiation to:

    KL11(p.ub, send_telnet=True)

It is also possible to use stdin/stdout (i.e., the python window itself) as the console. To do that instantiate the KL11 this way:

    KL11(p.ub, use_stdin=True)

The tty in which python is running will be placed into raw mode and all input will be consumed by the emulated KL11 device until the emulation is halted. Two things to note:

1. No control-C / SIGINT capability. If stdin is being used as the console, there will be no way to send a control-C or "abort" (or "break" or "SIGINT" or whatever it is called on your system). Any of those characters will simply become input to the emulation with no effect on the python interpreter. To work around this, the emulation looks for the unicode character c-cedilla ('ç' Unicode U+00E7; on a mac keyboard this can be typed using option-c). If that character is encountered the emulation is hard-halted, as if a user had hit the HALT toggle on a physical panel on a real machine.

2. Partly because of #1, when use_stdin is True character input processing by the KL11 is not started at device instantiation time; it does not happen until the first I/O access to any KL11 device register occurs in the emulated environment. Prior to that, control-c (and other such host special characters) will work correctly. This deferral will be completely transparent to any emulated program.


## To boot UNIX:
* I have successfully run unix v7 from the HP/RM disk emulation, and unix v5 from the RK05 emulation. unix v6 won't boot and I don't yet know why.

* Obtain a disk image; the ones I am using are hp.disk0 from the unix7 subdirectory of the systems directory from SIMH or v5root.rk from the same place.

### Booting v7 UNIX

* Take the hp.disk0 file and name it rp.disk ... the name is not configurable in the code yet.

* If you are using the socket (default) method for input you will need two separate shell windows. One will be the running python window, where not much happens once you get started. The other will be your emulated console. In the first window, type this:

    python3 boot.py

You may wish to examine 'boot.py' to see how all this works, and how to choose use_stdin if you prefer that method for console emulation.

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

NOTE: If you type a device that is compiled into the bootstrap code, but not present in your emulated machine, the emulation will hang. The real machine does this too (or, at least, SIMH does it). This is a bug/limitation of the unix bootstrap code and not a bug in the emulation.

You will see a single user shell prompt at this point if you are lucky!

Exit the single user shell (type a control-D) to get into multi-user mode. NOTE: The UNIX /etc/ttys file defaults to enabling four additional login lines but the emulation that is implemented (see dc11.py) simply makes them appear to the kernel as "no terminal connected". The (emulated) console is the only serial device supported so far.


Enjoy!

### Booting v5 UNIX

The same discussion of two shell windows, or use_stdin mode applies. In whatever window is your console window type this:

    python3 boot.py --drive v5root.rk --rk

You may wish to examine 'boot.py' to see how all this works. If you named your copy of the simh RK05 image something else, use that in place of "v5root.rk". The "--rk" option creates an 11/70 with RK05 drives instead of the hp/RM80. You can, of course, construct a machine with both (or multiple drives) but see the boot.py and machine.py code for how to do that.

In your console window you should get a prompt: '@' ... type:

   unix

at that '@' prompt and v5 unix will boot up. Login as root, no password.


## Understanding the code
I'll have to write more about this later; as a first step to getting around:

* machine.py is the heart of the instruction loop and CPU basics.
* Actual machine instructions have been split off - start with op4.py and trace the opparsing/dispatch from there.
* mmu.py implements, surprisingly enough, the PDP-11 MMU system.
* unibus.py handles the memory mapped Unibus I/O.
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
