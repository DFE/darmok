darmok
======

Darmok is a device subsystem which provides interfaces for communication with
the [HIPOS](/DFE/HIPOS) board controller via the serial line.

Description
------------
The [HIPOS](/DFE/HIPOS) board controller is a microcontroller that is serially
attached to the main ARM processor.  You will need the patched kernel of the
[HIPOS](/DFE/HIPOS) project to execute the darmok code.  In order to use its
functionality you have to enable `DARMOK_CORE` in the kernel config file.  All
of the high level darmok kernel drivers depend on drbcc-core.

The kernel module and compiled-in code provide an optional legacy tty interface
named 'drbcc-raw' which can be used with the userspace drbcc commandline tool
of the [HIPOS](/DFE/HIPOS) project.  If activated in the kernel config it also
provides an interface to the rtc device (if applicable) and the hardware
watchdog.  The darmok core component is implemented using a tty line
discipline.

    +-------------+-----------------+----------------+
    | hwclock, .. | commandline-    | watchdog,..    |        user space
    |             | drbcc-tool      |                |
    +-------------+-----------------+----------------+

    +-------------+-----------------+----------------+
    | drbcc-rtc   | drbcc-raw       | drbcc-watchdog |        kernel space
    +-------------+-----------------+----------------+
    |               drbcc-core                       |
    +------------------------------------------------+
    |               serial driver (8250.c)           |
    +------------------------------------------------+

    +------------------------------------------------+
    |               board controller                 |        hardware
    +------------------------------------------------+

Programming of new device drivers
----------------------------------
The module drbcc-core provides an interface to the board controller
functionality.  To exchange messages with the board controller the appropriate
values in `struct bcc_packet` need to be set.

_Required entries:_
 * **cmd**: see [drbcc_ll.h](/DFE/darmok/blob/master/drbcc-kmod/drbcc-kmod-sources/drbcc_ll.h) for command options
 * **data**: depending on command (see [drbcc_ll.h](/DFE/darmok/blob/master/drbcc-kmod/drbcc-kmod-sources/drbcc_ll.h))
 * **payloadlen**: number of data bytes passed with the command

The other fields must not be filled since they are only used internally in
drbcc-core.

The module code should then call the darmok-core function
[transmit_packet()](/DFE/darmok/blob/master/drbcc-kmod/drbcc-kmod-sources/drbcc-core.c#L454)
which will block until the transaction has been completed.  The board
controller response is filled into struct
[bcc_packet](/DFE/darmok/blob/master/drbcc-kmod/drbcc-kmod-sources/drbcc.h#L70)
which is passed to
[transmit_packet()](/DFE/darmok/blob/master/drbcc-kmod/drbcc-kmod-sources/drbcc-core.c#L454)
as a function argument.

Devices
-------
The darmok driver subsystem provides several device drivers.

The legacy interface /dev/drbcc-raw allows the use of the command-line
drbcc-tool which supports the usage of all  commands the board controller
supports.  It resembles the communication of the command line tool over the
serial interface which was used before work on darmok began.  It only works
with the [HIPOS](/DFE/HIPOS) drbcc command line tool.

The watchdog driver is represented by the device file /dev/watchdog.  It
supports the system calls `WDIOC_SETOPTIONS` (supported flags:
`WDIOS_DISABLECARD`, `WDIOS_ENABLECARD`), `WDIOC_KEEPALIVE`,
`WDIOC_SETTIMEOUT`, and `WDIOC_GETTIMEOUT`.  If the user space watchdog calls
`WDIOS_DISABLECARD` the kernel watchdog is in charge of kicking the watchdog.

The RTC device driver is represented by /dev/rtcX, where X is a number
depending on the kernel configuration.  If, for example, there is another RTC
device configured to be compiled into the kernel, this device might be
registered first with the RTC device subsystem and named /dev/rtc0.  Darmok's
RTC device would then be represented by /dev/rtc1.

Further device drivers (e.g. gpio) will eventually be implemented using
drbcc-core as an interface.
