bcc-kmod
========

Kernel module for HidaV board controller communication

_Description:_
The HidaV board controller is a microcontroller that is serially attached to the main ARM processor.
In order to use the functionality you have to enable DARMOK_CORE in the kernel config file. 

The kernel module and compiled-in code provide with an optional legacy tty interface named 'drbcc-raw' and alters the functionality of the rtc device.
The darmok core component is implemented as a tty line discipline.

You will need the patched kernel of the HidaV project to execute the darmok code.
