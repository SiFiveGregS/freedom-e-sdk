Notes on this RISC-V port
-------------------------

Currently the steps necessary to build are a bit wonky.  It's not a fully integrated build yet, and the build
is specific to the combination of (1) RISC-V single core.  (2) Freedom E SDK and Freedom Metal as the bare metal
environment (interrupt handler framework and I/O APIs). (3) UART I/O available on the Arty 100T.

There are 3 general areas of code for this combination.
1) MRI itself.  The source code is in the mri submodule.  We're
building this as a static library, and then linking it in to main
application outside of "mri".
2) Freedom E SDK and Freedom Metal (the
top-level freedom-e-sdk directory and all other subdirectories except
"mri").  If you're using an RTOS, then it's possible that no code in
Freedom E SDK or Freedom Metal would be applicable, and you'd just
want to concentrate on getting the "mri" subdirectory to
build/link/run with your RTOS environment.  The
mri/notes/mri-porting.creole file is the information on porting MRI to
a new board.  Also, link errors against your application are a pretty
good hint showing which functions you need to supply.  The
mri/architectures/riscv directory is intended to hold general RISC-V
related functionality without being board-specific.

For other bare metal APIs or RTOS, the "mri" submodule is the part that would still apply.  

Here are the steps to build for the Freedom E-SDK/Freedom Metal environment (though you should use whatever
BSP matches your target hardware, this assumes a BSP called e31_trace_1911_arty).  For that particular bsp,
these steps will build a version to run in RAM rather than from non-volatile memory:

cd mri
make -f makefile-riscv clean
make -f makefile-riscv 
cd ..
make PROGRAM=riscv-mri TARGET=e31_trace_1911_arty LINK_TARGET=scratchpad clean
make PROGRAM=riscv-mri TARGET=e31_trace_1911_arty LINK_TARGET=scratchpad

For porting to other RISC-V boards, if it's easier to localize all the customizations
under the mri subtree, under mri/architectures/riscv, mri/boards, and mri/devices, that
might be the cleanest way.  For Freedom E SDK and Freedom Metal, it wasn't clear how to
easily achieve that sort of structure, hence the pragmatic set of steps above.

******************************************************************************************
How to connect RISC-V GDB to a target that has MRI installed
******************************************************************************************

Invoke GDB on the host machine as usual, then issue the "target remote" command, supplying
the communication port that pertains to the I/O connection between the host and the target.
For instance, if the I/O connection is a USB-to-serial link, then the command would look
something like "target remote /dev/ttyUSB1".   If the I/O connection is via TCP/IP over
Ethernet or wireless, the command would be something like "target remote 10.1.2.3:4444"
(substitute the actual target IP address and the actual listening port number).  Note that
for the integration of MRI with Freedom Metal, a TCP/IP link isn't implemented, just a
USB-to-serial link, so the TCP/IP option would only apply to targets that specifically
have been designed to use that transport, and where the MRI platform-specific stubs have
been supplied that utilize TCP/IP transport (see section below about platform-specific stubs
for non-Freedom-Metal environments).

Once the appropriate "target remote" command has been issued in GDB, then GDB will attach
to MRI on the target, and GDB will then have control over the target, until a subsequent
"continue", "step", or "stepi" command cedes control to the (non-MRI) software running on
the target.


******************************************************************************************
What would need to be done to adapt this software to a RISC-V software environment that is
something other than Freedom Metal (e.g. an RTOS)?
******************************************************************************************

1) Given whatever I/O conduit you choose to use for host/target debugger communication, arrange
for that I/O channel to generate a high priority interrupt upon incoming transmission.  Note that
any code that runs as a result of any interrupt that runs at a higher priority than that of the
debug I/O channel will not be debuggable in this arrangement.

2) Reuse - or supply an analogue for - the assembly routines mri_exception_entry and mri_exception_exit
that are in riscv_mri.S. The mri_exception_entry implementation should save off x1..x31, mepc, mcause,
and mstatus, into __mriRiscVState, set the active flag in __mriRiscVState, and ultimately make a
call to __mriDebugException which is part of MRI proper.  (Basically, do the equivalent of things
that mri_exception_entry is doing). After __mriDebugException returns, mri_exception_exit should
take the previously saved register values in __mriRiscVState, and place them back into the machine
registers, before ultimately executing an MRET instruction to return control to the program being
debugged.

Before implementing your own version wholesale, consider re-using the existing
implementation and replacing the following assembly line with an alternate jump target
that makes sense for your environment (or replace the jump with additional inline code
to fit your environment).

	j __metal_original_exception_handler
	
Ultimately we want to call __mriDebugException, and then when that returns,
to execute mri_exception_exit, so if, in your environment there's nothing else to do
between the end of mri_exception_entry and the calling of __mriDebugException, then you
could replace the "j __metal_original_exception_handler" with "jal __mriDebugException"
and then when execution returns from __mriDebugException, it would fall through to
mri_exception_exit which is where we want to end up anyway once MRI cedes control.

3) Arrange for debug I/O channel interrupts, memory fault exceptions, and debug exceptions
to all vector to __mri_exception_entry, as well (or to vector to functionality equivalent to
__mri_exception_entry).  This is necessary to give the debugger control on asynchronous halt
requests (Ctrl-C) from GDB, to trap faulting memory accesses made on behalf of MRI itself,
and to give MRI control when a breakpoint happens.

4) Analyze all non-static functions prefixed with "Platform_" in
software/riscv-mri/mri_platform_freedom_metal.c, and either copy them verbatim, or re-implement
the ones that don't fit your situation.  The following functions are likely directly portable to
non-Freedom-Metal environments:
Platform_CommCausedInterrupt
Platform_CommShouldWaitForGdbConnect
Platform_CommIsWaitingForGdbToConnec
Platform_CommWaitForReceiveDataToStop
Platform_EnteringDebugger
Platform_WasMemoryFaultEncountered

These functions may need to be re-implemented or adjusted for non-Freedom-Metal environments:
Platform_CommClearInterrupt

These functions almost surely need to be re-implemented for non-Freedom-Metal environments:
Platform_Init
Platform_CommHasReceiveData
Platform_CommReceiveChar
Platform_CommSendChar

