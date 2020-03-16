Notes on this RISC-V port
-------------------------

Currently the steps necessary to build are a bit wonky.  It's not a fully integrated build yet, and the build
is specific to the combination of (1) RISC-V single core.  (2) Freedom E SDK and Freedom Metal as the bare metal
environment (interrupt handler framework and I/O APIs). (3) UART I/O available on the Arty 100T.

There are 3 general areas of code for this combination.
1) MRI itself.  The source code is in the mri submodule.  We're building this as a static library, and then linking it in to main application outside of "mri".
2) Freedom E SDK and Freedom Metal (the top-level freedom-e-sdk directory and all other subdirectories except "mri").  If you're using an RTOS, then it's possible that no code in Freedom E SDK or Freedom Metal would be applicable, and you'd just want to concentrate on getting the "mri" subdirectory to build/link/run with your RTOS environment.  The mri/notes/mri-porting.creole file is the information on porting MRI to a new board.  Also, link errors against your application are a pretty good hint showing which functions you need to supply.  The mri/architectures/riscv directory is intended to hold general RISC-V related functionality without being board-specific.

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
