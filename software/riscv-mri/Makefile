# Copyright 2019 SiFive, Inc #
# SPDX-License-Identifier: Apache-2.0 #

PROGRAM ?= riscv-mri

override CFLAGS += -Xlinker --defsym=__stack_size=0x800 -I ../../mri/include -I ../../mri/architectures/riscv

$(PROGRAM): $(wildcard *.c) $(wildcard *.h) $(wildcard *.S)

clean:
	rm -f $(PROGRAM) $(PROGRAM).hex

