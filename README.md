SeekWave EA6x21QX driver and firmware

Makefile for building out of coreelec source.

To build:

Edit these two variables in Makefile:

PATH := $(HOME)/coreelec/CoreELEC/build.CoreELEC-Amlogic-no.aarch64-22/toolchain/lib/clang/bin:$(PATH)
KERNEL_SRC := $(HOME)/coreelec/CoreELEC/build.CoreELEC-Amlogic-no.aarch64-22/build/linux-2c3e329b7dec94edae5e9fb5f33669239c384436

PATH should point to the coreelec clang toolchain (toolchain/lib/clang/bin)
KERNEL_SRC should point to the coreelec kernel source (build.CoreELEC*/build/linux-*)
