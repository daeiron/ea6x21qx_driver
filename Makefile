PATH := $(HOME)/coreelec/CoreELEC/build.CoreELEC-Amlogic-no.aarch64-22/toolchain/lib/clang/bin:$(PATH)
SKW_SRC_ROOT := $(shell pwd)
SKW_EXTRA_INC := $(SKW_SRC_ROOT)/include/linux/platform_data
SKW_EXTRA_SYMBOLS := $(SKW_SRC_ROOT)/drivers/seekwaveplatform/Module.symvers
ARCH := arm64
KERNEL_SRC := $(HOME)/coreelec/CoreELEC/build.CoreELEC-Amlogic-no.aarch64-22/build/linux-2c3e329b7dec94edae5e9fb5f33669239c384436

.PHONY: all skw_wifi skw_bsp modules_install clean

all: skw_bsp skw_wifi
	@echo "seekwave modules build complete"

skw_wifi: skw_bsp
	$(MAKE) INSTALL_MOD_STRIP=1 ARCH=$(ARCH) -C $(KERNEL_SRC) \
		M=$(PWD)/drivers/skwifi modules \
		CONFIG_WLAN_VENDOR_SEEKWAVE=m CONFIG_SKW_VENDOR=m \
		LLVM=1 \
		skw_extra_flags="-I$(SKW_EXTRA_INC) -I$(SKW_SRC_ROOT)/drivers/skwifi -include $(SKW_EXTRA_INC)/skw6160_config.h" \
		skw_extra_symbols=$(SKW_EXTRA_SYMBOLS)

skw_bsp:
	$(MAKE) INSTALL_MOD_STRIP=1 ARCH=$(ARCH) -C $(KERNEL_SRC) \
		M=$(PWD)/drivers/seekwaveplatform modules \
		CONFIG_SKW_SDIOHAL=m CONFIG_SEEKWAVE_BSP_DRIVERS=y CONFIG_SKW_BSP_UCOM=m CONFIG_SKW_BSP_BOOT=m \
		LLVM=1 \
		skw_extra_flags="-I$(SKW_EXTRA_INC)" \
		skw_extra_symbols=$(SKW_EXTRA_SYMBOLS)

modules_install: all
	mkdir -p $(SKW_SRC_ROOT)/modules/
	find $(SKW_SRC_ROOT)/drivers/seekwaveplatform -name "*.ko" -exec cp {} $(SKW_SRC_ROOT)/modules/ \;
	find $(SKW_SRC_ROOT)/drivers/skwifi -name "*.ko" -exec cp {} $(SKW_SRC_ROOT)/modules/ \;


clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SKW_SRC_ROOT)/drivers/skwifi clean
	$(MAKE) -C $(KERNEL_SRC) M=$(SKW_SRC_ROOT)/drivers/seekwaveplatform clean
	rm -rf $(SKW_SRC_ROOT)/modules
	@echo "seekwave modules cleaned"
