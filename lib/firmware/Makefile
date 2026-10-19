# Resolve relative O= paths from the kernel source root.
kernel_dir := $(abspath ../..)
output_dir := $(if $(O),$(if $(filter /%,$(O)),$(O),$(kernel_dir)/$(O)),$(kernel_dir))
KCONFIG_CONFIG ?= $(output_dir)/.config
-include $(KCONFIG_CONFIG)

files := imx/sdma/sdma-imx7d.bin
selected := $(filter $(files),$(subst ",,$(CONFIG_EXTRA_FIRMWARE)))

.PHONY: all clean
all: $(selected)

ifneq ($(selected),)
$(selected): %: %.arch
	gunzip -c $< > $@
endif

clean:
	rm -rf $(files)
