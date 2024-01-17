DTC_FLAGS := -@

include $(srctree)/arch/arm64/boot/dts/compulab/Makefile.ucm-imx8mp
include $(srctree)/arch/arm64/boot/dts/compulab/Makefile.iot-imx8mp
include $(srctree)/arch/arm64/boot/dts/compulab/Makefile.som-imx8mp
include $(srctree)/arch/arm64/boot/dts/compulab/Makefile.mcm-imx8mp
include $(srctree)/arch/arm64/boot/dts/compulab/Makefile.ucm-imx93

always-y       := $(dtb-y)
subdir-y       := $(dts-dirs)
clean-files    := *.dtb
