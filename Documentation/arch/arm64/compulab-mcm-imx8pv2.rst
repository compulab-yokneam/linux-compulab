.. SPDX-License-Identifier: GPL-2.0+

CompuLab MCM-iMX8Pv2 and SBC-MCM8PV2
===================================

Device trees
------------

``compulab/mcm-imx8pv2.dtb`` describes MCM-iMX8Pv2 on SBC-MCM8PV2. Its
filename matches the default ``fdtfile`` used by the corresponding U-Boot
target. ``compulab/mcm-imx8pv2-som.dtb`` describes the module alone, without
carrier Ethernet or display devices.

The module description is in ``arch/arm64/boot/dts/compulab/mcm-imx8pv2.dtsi``.
It includes UART2, PCA9450C on I2C1 at 0x25, EEPROM on I2C2 at 0x50,
RX8010 RTC on I2C2 at 0x32, eight-bit USDHC3 eMMC, the GPIO1_IO12 LED,
WDOG1 and USB1 peripheral mode. The kernel MMC aliases retain eMMC as
mmc2. U-Boot supplies the installed DRAM ranges through its memory fixup.

Carrier peripherals are kept in ``sbc-mcm8pv2.dtsi``:

* EQOS uses RTL8211F at MDIO address 4 with active-low GPIO1_IO01 reset.
* FEC uses KSZ9131 at MDIO address 7 with active-low GPIO4_IO24 reset on
  SAI2_TXFS. Address 7 was verified working in U-Boot. The carrier
  schematic annotates address 5: TBD: to talk to the HW engineer.
* Both MACs use RGMII with PHY RX/TX delays enabled.
* I2C3 hosts the PCA9555 at 0x20 and carrier EEPROM at 0x54.
* PCA9555 IO0.2 resets DSI, and IO0.3 releases LVDS display reset.
* LVDS backlight uses SPDIF_RX/PWM2; DSI backlight uses SPDIF_TX/PWM3.
  Both use a 50,000 ns period (20 kHz).

These definitions follow MCM-IMX8PV2_R1V0_12_CL2_T.PDF and
SBC-MCM8PV2_REV1V0.pdf. They retain the module EEPROM's 24C08 symbol
identification; the module schematic's part annotation names CAT24C02,
so confirm the installed part when finalizing the assembly description.
Unverified carrier peripherals such as SD, Wi-Fi, PCIe, audio, touch and
USB host are left disabled.

Displays
--------

The panel definitions retain provisional assumptions from the U-Boot
configuration. The LVDS panel reuses the predecessor's
``boe,hv070wsa-100`` driver definition: 1024x600 at 42.105 MHz, 154x90 mm
and its native 24-bit SPWG mapping. U-Boot currently assumes JEIDA for
this output; confirm the actual panel mapping before display bring-up
and align both configurations if necessary. The DSI panel
uses the existing ``startek,kd050hdfia020`` driver, four lanes and explicit
720x1280 timing at 60 MHz to match U-Boot.

The carrier schematic defines connectors and backlight circuits, not the
attached panel models. Confirm the actual panels, LVDS bit mapping,
physical dimensions and DSI initialization sequence before relying on
these display defaults. A different panel may require another compatible
or driver. Check each connector separately on hardware. Linux uses its
DRM display drivers and does not use U-Boot's ``video_link`` variable.

Build and boot
--------------

The existing ``compulab_v8_defconfig`` enables the required Ethernet,
GPIO-expander, display and backlight drivers. RX8010 support is enabled
in that defconfig and in ``compulab_add.config`` for BSP builds using the
configuration-fragment workflow.

To build the new device trees::

    make O=/tmp/mcm-imx8pv2-linux ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- compulab_v8_defconfig
    make O=/tmp/mcm-imx8pv2-linux ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j8 compulab/mcm-imx8pv2.dtb compulab/mcm-imx8pv2-som.dtb

Build the kernel and modules with the BSP's usual process and deploy the
carrier DTB as ``mcm-imx8pv2.dtb`` beside the kernel, or adjust the boot
script to its installed location. The DTB appears under
``arch/arm64/boot/dts/compulab/`` in the kernel output directory. Yocto
machine metadata must include this DTB in ``KERNEL_DEVICETREE`` to package
it; this source-tree change does not modify external BSP metadata.

On hardware, verify both network interfaces and their PHY addresses,
RX8010 timekeeping, EEPROM access and both DRM connectors. Confirm the
backlight PWM pins and display reset polarity during cold/warm boot and
when Linux takes over from U-Boot. Successful device-tree and driver
compilation does not establish Linux hardware operation.
