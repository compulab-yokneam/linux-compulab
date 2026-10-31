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

Carrier features use separate ``sbc-mcm8pv2-*.dtsi`` files and matching
``.dtso`` wrappers, following the SBEV-UCM-iMX8Plus layout. Shared carrier
rails, I2C3, EEPROM and the GPIO expander remain in ``sbc-mcm8pv2.dtsi``:

* EQOS uses RTL8211F at MDIO address 4 with active-low GPIO1_IO01 reset.
* FEC uses KSZ9131 at MDIO address 7 with active-low GPIO4_IO24 reset on
  SAI2_TXFS. Address 7 was verified working in U-Boot. The carrier
  schematic annotates address 5: TBD: to talk to the HW engineer.
* Both MACs use RGMII with PHY RX/TX delays enabled.
* I2C3 hosts the PCA9555 at 0x20 and carrier EEPROM at 0x54.
* PCA9555 IO0.2 resets DSI, and IO0.3 releases LVDS display reset.
* HDMI at P22 uses native DDC, HPD and CEC signals with fixed 5 V power
  and hardware-enabled level shifters. LCDIF3, HDMI PAVI, the transmitter,
  PHY, interrupt steering and AUD2HTX HDMI audio are enabled.
* LVDS backlight uses SPDIF_RX/PWM2; DSI backlight uses SPDIF_TX/PWM3.
  Both use a 50,000 ns period (20 kHz).
* The LVDS overlay includes the verified Goodix GT911 touchscreen on I2C3
  at 0x5d, with GPIO4_IO28 interrupt and PCA9555 IO0.4 reset release.

These definitions follow MCM-IMX8PV2_R1V0_12_CL2_T.PDF and
SBC-MCM8PV2_REV1V0.pdf. They retain the module EEPROM's 24C08 symbol
identification; the module schematic's part annotation names CAT24C02,
so confirm the installed part when finalizing the assembly description.
Unverified carrier peripherals such as SD, Wi-Fi, PCIe and
USB host are left disabled.

Feature overlays
----------------

``mcm-imx8pv2-base.dtb`` contains the module and shared carrier devices.
Apply any of these overlays to that base; they do not depend on each other:

* ``sbc-mcm8pv2-enet.dtbo``: EQOS/RTL8211F.
* ``sbc-mcm8pv2-enet1.dtbo``: FEC/KSZ9131.
* ``sbc-mcm8pv2-lvds.dtbo``: LVDS panel, PWM2 backlight and Goodix touch.
* ``sbc-mcm8pv2-mipi.dtbo``: MIPI-DSI panel and PWM3 backlight.
* ``sbc-mcm8pv2-hdmi.dtbo``: HDMI video and audio.
* ``sbc-mcm8pv2-wm8962.dtbo``: analog audio codec on SAI3/I2C3.
* ``sbc-mcm8pv2-p17_can.dtbo``: FlexCAN2 and the ATA6563 transceiver.
* ``sbc-mcm8pv2-p19_uart-b.dtbo``: UART4 on P19.
* ``sbc-mcm8pv2-p20_uart-a.dtbo``: independent UART1 and UART3 on P20.
* ``sbc-mcm8pv2-p18_gpio.dtbo``: all six GPIO signals on P18.
* ``sbc-mcm8pv2-p15_spi.dtbo``: ECSPI2 on P15.
* ``sbc-mcm8pv2-p16_spi.dtbo``: ECSPI1 on P16.
* ``sbc-mcm8pv2-p16_i2c.dtbo``: I2C4 on P16.

For ``mcm-imx8pv2-som.dtb``, first apply ``sbc-mcm8pv2.dtbo`` to add
shared carrier devices, then the required feature overlays. As on SBEV,
each wrapper records its name in ``/chosen/overlays``. Do not apply a
feature overlay again to a DTB which already includes it.

The Makefile also assembles these ready-to-boot DTBs:

* ``sbc-mcm8pv2-headless.dtb``: carrier, both Ethernet ports, WM8962, CAN,
  header UARTs, GPIO, SPI and I2C.
* ``sbc-mcm8pv2-hdmi.dtb``: headless plus HDMI.
* ``sbc-mcm8pv2-lvds.dtb``: headless plus LVDS.
* ``sbc-mcm8pv2-mipi.dtb``: headless plus MIPI-DSI.
* ``sbc-mcm8pv2-allheads.dtb``: headless plus LVDS, HDMI and MIPI-DSI.
* ``mcm-imx8pv2.dtb``: headless plus LVDS and HDMI, preserving the default
  U-Boot filename. MIPI-DSI is disabled in this default configuration.

For example, assemble Ethernet and analog audio without displays::

    fdtoverlay -i mcm-imx8pv2-base.dtb -o selected.dtb \
        sbc-mcm8pv2-enet.dtbo sbc-mcm8pv2-enet1.dtbo sbc-mcm8pv2-wm8962.dtbo

Carrier headers
---------------

These overlays follow carrier schematic sheet 10. All seven can be used
together and with the Ethernet, audio and display overlays. They are also
included in the headless DTB, so every combined carrier variant includes
them. The base and module-only DTBs leave these interfaces disabled or
unclaimed until the selected overlays are applied before Linux boots.

CAN uses FlexCAN2 on SAI2_TXD0 (TX) and SAI2_MCLK (RX). U59 is an ATA6563
powered from 5 V with 3.3 V I/O; R325 pulls STBY low, so no standby GPIO is
needed. P17.2 is CAN_H and P17.6 is CAN_L. The schematic shows a 120-ohm
termination through E6; check the fitted termination when wiring a bus.
After loading ``flexcan``, configure the bus bitrate with ``ip link`` and
test with an external CAN peer. The controller supports CAN FD; actual
operation still requires hardware validation.

P19 exposes UART4 RX/TX on pins 2/4 (``/dev/ttymxc3``). P20 exposes UART1
RX/TX on pins 2/4 (``/dev/ttymxc0``) and UART3 RX/TX on pins 6/3
(``/dev/ttymxc2``). These are separate UARTs, without hardware flow control.
UART2 remains the debug console. The connector signals use 3.3 V logic.

P18 GPIO line names map directly to schematic pin numbers:

========  ==========  ==========
Pin       GPIO        Line name
========  ==========  ==========
3         GPIO1_IO10  P18P3
4         GPIO1_IO00  P18P4
5         GPIO2_IO11  P18P5
6         GPIO4_IO25  P18P6
7         GPIO2_IO05  P18P7
9         GPIO2_IO04  P18P9
========  ==========  ==========

Use ``gpioinfo`` and libgpiod to find and request these lines by name. The
overlay selects their GPIO mux but does not request a direction or value.
P18.4 is also connected to ALT_BOOT/SW1; account for its boot strap role
and the button connection when using it as GPIO. P18.1 is 3.3 V and
P18.10 is ground; P18.2 and P18.8 are not connected.

The SPI connector is P15, not P5 (P5 is an Ethernet jack). ECSPI2 on P15
uses MISO/MOSI/SCLK/CS0 on pins 2/3/5/4. ECSPI1 on P16 uses
MISO/MOSI/SCLK/CS0 on pins 2/6/7/4. CS0 uses the corresponding SS0 pad as
an active-low GPIO. The SPI aliases reserve bus 1 for ECSPI1 and bus 2
for ECSPI2, independently of which overlay is selected.

Following the SBEV SPI example, each overlay includes a userspace SPI
placeholder with ``rohm,dh2228fv`` and a conservative 500 kHz maximum.
This compatible selects the existing spidev driver; it does not describe
an installed ROHM device. With ``spidev`` loaded, the interfaces appear as
``/dev/spidev1.0`` (P16) and ``/dev/spidev2.0`` (P15). Replace the child
node's compatible and properties when attaching a specific SPI peripheral.

P16 also exposes I2C4 SDA/SCL on pins 3/5 (``/dev/i2c-3``). These are
separate from its SPI signals, so the I2C and SPI overlays can be applied
together. I2C runs at 100 kHz and includes GPIO bus-recovery pinctrl.
No external I2C child is assumed. Verify suitable pull-ups for the attached
hardware; the header provides no dedicated external pull-up resistors on
sheet 10. P16.1 is 3.3 V and P16.8 is ground.

Audio codec
-----------

The WM8962B at I2C3 address 0x1a uses SAI3 with a 12.288 MHz MCLK,
following the SBEV ``fsl,imx-audio-wm8962x`` CPU-master setup. The codec
supplies follow carrier schematic sheet 13: AVDD, DCVDD, CPVDD and PLLVDD
use 1.8 V; DBVDD and MICVDD use 3.3 V; SPKVDD1/2 use 5 V.

Audio routing connects HPOUTL/R to the headphone jack and the line-input
jack to IN4L/R. Speaker outputs and microphone inputs are not connected.
The headphone detect signal goes to codec GPIO5. GPIO4_IO22 carries
CODEC_INT and must not be used as the SBEV example's ``hp-det-gpio``.
This overlay provides playback and line capture without automatic jack
reporting. CODEC_INT is left unused, as in the existing SBEV codec setup.

On hardware, check ``aplay -l`` and ``arecord -l`` for ``wm8962-audio``.
Use ``alsamixer`` to enable headphone playback or the IN4 input path,
then test stereo playback and line capture at 48 kHz. The existing
configuration builds the sound-card and codec drivers as modules.

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
HDMI modes are discovered from the connected monitor's EDID. Connect a
monitor to P22 before boot and check the HDMI connector's status and modes
under ``/sys/class/drm/``. The existing kernel configuration provides the
HDMI video/PHY drivers and HDMI audio modules; no new driver is required.

Build and boot
--------------

The existing ``compulab_v8_defconfig`` enables the required Ethernet,
GPIO-expander, display, backlight, FSL ASoC card, SAI and WM8962 drivers. RX8010 support is enabled
in that defconfig and in ``compulab_add.config`` for BSP builds using the
configuration-fragment workflow.

To build the new device trees::

    make O=/tmp/mcm-imx8pv2-linux ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- compulab_v8_defconfig
    make O=/tmp/mcm-imx8pv2-linux ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j8 compulab/mcm-imx8pv2.dtb compulab/mcm-imx8pv2-som.dtb

Build the kernel and modules with the BSP's usual process and deploy the
carrier DTB as ``mcm-imx8pv2.dtb`` beside the kernel, or adjust the boot
script to its installed location. The DTB appears under
``arch/arm64/boot/dts/compulab/`` in the kernel output directory. Yocto
machine metadata lists the base trees, feature overlays and combined DTBs
in ``KERNEL_DEVICETREE`` to package them. The corresponding
``meta-bsp-imx8mp/conf/machine/mcm-imx8pv2.conf`` must be updated alongside
this kernel revision.

On hardware, verify both network interfaces and their PHY addresses,
RX8010 timekeeping, EEPROM access and the enabled DRM connectors and analog audio. Confirm the
backlight PWM pins and display reset polarity during cold/warm boot and
when Linux takes over from U-Boot. Successful device-tree and driver
compilation does not establish Linux hardware operation.
