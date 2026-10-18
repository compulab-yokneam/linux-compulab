CompuLab standalone FIT image
============================

The ``fitImage`` target builds an ARM64 kernel and the selected device trees,
then packages them with SHA-256 hashes using the host ``mkimage`` and ``dtc``
tools. Python 3 is also required. It works without BitBake or the Yocto Python
modules. Configure the kernel for your board first (including
``CONFIG_SOC_IMX9=y`` for the IOT-LINK device trees), then merge the platform
FIT settings fragment.

From the kernel source directory::

    make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- fitimage-imx93.config
    make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j8 fitImage

Outputs are ``arch/arm64/boot/fitImage`` and ``arch/arm64/boot/fit-image.its``.
With ``O=/path/to/build``, both are under that build directory instead.
The target builds only the selected DTBs/overlays. It is opt-in; the default
kernel build is unchanged.

The ``arch/arm64/configs/fitimage-imx93.config`` fragment follows
``iot-link.conf`` and the ``linux-yocto-fitimage`` recipe:
a gzip-compressed ``Image.gz``, three base DTBs (``iot-link.dtb``,
``iot-link-ml.dtb``, ``iot-link-cam.dtb``), and the ``iot-link-fars485.dtbo``,
``iot-link-fbrs485.dtbo``, ``iot-link-facan.dtbo`` and ``iot-link-fbcan.dtbo``
overlays. Images use ``kernel-1``, ``fdt-<basename>`` and, when supplied,
``ramdisk-1`` nodes. Each device tree has a ``conf-<basename>`` configuration;
``conf-iot-link.dtb`` is the default. For example, in U-Boot::

    bootm ${loadaddr}#conf-iot-link.dtb
    bootm ${loadaddr}#conf-iot-link.dtb#conf-iot-link-facan.dtbo

Load the FIT at a suitable address before these commands. The kernel's load
and entry addresses default to ``0x80400000``, the DTB load address to
``0x93000000``, and the overlay load address to ``0x93100000``.

The fragment sets ``CONFIG_FIT_DEVICETREE``, ``CONFIG_UBOOT_LOADADDRESS``,
``CONFIG_UBOOT_ENTRYPOINT``, ``CONFIG_UBOOT_DTB_LOADADDRESS`` and
``CONFIG_UBOOT_DTBO_LOADADDRESS`` as strings. These are Kconfig options, so
they survive ``olddefconfig`` and can be edited under "FIT image settings"
in ``menuconfig``. For another platform, supply an appropriate fragment or
set the options there; the architecture Makefile has no board-specific values.
Use the same ``O=`` directory for configuration and build commands when
building outside the source tree.

Make variables
--------------

``FIT_DEVICETREE``
    Space-separated DTB/DTBO paths relative to ``arch/arm64/boot/dts``.
    Basenames must be unique. Defaults to ``CONFIG_FIT_DEVICETREE``.
``FIT_CONF_DEFAULT_DTB``
    Basename of the default base DTB; defaults to the first selected file.
``FIT_KERNEL_IMAGE``
    ``Image.gz`` (default) or ``Image``. Compression metadata follows the name.
``FIT_INITRAMFS``
    Optional existing initramfs archive. Absolute paths are recommended,
    especially with ``O=``. The archive is embedded as supplied; Linux must
    support its compression format. No root filesystem is built by this target.
``FIT_DESC``
    FIT description, defaulting to Linux and the kernel release.
``FIT_MKIMAGE``
    Path to the host ``mkimage`` executable; defaults to ``mkimage`` on PATH.
``UBOOT_LOADADDRESS``, ``UBOOT_ENTRYPOINT``
    Kernel addresses, defaulting to the corresponding ``CONFIG_`` strings.
``UBOOT_DTB_LOADADDRESS``, ``UBOOT_DTBO_LOADADDRESS``
    Device tree and overlay addresses, also taken from the ``CONFIG_`` strings.

Make command-line overrides still take precedence over Kconfig settings.

For example, include an existing initramfs and a compressed kernel::

    make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j8 fitImage \
        FIT_KERNEL_IMAGE=Image.gz FIT_INITRAMFS=/path/to/rootfs.cpio.gz

Select a single base device tree::

    make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j8 fitImage \
        FIT_DEVICETREE=compulab/iot-link-cam.dtb

Changed inputs and make options trigger image regeneration. ``make clean``
removes both outputs. This target produces an unsigned FIT with hashes;
Yocto signing, configuration mappings, extra loadables and deployment remain
features of the separate Yocto recipe.

Debian kernel package
---------------------

Enable ``CONFIG_FIT_IMAGE=y`` to build and include the FIT in the standard
``linux-image-<kernel-release>`` Debian package. The
``fitimage-imx93.config`` fragment enables this option alongside the IOT-LINK
device trees and load addresses::

    make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- fitimage-imx93.config
    make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j8 bindeb-pkg

The normal kernel image, modules and device trees are packaged as before.
The linux-image package additionally installs ``/boot/fitImage-<kernel-release>``.
Using a versioned filename allows multiple kernel releases to be installed
together. No separate ``compulab-fit-image`` package is generated.

With ``CONFIG_FIT_IMAGE=n`` (the default), ``bindeb-pkg`` does not build or
package a FIT image, even if a previous ``arch/arm64/boot/fitImage`` exists.
The explicit ``make fitImage`` target remains available independently.

The packaging build declares host ``u-boot-tools`` and
``device-tree-compiler`` dependencies only when the option is enabled.
``fitImage`` options such as ``FIT_INITRAMFS`` and ``FIT_KERNEL_IMAGE`` also
apply when passed to ``bindeb-pkg``. Debian package versions can be set with
the existing ``KDEB_PKGVERSION`` option.

Copy the resulting linux-image package to the ARM64 target and install it::

    sudo apt install ./linux-image-<kernel-release>_<version>_arm64.deb

ARM64 i.MX linux-image packages also contain a version-specific CompuLab boot
updater when ``CONFIG_COMPULAB_BOOT_UPDATE=y`` (the default on ARM64 i.MX).
It is called directly by ``postinst configure``. A separate
``compulab-kernel-hooks`` package is no longer needed. The linux-image package
declares a conflict with that legacy package so APT can remove it during the
transition and prevent duplicate boot updates. ``compulab-bindeb-pkg`` is an
alias for the normal binary package build; ``compulab-kernel-hooks-pkg`` is
retained only for legacy installations.

Set ``CONFIG_COMPULAB_BOOT_UPDATE=n`` to omit the updater, its post-install
call, its runtime dependencies and the legacy hook-package conflict.
``CONFIG_FIT_IMAGE`` still independently controls whether the FIT is built
and included in the package; disabling automatic boot updates does not
disable FIT packaging. Other kernel post-install hooks continue to run.
This option is available in ``menuconfig`` as "Deploy CompuLab boot files
after Debian kernel installation" and defaults to disabled on non-i.MX
platforms.

The updater backs up and verifies the existing boot payload before deploying
the kernel and device trees. When the linux-image package contains a FIT, it
also copies ``fitImage-<kernel-release>`` to the boot partition and creates
the relative symlink ``fitImage -> fitImage-<kernel-release>``. FIT copies and
the symlink are verified; deployment failures roll back the managed boot
files and root-filesystem device trees. Older versioned FIT files are retained.
When no FIT is packaged, existing boot-partition FIT files and links are left
unchanged, even if a stale FIT exists on the root filesystem.

The existing device-selection rules still apply: boot updates require root
on partition 2 and deploy to partition 1 of the same device. If that boot
partition is already mounted at ``/boot``, it is updated there; otherwise it
is temporarily mounted. On unsupported root layouts the updater mirrors
device trees into root-filesystem ``/boot`` and skips boot-partition changes.
``COMPULAB_SKIP_BOOT_UPDATE=1`` skips the updater entirely.

The boot filesystem must support symbolic links for FIT deployment; failure
to create the requested link aborts and rolls back the update. Configure
U-Boot to load ``fitImage`` from the boot partition. The package includes
matching kernel modules as part of the normal linux-image payload. U-Boot
environment settings are not changed by the updater.
