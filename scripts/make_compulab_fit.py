#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Assemble a standalone CompuLab FIT using the OpenEmbedded node names."""

import argparse
import json
from pathlib import Path
import re
import subprocess


def address(value):
    number = int(value, 0)
    if not 0 <= number <= 0xffffffff:
        raise argparse.ArgumentTypeError("address must fit in one 32-bit cell")
    return number


def generate(args):
    """Write an ITS with embedded data and SHA-256 hashes, then run mkimage."""
    kernel = Path(args.kernel)
    if kernel.name not in ("Image", "Image.gz"):
        raise ValueError("FIT_KERNEL_IMAGE must be Image or Image.gz")
    dtbs = [Path(name) for name in args.dtbs]
    names = [dtb.name for dtb in dtbs]
    if len(names) != len(set(names)):
        raise ValueError("device tree basenames must be unique")
    if any(not re.fullmatch(r"[a-zA-Z0-9_+.,-]+\.dtbo?", name)
           for name in names):
        raise ValueError("device trees must have valid .dtb or .dtbo names")
    default = args.default_dtb or names[0]
    if default not in names or not default.endswith(".dtb"):
        raise ValueError("default device tree must name an included base .dtb")
    inputs = [kernel, *dtbs]
    if args.initramfs:
        inputs.append(Path(args.initramfs))
    for path in inputs:
        if not path.is_file() or not path.stat().st_size:
            raise ValueError(f"missing or empty input: {path}")

    # JSON quoting also escapes paths and descriptions for DTS strings.
    quote = json.dumps
    lines = ["/dts-v1/;", "/ {",
             f"\tdescription = {quote(args.description)};",
             "\t#address-cells = <1>;", "\timages {"]

    def image(name, path, kind, compression="none", load=None, entry=None):
        lines.extend([f"\t\t{name} {{",
                      f"\t\t\tdescription = {quote(name)};",
                      f"\t\t\tdata = /incbin/({quote(str(path.resolve()))});",
                      f"\t\t\ttype = {quote(kind)};",
                      '\t\t\tarch = "arm64";',
                      f"\t\t\tcompression = {quote(compression)};"])
        if kind != "flat_dt":
            lines.append('\t\t\tos = "linux";')
        for prop, value in (("load", load), ("entry", entry)):
            if value is not None:
                lines.append(f"\t\t\t{prop} = <0x{value:x}>;")
        lines.extend(['\t\t\thash-1 { algo = "sha256"; };', "\t\t};"])

    image("kernel-1", kernel, "kernel",
          "gzip" if kernel.name.endswith(".gz") else "none",
          args.load, args.entry)
    for dtb in dtbs:
        image("fdt-" + dtb.name, dtb, "flat_dt",
              load=args.dtbo_load if dtb.suffix == ".dtbo" else args.dtb_load)
    if args.initramfs:
        # As in OE, the archive may itself be compressed; U-Boot passes it
        # to Linux without decompressing it.
        image("ramdisk-1", Path(args.initramfs), "ramdisk")
    lines.extend(["\t};", "\tconfigurations {",
                  f'\t\tdefault = {quote("conf-" + default)};'])
    for name in names:
        lines.extend([f"\t\tconf-{name} {{",
                      f"\t\t\tdescription = {quote(name)};",
                      '\t\t\tkernel = "kernel-1";',
                      f'\t\t\tfdt = {quote("fdt-" + name)};'])
        if args.initramfs:
            lines.append('\t\t\tramdisk = "ramdisk-1";')
        lines.append("\t\t};")
    lines.extend(["\t};", "};", ""])

    output = Path(args.output)
    its = output.with_name("fit-image.its")
    its.write_text("\n".join(lines), encoding="utf-8")
    # Keep a previous successful image intact if assembly fails.
    temporary = output.with_name(output.name + ".tmp")
    try:
        subprocess.run([args.mkimage, "-f", str(its), str(temporary)], check=True)
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--kernel", required=True)
    parser.add_argument("--description", required=True)
    parser.add_argument("--default-dtb", default="")
    parser.add_argument("--load", type=address, required=True)
    parser.add_argument("--entry", type=address, required=True)
    parser.add_argument("--dtb-load", type=address, required=True)
    parser.add_argument("--dtbo-load", type=address, required=True)
    parser.add_argument("--mkimage", default="mkimage")
    parser.add_argument("--initramfs")
    parser.add_argument("dtbs", nargs="+")
    args = parser.parse_args()
    try:
        generate(args)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"fitImage: {error}\n")


if __name__ == "__main__":
    main()
