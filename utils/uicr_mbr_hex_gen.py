#!/usr/bin/env python3
import argparse
from elftools.elf.elffile import ELFFile
from intelhex import IntelHex

parser = argparse.ArgumentParser(
    description="Generate NRF52 UICR MBR hex file from dfu binary."
)
parser.add_argument("-i", "--input", required=True, help="the file to be converted")
parser.add_argument("-o", "--output", required=True, help="write output to a file")
parser.add_argument("--use-mbr", type=bool, action=argparse.BooleanOptionalAction, default=True, help="use MBR")
parser.add_argument("--use-uicr", type=bool, action=argparse.BooleanOptionalAction, default=True, help="use UICR")
args = parser.parse_args()

MBR_BOOTLOADER_ADDR = 0xFF8
MBR_PARAM_PAGE_ADDR = 0xFFC
UICR_BOOTLOADER_ADDR = 0x10001014
UICR_PARAM_PAGE_ADDR = 0x10001018

if __name__ == "__main__":

    dfu_addr = 0
    mbr_param_addr = 0
    intel_hex_uicr = IntelHex()
    intel_hex_mbr = IntelHex()
    intel_hex_merged = IntelHex()

    if (not args.use_mbr) and (not args.use_uicr):
        raise SystemExit('Error: MBR and UICR cannot both be disabled!')

    with open(args.input, "rb") as f:
        elf_file = ELFFile(f)
        dfu_addr = elf_file.get_section_by_name(".text").header["sh_addr"]
        mbr_param_addr = elf_file.get_section_by_name(".mbr_params_page").header["sh_addr"]

    print(f"DFU Address: {hex(dfu_addr)}")
    print(f"MBR Param Address: {hex(mbr_param_addr)}")

    if args.use_mbr:
        print("MBR HEX Dump: ")
        intel_hex_mbr.frombytes(
            bytes=(dfu_addr).to_bytes(4, byteorder="little"), offset=MBR_BOOTLOADER_ADDR
        )
        intel_hex_mbr.frombytes(
            bytes=(mbr_param_addr).to_bytes(4, byteorder="little"), offset=MBR_PARAM_PAGE_ADDR
        )
        intel_hex_mbr.dump()

    if args.use_uicr:
        print("UICR HEX Dump: ")
        intel_hex_uicr.frombytes(
            bytes=(dfu_addr).to_bytes(4, byteorder="big"), offset=UICR_BOOTLOADER_ADDR
        )
        intel_hex_uicr.frombytes(
            bytes=(mbr_param_addr).to_bytes(4, byteorder="big"), offset=UICR_PARAM_PAGE_ADDR
        )
        intel_hex_uicr.dump()

    intel_hex_merged.merge(intel_hex_mbr)
    intel_hex_merged.merge(intel_hex_uicr)

    intel_hex_merged.write_hex_file(args.output)
