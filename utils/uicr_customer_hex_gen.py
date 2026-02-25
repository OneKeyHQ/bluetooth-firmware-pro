#!/usr/bin/env python3
import argparse
from intelhex import IntelHex

parser = argparse.ArgumentParser(
    description="Generate NRF52 UICR CUSTOMER hex file."
)
parser.add_argument("-o", "--output", required=True, help="write output to a file")
parser.add_argument("--battery-profile", type=str, default="JSEL", help="battery profile flag, 4 ASCII chars (default: JSEL)")
args = parser.parse_args()

UICR_CUSTOMER_31_ADDR = 0x100010FC # UICR_CUSTOMER[31] is used to store battery profile flag, which is 4 ASCII characters

if __name__ == "__main__":

    if len(args.battery_profile) != 4 or not args.battery_profile.isascii():
        raise SystemExit('Error: battery profile flag must be exactly 4 ASCII characters!')
    battery_profile_flag = int.from_bytes(args.battery_profile.encode('ascii'), byteorder='big')

    print(f"Battery Profile: {args.battery_profile} ({hex(battery_profile_flag)})")

    intel_hex = IntelHex()
    intel_hex.frombytes(
        bytes=(battery_profile_flag).to_bytes(4, byteorder="big"), offset=UICR_CUSTOMER_31_ADDR
    )
    intel_hex.dump()
    intel_hex.write_hex_file(args.output)
