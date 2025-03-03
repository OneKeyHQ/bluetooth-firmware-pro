#!/usr/bin/env python3

import io
import hashlib
import struct
import zipfile
import click
import re

# input_file = "../artifacts_signed/ota.zip"
# output_file = "../artifacts_signed/ota.bin"

INIT_DATA_SIZE = 512

OFFSET_MAGIC = 0x00
OFFSET_VERSION = 0x10
OFFSET_VERSION_MAJOR = 0x10
OFFSET_VERSION_MINOR = 0x11
OFFSET_VERSION_PATCH = 0x12
OFFSET_HASHES = 0x20
OFFSET_NRF_DAT_SIZE = 0x400
OFFSET_NRF_DAT = 0x404
OFFSET_NRF_BIN_SIZE = 0x0C
OFFSET_NRF_BIN = 0x600

def get_fw_version_bytes(fw_config_h_path:str = "../app/firmware_config.h") -> bytes:
    with open(fw_config_h_path, mode="r") as fw_config:
        for line in fw_config.readlines():
            match = re.match("#define FW_REVISION \"(.*)\"",line)
            if match is not None:
                return bytes(int(ver) for ver in match.groups()[0].split('.'))
        raise Exception("Firmware config header parse error!")

def get_fw_version_splited(fw_config_h_path:str = "../app/firmware_config.h"):
    with open(fw_config_h_path, "r") as file:
        content = file.read()

    match = re.search(r'#define\s+FW_REVISION\s+"([^"]+)"', content)
    if match:
        version_string = match.group(1)
        return map(int, version_string.split('.'))
    else:
        raise Exception("Firmware config header parse error!")

def gen_hashes(data: bytes) -> bytes:
    hashes_buffer: bytearray = io.BytesIO()
    chunk_size = 64 * 1024
    total_size = chunk_size *16 - 1024
    data_index = 0
    
    while data_index < total_size:

        if data_index == 0:
            current_chunk_size = chunk_size - 1024
        else:
            current_chunk_size = chunk_size

        chunk = data[data_index:data_index + current_chunk_size]

        if len(chunk) == current_chunk_size:
            hashes_buffer.write(hashlib.sha256(chunk).digest())
        elif len(chunk) > 0 and len(chunk) < current_chunk_size:
            chunk_padded = chunk + b"\xFF" * (current_chunk_size - len(chunk))  # padding
            hashes_buffer.write(hashlib.sha256(chunk_padded).digest())
        else:
            hashes_buffer.write(b"\x00" * 32)

        data_index+=current_chunk_size

    hashes: bytes = hashes_buffer.getvalue()
    hashes_buffer.close()

    return hashes

def gen_onekey_bin(nrf_dat: bytes, nrf_bin: bytes) -> bytes:

    buffer = io.BytesIO()

    buffer.seek(OFFSET_MAGIC, 0)
    buffer.write(b"5283")

    buffer.seek(OFFSET_VERSION, 0)
    buffer.write(get_fw_version_bytes())

    buffer.seek(OFFSET_NRF_DAT_SIZE, 0)
    buffer.write(struct.pack("i", len(nrf_dat)))
    buffer.seek(OFFSET_NRF_DAT, 0)
    buffer.write(nrf_dat)

    buffer.seek(OFFSET_NRF_BIN_SIZE, 0)
    buffer.write(struct.pack("i", INIT_DATA_SIZE + len(nrf_bin)))
    buffer.seek(OFFSET_NRF_BIN, 0)
    buffer.write(nrf_bin)

    buffer.seek(OFFSET_HASHES, 0)
    buffer.write(gen_hashes(buffer.getvalue()[OFFSET_NRF_DAT_SIZE:]))

    data: bytes = buffer.getvalue()

    return data

@click.command()
@click.argument("input_file", type=str, default='../artifacts_signed/ota.zip')
@click.argument("output_file", type=str, default='../artifacts_signed/ota.bin')
def main(input_file:str, output_file:str):

    print(f'Creating {output_file} from {input_file}')

    file_in = zipfile.ZipFile(input_file, "r")

    nrf_dat: bytes = b""
    nrf_bin: bytes = b""

    for f in file_in.namelist():
        if f.endswith(".dat"):
            nrf_dat: bytes = file_in.read(f)
        if f.endswith(".bin"):
            nrf_bin: bytes = file_in.read(f)

    assert len(nrf_dat) != 0 and len(nrf_bin) != 0

    print(f'Validated {input_file}')

    onekey_bin: bytes = gen_onekey_bin(nrf_dat, nrf_bin)
    file_out = open(output_file, "wb")
    file_out.write(onekey_bin)
    file_out.close()

    print(f'Created {output_file} ({len(onekey_bin)} bytes)')


if __name__ == "__main__":
    main()
