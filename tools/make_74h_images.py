#!/usr/bin/env python3
"""Chroma 74H release images from the bootloader and app builds.

    make_74h_images.py <boot.bin> <app.bin> <version> [--hwtype 0x81] [--out DIR]

boot.bin: bootloader_CC1310 as a 64 KB flat image (tools/out2bin.sh 0x0 0x10000)
app.bin:  oepl_CC1310 CONFIG=Chroma74H from 0x1000 (out2bin.sh 0x1000 0xE000)

Writes
  chroma74h_full_<ver>.bin  64 KB JTAG image: bootloader + app + CCFG
                            (dslite --mode flash -e -f -v <file>,0x0)
  chroma74h_ota_<ver>.bin   OTA file for the AP: OtaHeader + app image
                            (layout in oepl_CC1310/ota_74h.h)

<version> must be the FW_VERSION the app was built with: the app refuses an
OTA file whose version equals its own.
"""
import argparse
import os
import struct
import sys
import zlib

APP_BASE = 0x1000
APP_MAX = 0xE000
FLASH_SIZE = 0x10000
CCFG_ADR = 0xFFA8
OTA_FILE_MAGIC = 0x3437414F   # "OA74"
HEADER_FMT = '<IBBHII'       # Magic, HwType, HeaderLen, Version, ImageLen, ImageCrc
HEADER_LEN = struct.calcsize(HEADER_FMT) + 4


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('boot')
    ap.add_argument('app')
    ap.add_argument('version', type=lambda s: int(s, 0))
    ap.add_argument('--hwtype', type=lambda s: int(s, 0), default=0x81)
    ap.add_argument('--out', default='.')
    a = ap.parse_args()

    boot = open(a.boot, 'rb').read()
    app = open(a.app, 'rb').read()
    if len(boot) != FLASH_SIZE:
        sys.exit('boot.bin must be the 64 KB flat image')
    if any(b != 0xFF for b in boot[APP_BASE:CCFG_ADR]):
        sys.exit('bootloader image has data outside sector 0 / the CCFG')
    image = app[:APP_MAX].rstrip(b'\xff')
    image += b'\xff' * (-len(image) % 4)
    sp, pc = struct.unpack_from('<II', image)
    if not (0x20000000 < sp <= 0x20004000 and pc & 1 and APP_BASE < pc < APP_BASE + len(image)):
        sys.exit('app vectors look wrong: SP 0x%08x reset 0x%08x' % (sp, pc))

    full = bytearray(boot)
    full[APP_BASE:APP_BASE + len(image)] = image
    crc = zlib.crc32(image) & 0xFFFFFFFF
    hdr = struct.pack(HEADER_FMT, OTA_FILE_MAGIC, a.hwtype, HEADER_LEN, a.version,
                      len(image), crc)
    hdr += struct.pack('<I', zlib.crc32(hdr) & 0xFFFFFFFF)

    os.makedirs(a.out, exist_ok=True)
    full_name = os.path.join(a.out, 'chroma74h_full_%04x.bin' % a.version)
    ota_name = os.path.join(a.out, 'chroma74h_ota_%04x.bin' % a.version)
    open(full_name, 'wb').write(full)
    open(ota_name, 'wb').write(hdr + image)
    print('%s: %d bytes' % (full_name, len(full)))
    print('%s: %d bytes (image %d bytes, CRC32 0x%08x, version %d, hwType 0x%02x)'
          % (ota_name, len(hdr) + len(image), len(image), crc, a.version, a.hwtype))


if __name__ == '__main__':
    main()
