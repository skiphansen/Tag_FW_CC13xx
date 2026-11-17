#!/usr/bin/env python3
"""Decode the factory data in the SPI flash dump of a CC1310 based Chroma tag.

usage: decode_factory.py dump.bin [--png out.png]

Prints the live factory NVRAM records (0x0000/0x1000), the panel command
script (0x2000) and the stored screen slots. --png renders the slots, one row
per slot, B/W plane on the left and the second plane on the right.

Formats were worked out on a Chroma 74H (SN MS10085924C); other CC1310 tags
may differ. The NVRAM record format is the same as on the CC1110 Chroma tags
(see Tag_FW_Chroma), but the record type numbers are not.
"""
import struct
import sys

NVRAM_MAGIC = bytes([0x56, 0x12, 0x09, 0x85])
SCRIPT_MAGIC = bytes([0xce, 0xfa, 0xef, 0xbe])   # 0xBEEFFACE
SCRIPT_RECORDS = 0x2047                          # first record (init segment), format 04 03
SLOTS = (0x15000, 0x2d000)
SLOT_HDR = 0x20

# Record types seen on the 74H. Everything else is stock firmware state.
NVRAM_NAMES = {
    0x80: "tag SN",
    0x81: "panel serial (leading flag byte)",
    0x88: "resolution (w16, h16, ...)",
}


def nvram(d):
    """<type> <len incl. 2 byte header> <data>, 0xff = end. Type 0 records
    have been invalidated, later records win. The stock firmware alternates
    the NVRAM between sector 0 and sector 1."""
    live, where = {}, None
    for a in (0x0000, 0x1000):
        if d[a:a + 4] != NVRAM_MAGIC:
            continue
        where = a
        o = 4
        while o < 0x1000:
            t, l = d[a + o], d[a + o + 1]
            if t == 0xff:
                break
            if l < 2:
                print(f"  zero length record at 0x{a + o:05x}, rest of sector unreadable")
                break
            if t:
                live[t] = d[a + o + 2:a + o + l]
            o += l
    return where, live


def sn(v):
    return f"{chr(v[0])}{chr(v[1])}{v[2]:02x}{v[3]:02x}{v[4]:02x}{v[5]:02x}{chr(v[6])}"


def script(d):
    """01 <len16> <cmd> <data> sends a command. The other record types are
    3 bytes (type, u16 argument). Meanings are from the stock interpreter at
    0xB330 (../chroma74h_reverse_engineering/docs/epd.md)."""
    names = {0x02: "delay ms", 0x03: "wait busy", 0x04: "cmd + zero fill",
             0x05: "wait busy", 0x06: "reset + hidden regs",
             0x07: "image (TRES, VDCS, one plane: 0x10 if byte1 in {2,5} else 0x13)",
             0x08: "read status (cmd 0x81, 1 byte back)",
             0x09: "panel power", 0x0a: "conditional power",
             0x0b: "BTST + PON + DSLP"}
    o = SCRIPT_RECORDS
    while o < 0x3000:
        t = d[o]
        if t == 0x01:
            l = d[o + 1] | d[o + 2] << 8
            print(f"  {o:05x}: cmd {d[o + 3]:02x} " + ' '.join(f"{x:02x}" for x in d[o + 4:o + 3 + l]))
            o += 3 + l
        elif t == 0x00 or t == 0xff:
            print(f"  {o:05x}: end ({t:02x})")
            return
        else:
            arg = d[o + 1] | d[o + 2] << 8
            print(f"  {o:05x}: {t:02x} {names.get(t, '?')} {arg}")
            o += 3


def main():
    d = open(sys.argv[1], 'rb').read()

    where, live = nvram(d)
    if where is None:
        print("No factory NVRAM found")
    else:
        print(f"Factory NVRAM (live copy at 0x{where:04x}):")
        for t, v in sorted(live.items()):
            note = NVRAM_NAMES.get(t, "")
            if t == 0x80 and len(v) == 7:
                note += f" = {sn(v)}"
            elif t == 0x81:
                note += f" = {v[1:].decode(errors='replace')}"
            elif t == 0x88:
                note += f" = {v[0] | v[1] << 8} x {v[2] | v[3] << 8}"
            print(f"  0x{t:02x}: {v.hex(' ')}  {note}")

    if d[0x2000:0x2004] == SCRIPT_MAGIC:
        name = d[0x2006:0x2024].decode(errors='replace')
        print(f"\nPanel script, format {d[0x2004]:02x} {d[0x2005]:02x}, '{name}':")
        script(d)

    print("\nScreen slots:")
    for a in SLOTS:
        size, extra = struct.unpack_from('<II', d, a)
        state = "erased" if d[a:a + SLOT_HDR] == b'\xff' * SLOT_HDR else f"size {size} hdr2 0x{extra:x}"
        print(f"  0x{a:05x}: {state}")

    if '--png' in sys.argv:
        from PIL import Image
        plane = 800 * 480 // 8
        out = Image.new('L', (1600, 480 * len(SLOTS)), 128)
        for i, a in enumerate(SLOTS):
            p = a + SLOT_HDR
            out.paste(Image.frombytes('1', (800, 480), d[p:p + plane]), (0, i * 480))
            out.paste(Image.frombytes('1', (800, 480), d[p + plane:p + 2 * plane]), (800, i * 480))
        out.save(sys.argv[sys.argv.index('--png') + 1])


if __name__ == '__main__':
    main()
