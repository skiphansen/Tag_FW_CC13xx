#!/bin/bash
# out2bin.sh <in.out> <out.bin> <origin> <length>: flat binary of a TI .out over a
# flash range, gaps filled with 0xFF (TI armhex)
set -e
CGT="${CGT:-$HOME/ti/ti-cgt-arm_18.12.8.LTS}"
TMP=$(mktemp -d)
cat > "$TMP/r.cmd" <<END_OF_CMD
-b
-image
--memwidth=8
--romwidth=8
--fill=0xFFFFFFFF
ROMS
{
   all: origin=$3, length=$4, romwidth=8, fill=0xFFFFFFFF
}
END_OF_CMD
"$CGT/bin/armhex" -q "$1" "$TMP/r.cmd" -o "$2"
rm -rf "$TMP"
