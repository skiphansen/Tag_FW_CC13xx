#!/bin/bash
# Chroma 74H release images in one go: clean builds of bootloader_CC1310 and
# oepl_CC1310 CONFIG=Chroma74H, then tools/out2bin.sh and
# tools/make_74h_images.py. Runs on macOS (the x86 TI compiler runs under
# Rosetta on Apple Silicon), Linux and WSL:
#
#   tools/build_release.sh                 # FW_VERSION from oepl_CC1310/main.h
#   FW_VERSION=8 tools/build_release.sh    # override, e.g. for OTA tests
#
# Writes oepl_CC1310/build/release/chroma74h_{full,ota}_<ver>.bin.
# Builds in place, except in WSL with the repo under /mnt (the Windows
# filesystem, slow from WSL): there it builds in the same ~/build copy as
# tools/wsl_build.sh and copies the build directories back.
# SDK and CGT default to ~/ti as in the makefiles; set them in the
# environment to override.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$REPO"

if [ -n "$WSL_DISTRO_NAME" ] || grep -qsi microsoft /proc/version; then
   case "$REPO" in
   /mnt/*)
      SRC="$HOME/build/Tag_FW_CC13xx"
      echo "WSL build detected: $REPO is on the Windows filesystem."
      echo "Syncing it to $SRC (rsync, as tools/wsl_build.sh does) and building there."
      mkdir -p "$SRC"
      rsync -a --delete --exclude .git --exclude dumps --exclude 'oepl_CC1310/build' \
            --exclude 'bootloader_CC1310/build' "$REPO/" "$SRC/"
      ;;
   *)
      echo "WSL build detected: $REPO is on the Linux filesystem, building in place."
      ;;
   esac
fi

OUT="$SRC/oepl_CC1310/build/release"

if [ -n "$FW_VERSION" ]; then
   MAKE_ARGS=("EXTRA_DEFINES=-DFW_VERSION=$FW_VERSION")
else
   FW_VERSION=$(sed -n 's/^#define FW_VERSION[[:space:]]*\(0x[0-9A-Fa-f]*\).*/\1/p' \
                "$SRC/oepl_CC1310/main.h")
   [ -n "$FW_VERSION" ] || { echo "FW_VERSION not found in oepl_CC1310/main.h"; exit 1; }
   MAKE_ARGS=()
fi

make -C "$SRC/bootloader_CC1310" clean all
make -C "$SRC/oepl_CC1310" CONFIG=Chroma74H "${MAKE_ARGS[@]}" clean all

mkdir -p "$OUT"
bash "$SRC/tools/out2bin.sh" "$SRC/bootloader_CC1310/build/boot.out" \
     "$OUT/boot.bin" 0x0 0x10000
bash "$SRC/tools/out2bin.sh" "$SRC/oepl_CC1310/build/Chroma74H/oepl_CC1310.out" \
     "$OUT/app.bin" 0x1000 0xE000
python3 "$SRC/tools/make_74h_images.py" "$OUT/boot.bin" "$OUT/app.bin" \
        "$FW_VERSION" --out "$OUT"

if [ "$SRC" != "$REPO" ]; then
   echo "Copying bootloader_CC1310/build and oepl_CC1310/build/{Chroma74H,release} back to $REPO"
   mkdir -p "$REPO/bootloader_CC1310/build" "$REPO/oepl_CC1310/build"
   rsync -a "$SRC/bootloader_CC1310/build/" "$REPO/bootloader_CC1310/build/"
   rsync -a "$SRC/oepl_CC1310/build/Chroma74H" "$SRC/oepl_CC1310/build/release" \
         "$REPO/oepl_CC1310/build/"
   echo "Release images are in $REPO/oepl_CC1310/build/release"
fi
