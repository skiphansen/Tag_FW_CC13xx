#!/bin/bash
# Build bootloader_CC1310 in WSL from the same rsync'd copy as wsl_build.sh.
#   wsl -d Ubuntu -- bash tools/wsl_build_boot.sh [make args...]
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$HOME/build/Tag_FW_CC13xx"
mkdir -p "$WORK"
rsync -a --delete --exclude .git --exclude dumps --exclude 'oepl_CC1310/build' \
      --exclude 'bootloader_CC1310/build' "$REPO/" "$WORK/"
make -C "$WORK/bootloader_CC1310" "$@"
mkdir -p "$REPO/bootloader_CC1310/build"
rsync -a "$WORK/bootloader_CC1310/build/" "$REPO/bootloader_CC1310/build/"
