# Tag_FW_CC13xx — notes for AI assistants

OEPL firmware for TI CC13xx based Chroma tags. Current work: the **Chroma 74H**
(CC1310, SN prefix `MS`) in `oepl_CC1310/`. `PLAN.md` tracks what is in
flight; read it first. Work one verified step at a time.

Related checkouts next to this one, used as reference:

- `../Tag_FW_Chroma` — the CC1110 Chroma firmware. Factory NVRAM / waveform
  decoders and notes (`chroma_60_files/`, `chroma_21_files/`).
- `../chroma74h_reverse_engineering` — static analysis (Rizin + rz-ghidra) of
  the 74H stock firmware dump, run as its own Claude session. Its
  `docs/firmware.md` is the source for the pin map, peripheral use and (in
  progress) the stock radio protocol, EPD engine, OTA and power management.
  Check it before reverse engineering anything here.
- `../oepl-cc2630-firmware` — a finished bare-metal OEPL port to a CC2630
  (same CC26xx core family, 2.4 GHz IEEE radio, UC8159 panel). Its
  `DEVELOPMENT.md` and `ARCHITECTURE.md` are the model for how a port gets to
  "works on batteries": AP-driven verification, crash reporting, OTA safety,
  sleep current measured pin by pin. It is GPL-3.0 and by another author —
  carry over facts and techniques, don't copy code into this repo.

## Repo layout

| Path | What |
|---|---|
| `ChromaAeon74/` | CC1311 port (Aeon 74), the most complete one. TI-RTOS-free (NoRTOS) |
| `oepl_CC1310/` | CC1310 port. Today only a ping test and the `CHROMA_PROXY` build |
| `common/` | Shared by both: radio, syncedproto, eeprom (SPI flash), powermgt, proxy |
| `proxy_CC1311/`, `rfEchoTx*`, `uartecho_CC1310/` | Bring-up / test projects |
| `shared/`, `uzlib/`, `bb_epaper/` | Submodules |
| `tools/` | Host tools (factory-data decoder, XDS110 target config) |
| `dumps/` | Local vendor dumps — **gitignored, never commit** |

## Build

CCStudio 20 projects (see README): SIMPLELINK_CC13X0_SDK 4.20.02.07 and
ARM-CGT 18.12.8 for CC1310, the F2 SDK 8.30.01.01 for CC1311.

**Command-line build (WSL, TI ARM-CGT 18.12.8)** — `oepl_CC1310/makefile`
mirrors the CCS project's options; run it through the wrapper:

```
MSYS_NO_PATHCONV=1 wsl -d Ubuntu -- bash /mnt/c/<path to repo>/tools/wsl_build.sh Chroma74H   # or Debug, Proxy, EpdTest
```

**Command-line build (macOS)** — the same makefiles run in place
(`make -C oepl_CC1310 CONFIG=Chroma74H`, `make -C bootloader_CC1310`) with
ARM-CGT 18.12.8 and the CC13X0 SDK in `~/ti` and Xcode's command line tools
(make 3.81, rsync, python3). Both TI installers are x86 and their
`installbuilder.sh` launcher rejects Apple Silicon ("The current OS X version
is not supported"); with Rosetta installed, run the bundled binary directly:

```
arch -x86_64 <installer>.app/Contents/MacOS/osx-x86_64 --mode unattended --prefix ~/ti
```

`tools/build_release.sh` (macOS or Linux/WSL) does the whole 74H release
below in one step: clean builds of both, `out2bin.sh`, `make_74h_images.py`,
output in `oepl_CC1310/build/release/` (`FW_VERSION=<n>` in the environment
overrides `main.h`). In WSL with the repo under `/mnt` it builds in the
`~/build` copy and copies the build directories back. DSLite comes with CCS:
`/Applications/ti/ccs<ver>/ccs/ccs_base/DebugServer/bin/DSLite` takes the
same arguments as `dslite.bat` below.

Machine-specific notes (this checkout's path, the webcam on the panel) are in
`CLAUDE.local.md`, which is gitignored.

**74H images: bootloader + app.** Internal flash is `bootloader_CC1310`
(sector 0, plus the CCFG at 0xFFA8, byte-identical to stock) and the app
(`CONFIG=Chroma74H`, linked at 0x1000 with `--define=OEPL_APP`, no CCFG).
OTA only ever rewrites 0x1000–0xEFFF; layout and the app/bootloader contract
are in `oepl_CC1310/ota_74h.h` (keep it stable). To make release images:

```
MSYS_NO_PATHCONV=1 wsl -d Ubuntu -- bash .../tools/wsl_build_boot.sh          # bootloader_CC1310/build/boot.out
MSYS_NO_PATHCONV=1 wsl -d Ubuntu -- bash .../tools/wsl_build.sh Chroma74H
# in WSL: tools/out2bin.sh boot.out boot.bin 0x0 0x10000 ; tools/out2bin.sh oepl_CC1310.out app.bin 0x1000 0xE000
python tools/make_74h_images.py boot.bin app.bin <FW_VERSION> --out <dir>
```

`chroma74h_full_<ver>.bin` is the 64 KB JTAG image (`dslite --mode flash -e
-f -v <file>,0x0`; dslite leaves the core halted after a .bin, so power
cycle). `chroma74h_ota_<ver>.bin` goes to the AP: upload it, then set the
tag's content to Firmware (mode 5) with that filename (or `/save_cfg` with
`contentmode=5` and `modecfgjson={"filename":"/<file>"}`). Automatic OTA
through `tagotaversions.json` is keyed by hwType, and 0x81 is shared with the
Aeon 74, so it needs an AP change before the 74H can be released that way.
The app refuses a file with its own FW_VERSION, a wrong hwType or CRC, or a
battery below 2400 mV (resting reading, `batteryRestMv`); a FW 7 tag needs
`make_74h_images.py --hwtype 0x87`, since it still checks for 0x87; the bootloader re-checks the CRC before erasing and
repeats an interrupted copy on the next boot. The bootloader itself is only
ever written over JTAG.

`Chroma74H` is the 74H OEPL tag firmware (`oepl_74h.c`, `common/` radio,
syncedproto and powermgt; no EasyLink). `EpdTest` draws the stock screen from
SPI flash and idles. Debug/Proxy are the original ping test and proxy builds.
One-off test switches for `Chroma74H` (pass as the wrapper's make argument
`"EXTRA_DEFINES=-D..."`): `WDT_TEST` (hang once, expect a watchdog reset),
`SCREEN_TEST` (draw each built-in screen), `FILL_TEST=0|1|2` (whole panel
black/yellow/white, then hold; for judging drive uniformity),
`REPORT_REFRESH_MV` (the check-in after a draw reports the lowest supply
voltage during the refresh as the battery voltage; every build keeps it, with
the max and sample count, in `gEpdDiag` for a JTAG read), `EPD_BTST=0xAABBCC`
(booster soft start bytes instead of the script's `57 2B 38`; `57 2B 17`
drove the panel too weakly and left the gradient). `EPD_SPI=` as a
make argument builds the bit-banged EPD SPI instead of SSI1.

Output: `oepl_CC1310/build/<CONFIG>/oepl_CC1310.out` + `.map` (gitignored).
The wrapper builds from an rsync'd copy in `~/build/Tag_FW_CC13xx` because
WSL2 reads `/mnt/c` slowly; `tools/build_release.sh` uses the same copy when
run from `/mnt`. The copy also used to be needed for case-sensitive includes:
`common/oepl_board.h` was `common/board.h`, and on a case-insensitive
filesystem (`/mnt/c`, macOS) `#include "Board.h"` from `common/` picked it up
instead of `oepl_CC1310/Board.h`. Since the rename the tree builds in place
on any filesystem; don't add another header that differs from an existing one
only by case. `CONFIG=Release` doesn't
build (no board define, stale in the CCS project too). As of 2026-11-15, with
no optimisation (as in CCS Debug): Debug uses 52.4 KB flash / 6.3 KB RAM,
Proxy 59.4 KB / 7.0 KB, `.ccfg` at `0xFFA8` in both.

WSL already has (`~/ti`): **CC13X0 SDK 4.20.02.07** (installed 2026-11-15; it
ships both TI-compiler `.aem3` and GCC `.am3g` libraries, plus `ccs/` and
`gcc/` makefiles for `rfEasyLinkEchoTx`, which `oepl_CC1310` was created
from), `arm-none-eabi-gcc` 13.2, and the CC13xx/CC26xx SDK 8.32 (CC13x1 and
newer only — usable for the Aeon, **not** for the CC1310).

## Hardware access on this machine

**SPI flash (CH341A clip).** The CH341A is shared with usbipd-win (busid
`1-3`) and used from WSL Ubuntu, where flashrom is installed:

```
usbipd attach --wsl --busid 1-3        # PowerShell; usbipd detach --busid 1-3 to give it back
MSYS_NO_PATHCONV=1 wsl -d Ubuntu -- flashrom -p ch341a_spi -r /mnt/c/.../dump.bin
```

From Git Bash, set `MSYS_NO_PATHCONV=1` or `/mnt/c/...` paths get rewritten,
and put loops in a script file: `wsl` re-expands `$vars` on the command line.
Read at least twice and compare hashes before trusting a dump. The 74H chip
reads as an "SFDP-capable chip", JEDEC `C2 2014` (Macronix MX25V80xx, 1 MB).

**CC1310 internal flash (XDS110, cJTAG 2-pin).** UniFlash 9.5 is at
`C:\ti\uniflash_9.5.0`. Use the `tools/cc1310f*_xds110.ccxml` matching the
chip marking (OSCAN1, 500 kHz). **The 74H is a CC1310F64**: 64 KB flash
(`0x10000`, CCFG in the last 88 bytes at `0xFFA8`), 16 KB RAM. The F128 has
128 KB / 20 KB.

```
dslite.bat --mode memory --config=tools\cc1310f64_xds110.ccxml --range=0,0x10000 --output=dumps\chroma74h\cc1310_flash.bin
dslite.bat --mode flash  --config=tools\cc1310f64_xds110.ccxml -e -f -v image.bin,0x0
```

`oepl_CC1310/Chroma_CC1310.cmd` (originally an unmodified copy of TI's F128
LaunchPad file) now defaults to the F64: `FLASH_SIZE 0x10000`, `RAM_SIZE
0x4000`, so `.ccfg` lands at `0xFFA8`. An F128 tag (the Solum 2.9") needs
`--define=CC1310F128` in the **linker** options (the linker command file only
sees linker defines). Don't confuse it with the compiler define `CC1310_128K`,
which Debug/Proxy set: in `common/oepl_board.h` that selects the 128 KB **SPI flash**
(MX25V1006) board config, not the chip. Before it was fixed, `.ccfg`
went to 0x1FFA8 — nonexistent on an F64 — so earlier F64 builds can only have
booted on the stock CCFG left in the last sector.

A `dslite --mode memory` read of a running tag returns live RAM (useful for
diagnostics, e.g. `gEpdDiag` in `epd_74h.c`; find the address in the
`.map`), but **the firmware restarts afterwards** — seen as the EPD test
redrawing after each read.

**The 74H panel draws worse at higher supply voltage (2026-11-16).** At 3.0 V
and 3.3 V (XDS110 output or bench supply) an all-black draw comes out black in
the top third and increasingly yellow/brown and speckled below, with a ghost
of the previous image. Only drawn (black) pixels shade toward yellow; white
areas stay white. The top third flashes darker during the refresh, and
yellow bars look brown. Only at ~2.3 V (bench supply, or 2 of the 6 CR2450 coin cells (in parallel)
sagging under load) is it even. **Stock firmware does the same**: flashed
back, it drew the same all-black slot at 3.3 V with the same gradient. Our
command stream matches stock exactly (RE decode), and a bit-banged SPI build
looked the same as SSI1, so this is the panel/board, not the firmware. Stock's
own last screen (black circle + checkmark on a full pack) looked clean; why is
open. **On its own coin cells the tag draws evenly** (2026-11-17, FW 7, 4 of
6 CR2450s, closed tag): the overnight weather draw came out evenly black,
with the supply sagging to 2292–2468 mV during refreshes (`REPORT_REFRESH_MV`).
The cells' internal resistance pulls the supply into the good range, so the
gradient is mostly a bench-supply artefact. With all six cells (same day,
resting 2828 mV vs 2738–2781 with four) it came partly back: the dark band
during the refresh reached halfway down and the weather screen showed a
gradient, weaker than on the bench. Judge drawing quality at ~2.3–2.7 V or on the tag's batteries; the
debugger's 3.3 V is fine for everything else.

Errors -242 / -2131 / -275 are a marginal TMS/TCK connection or the wrong scan
format: re-check the two JTAG wires, then drop TCLK to 100 kHz. On the 74H+
the back-side pads are unlabelled, and **the OEPL wiki's TMS/TCK assignment
is swapped**: wired as the wiki shows, the connect fails with -2131; with TMS
and TCK exchanged it works (user-verified 2026-11-15). RESET as on the wiki. The
`dbgjtag -S integrity` scan test reports `SC_ERR_PATH_MEASURE` (-230) on this
tag even when wired correctly, so test with a small `dslite --mode memory`
read instead. dslite reads past the end of an F64 (it returned 0x1FFC0 bytes
for `--range=0,0x10000`); everything above 0x10000 is `BA D0` fill, so keep
only the first 64 KB. After flashing,
**power cycle** the tag; the XDS110 reset pulse doesn't start the new image.


**Take the vendor dumps before flashing anything.** They are the only way back
to stock. If the debug port turns out to be locked, getting in means a mass
erase that destroys the stock firmware — stop and ask the user first.

## Chroma 74H facts (from the SPI dump, 2026-10-30)

`tools/decode_factory.py dumps/chroma74h/chroma74h_spi_flash.bin [--png out.png]`
prints all of this.

| SPI address | Contents |
|---|---|
| `0x0000`, `0x1000` | Factory NVRAM, magic `56 12 09 85`. Stock firmware alternates between the two sectors |
| `0x2000` | Panel command script, magic `0xBEEFFACE`, format `04 03`, name `D21Y10041BSMY09Z_V4 686 74OTP` |
| `0x8000` | Flash self-test pattern |
| `0x15000`, `0x2D000` | Stock screen slots: 0x20-byte header (u32 size = 96000), then two 48000-byte planes |

- NVRAM records: `<type> <len incl. 2-byte header> <data>`, type 0 =
  invalidated, `0xFF` = end, later records win. On CC1310 the types differ from
  CC1110: `0x80` tag SN (7 bytes: 2 chars, 4 BCD bytes, revision char — same
  layout as the Aeon `InitSN()` expects), `0x81` panel serial, `0x88`
  resolution (800 × 480). VCOM and temperature calibration records are not
  identified yet.
- Panel: **800×480 BWY** (black/white/yellow; user confirmed, and the `-Y-` in
  the panel serial matches the Chroma21's yellow `Y005`). The second plane in
  the screen slots is the yellow plane. The waveform is in the controller's OTP
  (`PSR 0x0F`, the "74OTP" in the name), so it ships with the panel and the
  firmware needs no colour-specific LUTs; a BWR 74H, if one exists, would
  differ only in the AP tagtype's colour table. The script has three
  segments: init `0x2047–0x20A0`, image `0x20A1–0x20A6`, refresh + off
  `0x20A7–0x20BE`. Init: panel power on (`09`), reset + hidden registers
  (`06`), `03 00`, eight `F8 <reg> <val>` hidden-register writes,
  `PSR 0F`, `BTST 57 2B 38`, `TSE 00`, `CDI 77`, `E8 36`, `PON`. Then: wait
  busy, image record (`07`), `11 00`, wait busy, `POF`, wait busy, 5 s delay,
  panel power off (`09`), end (`00`). Record meanings come from the stock
  interpreter (see "EPD interface" below).
- bb_epaper's `EP75R_800x480` (UC8179) left this panel blank: it uses partial
  window opcodes `0x91`/`0x92` that this controller doesn't accept. "686" in the
  script name probably names the actual controller.
- **Controller: probably the Jadard JD79686AB** (inference: "686" plus every
  script opcode matches its command set; datasheet v1.0.5 at
  buydisplay.com/download/ic/JD79686AB.pdf). There `0x11` is DSP (data stop;
  refresh starts once a full frame is in, which is why stock never sends DRF
  `0x12`), partial update is `0x14`/`0x15`/`0x16` (PDTM1/PDTM2/PDRF, not
  `0x91`/`0x92`), `E8` is PWS (power saving), `E5` TSSET (force temperature),
  `E9` AUTO (PON-DRF-POF). PSR bit 4 = B/W only (LUT1 only), bit 5 = LUTs from
  registers `0x20`–`0x24` instead of OTP; the script's `0x0F` is OTP LUTs,
  B/W/colour. CDI `0x77` = DDX `11`: colour bit 0 = yellow, else B/W bit 1 =
  white, as measured. Panel label: `YS4750GS091` / `SMY09Z1` / `J0AC6D02WF`
  (no web hits).
- The SPI flash holds no firmware copy; the stock firmware only exists in the
  CC1310's internal flash.
- **Board pin map** (from the stock `BoardGpioInitTable` at `0xF164`, see
  `../chroma74h_reverse_engineering/docs/firmware.md` §3). It matches
  `oepl_CC1310/Chroma_CC1310.h` exactly, so that header's "Chroma 21" pinout
  is the 74H pinout too:

  | DIO | Net | Stock reset state |
  |---|---|---|
  | 0 | EPD_PWR | output high |
  | 1 | EPD_BUSY | input, no pull |
  | 2, 3, 4 | EPD CS, SCLK, SDI (stock drives the panel with hardware **SSI1**) | output low |
  | 5 | EPD_RST | output low |
  | 6, 7 | UART0 RX (pull-down) / TX (high) — jig/debug port | |
  | 8, 9, 10, 11 | SPI flash MOSI, CLK, MISO (pull-up), CS (high) on **SSI0** | |

- **EPD interface** (stock driver at `0xB130–0xBB00`, write-up in
  `../chroma74h_reverse_engineering/docs/epd.md`; items marked ✓ re-checked
  against the dump here):
  - 3-wire SPI on SSI1, **9-bit frames, D/C = bit 8**: commands sent raw, data
    as `byte | 0x100` ✓ (`orr.w #0x100` at `0xB6D4`). Same convention as the
    Aeon's `write9bits()`. CS (DIO2) driven by hand. Clock rate and SPI mode
    not yet found (mode 0 assumed).
  - **EPD_PWR (DIO0) is active-low: HIGH = off** ✓ (power-down sets DIO0 via
    DOUTSET). RST (DIO5) active-low. **BUSY (DIO1) is LOW while busy** —
    measured on hardware 2026-11-16; the RE notes' "active-high" is wrong.
    BUSY asserts within 1 ms of the command; waits after PON ~60 ms, after
    POF ~30 ms, and the **full refresh ~52 s** (room temperature).
  - **First light 2026-11-16:** `oepl_CC1310/epd_74h.c` (`make` config
    `EpdTest`) drew the stock checkmark screen from SPI flash `0x15000` by
    running the factory script. Bit-banged 9-bit SPI works.
  - **Plane polarity** (same test: a yellow plane of all `0xFF` showed no
    yellow; the B/W plane's `0xFF` background showed white): B/W plane
    (`0x10`) `1` = white, `0` = black; yellow plane (`0x13`) `1` = no
    yellow, `0` = yellow.
  - Power-up: RST low, PWR on, 25 ms, CS high, reset pulse (2 ms low, 5 ms
    settle), then the eight `F8` hidden-register writes (table at `0xBB30` ✓,
    same values as the script) and BTST `57 2B 38` (`0xBB40` ✓).
  - Power-down: PWR off, RST and CS driven **low**, not left floating.
  - Planes go out with `0x10` (B/W) and `0x13` (yellow), streamed from the
    SPI flash slots in small chunks, bytes sent exactly as stored.
  - Stock runs the 0x2000 script with an interpreter (`0xB330`); record types
    are decoded by `tools/decode_factory.py`.
  - **Refresh is `11 00`, then wait for busy** — stock never sends `0x12`
    (no `movs #0x12` or `#0x11` anywhere in the EPD code ✓; `0x11` only comes
    from the script). Draw tail: planes → `11 00` → wait busy → `POF 02` →
    wait busy → 5 s → panel power off. That `0x11` is the refresh opcode on
    this controller is an inference from the sequence; confirm on hardware.
  - Image record (`07`): `TRES 61 03 20 01 E0` (✓ `0x61` at `0xB89E`), then
    `VDCS 82 <vcom>` if a VCOM value is stored (✓ `0xB8C2`), then planes by
    record **byte 2**: `0` = both planes, `0x10` (B/W) then `0x13` (yellow);
    `1` = TRES/VCOM only; `≥2` = one plane, `0x10` if byte 1 is 2 or 5 else
    `0x13` (✓ selector at `0xB31E`). The factory script's `07 00 00` therefore
    sends both planes.
  - VCOM comes from stock's settings lookup, key `0x17`. If absent, `VDCS` is
    skipped. Guess (unconfirmed): NVRAM type = key + `0x80`, and this tag has
    no `0x97` record, so stock sends no VCOM to this OTP panel.
  - A full draw runs three script segments chosen by temperature from a table
    of 9-byte entries at SPI `0x202A` (layout being decoded).
  - Other readbacks: `0x81` (status, 1 byte, script record `08`, ✓ `0xB8E4`)
    and `0xA2` (OTP/VCOM, N bytes). Reads need the SDI line turned around.
- **SPI flash power** (✓ `0xB9` at `0xE49C`, `0xAB` at `0xE5F6`): stock wakes
  the flash with `0xAB` before every access and puts it back in deep
  power-down with `0xB9` after, with SSI0 power-gated in between.
- **Standby pin states** (RE project `firmware.md` §8.3.1): stock never
  reconfigures pins for sleep; each keeps its boot config. EPD_PWR high (off),
  EPD CS/CLK/SDI/RST low, BUSY input; UART RX pull-down, TX high; flash
  MOSI/CLK low, MISO pull-up, CS high (flash in deep power-down); **every
  unused DIO is an input with pull-down**. Nothing floats — the target for
  our sleep current.
- **CCFG:** stock has the internal DC/DC converter **off** (`MODE_CONF`
  bits 27:26 set); `oepl_CC1310/ccfg.c` overrides TI's default to match, so
  our `.ccfg` is byte-identical to stock.

## Chroma 74H stock firmware (internal flash, 2026-11-15)

`dumps/chroma74h/cc1310f64_stock_flash.bin` (64 KB, two identical reads) and
`cc1310f64_stock_ccfg.bin` (its last 88 bytes). All 16 sectors are used. No
text strings. Initial SP `0x20004000`, reset handler `0xF535`, exception
handlers in ROM at `0x1001C9FD` (TI-RTOS kernel in ROM).

Stock CCFG (at `0xFFA8`):

| Register | Value | Meaning |
|---|---|---|
| `BL_CONFIG` | `0x00FFFFFF` | `BOOTLOADER_ENABLE = 0x00`: **ROM serial bootloader disabled**, so a stock tag can't be read or flashed over UART |
| `CCFG_TAP_DAP_0` | `0xFFC500C5` | CPU DAP enabled (`0xC5`), so JTAG works; PRCM TAP off |
| `CCFG_TAP_DAP_1` | `0xFF000000` | PBIST / WUC TAPs off |
| `CCFG_TI_OPTIONS` | `0xFFFFFF00` | TI failure analysis off |
| `IMAGE_VALID_CONF` | `0x00000000` | Boot the image at 0x0 |
| `MODE_CONF` / `MODE_CONF_1` | `0xFFBFFF3A` / `0xFF880010` | Oscillator/DC-DC settings; compare with ours before flashing |
| `EXT_LF_CLK`, `SIZE_AND_DIS_FLAGS` | `0x01800000`, `0x0058FFFD` | |

Our own CCFG may set `BOOTLOADER_ENABLE = 0xC5` with a reachable backdoor pin
to allow serial re-flashing later; enabling it is safe, disabling it is what
bricked a CC2630.

## Radio: sub-GHz, compatible with the CC1110 Chroma tags

The CC1310 radio is sub-1 GHz only, so OEPL's 2.4 GHz 802.15.4 mode (as on
the CC2630 TG-GR6000N, `CMD_IEEE_RX`, channels 11–26) is not an option on
these tags.

OEPL here is **sub-GHz proprietary, not 802.15.4 PHY**, matching the CC1110
Chroma firmware (`../Tag_FW_Chroma/Chroma_Tag_FW/soc/cc111x/radio.c`) setting
for setting: 250 kbps 2-GFSK, 165 kHz deviation, sync `0xD391D391`,
CC1101/CC2500-compatible whitening, radio CRC16, variable length; channels
100–105 (864–869 MHz) and 200–205 (903–923 MHz) with identical synth values.
Only the frame *layout* is 802.15.4 MAC style (`MacFrameBcast` etc. from
`shared/oepl-proto.h`), with PAN ID `PROTO_PAN_ID_SUBGHZ` (`0x1337`), as on the
CC1110 tags. The CC1311 Aeon 74 uses this path (`common/radio.c`, sysconfig
`custom868`) and has downloaded images from a sub-GHz AP.

The CC1310 build does **not** use it yet: `oepl_CC1310/.cproject` excludes
`common/radio.c`, `syncedproto.c`, `RFQueue.c` and `powermgt.c`. Its main is an
EasyLink ping test on channel 203 with the same PHY
(`oepl_CC1310/smartrf_settings/`). `common/radio.c` expects sysconfig's
`ti_radio_config.h` (`RF_*_custom868_0` names), which the CC13x0 SDK project
doesn't generate, so porting it means mapping onto `smartrf_settings.c`. The
SmartRF settings ask for 14 dBm TX, which needs `CCFG_FORCE_VDDR_HH = 1` in the
CCFG.

## 74H OEPL status (2026-11-16)

- Checks in with the sub-GHz AP and displays G5 and zlib images end to end.
  MAC `44670C1210085924` (`InitSN()`: `44 67`, SN letters − 'A', 4 BCD
  bytes).
- **hwType 0x81** (`CHROMA_AEON_74`, shared with the Aeon 74) from FW 8, at
  the maintainer's suggestion (PR review). Upstream `81.json` (800×480, 2 bpp,
  `zlib_compression` "3", templates from `36.json`; `usetemplate` is a
  decimal hwType, 54 = 0x36) fits this panel; its colour table says red,
  which this panel shows as yellow. Upload it to an AP that lacks it. The
  AP's single `tagotaversions.json` entry per hwType can't serve both
  boards' firmware; 74H OTA files carry their own magic (`OA74`), so the
  tags never accept each other's files. FW 3–7 used a provisional **0x87**
  with `87.json` (`g5_compression` "3", branch `feat/chroma74h-tagtype` in
  `../OpenEPaperLink-chroma74-bwy`), now obsolete. Not 0x86: that is
  `CHROMA_74_BWY`, the 640×384 CC1110 Chroma74 BWY.
- **zlib** (`ZLIB_SUPPORT`, `common/zlib_decode.c`, uzlib): the file is a u32
  size, then a zlib stream of the 6-byte image header and planes. The AP's
  miniz uses a 4 KB dictionary and `rewriteHeader()` declares 4 KB (`0x48`),
  so a 4 KB window suffices; it borrows `blockXferBuffer` (idle after the
  download), decoding slot to slot like G5. The AP prefers zlib over G5 when
  the tagtype allows both. Tested on the PC with the real weather image
  (AP-style sync flush between planes; bad window, corrupt and truncated
  files refused) and with the AP's own zlib file (6011 bytes for the weather
  screen vs ~5.5 KB G5), then on the tag: FW 8 drew it and acked
  (2026-11-17 13:43). After the hwType change the AP rendered once more with
  the old tagtype; re-save the content to get zlib. `.bss` is split (`>>` in `Chroma_CC1310.cmd`) around the
  fixed `.TI.noinit`, and the stack is 1.5 KB for uzlib's tree decoding.
- Upload to the AP with `curl -F path=/tagtypes/81.json -F file=@...
  .../littlefs_put`, and **set `MSYS_NO_PATHCONV=1` in Git Bash**: it
  rewrites `path=/tagtypes/...` into `C:/Program Files/Git/tagtypes/...`
  and the AP answers "Error. Disk full?" (not an AP bug).
- AP content is held until the tag's expected check-in (from the sleep the AP
  offered) is < 5 min away. For bench tests set the AP's "Shorten latency
  during config" to yes and keep its web page open.
- G5 decoder: `g5/` (from Tag_FW_Chroma, BSL 1.1 like `bb_epaper`), decoded
  slot to slot in SPI flash, enabled by `G5_SUPPORT` in `common/oepl_board.h`. The
  AP's `g5Compress()` takes a 16-bit size, so a frame compressing above ~30 KB
  falls back to raw (96000 bytes, also handled).
- To test the decoder on the PC: fetch the AP's file
  (`/edit?download=/current/<MAC>_*.pending`) and run `g5/g5.c` with
  `eepromRead`/`eepromWrite` mocked on a RAM array (gcc in WSL, ASan).
- **Standby wipes peripherals set up by hand.** Any `WaitMs()`/`doSleep()`
  may enter standby, which powers the peripheral domain down. TI drivers
  re-init through Power notifications; direct driverlib use (SSI1 for the
  EPD) does not. `epd_74h.c` closes SSI1 before every sleep (`EpdSleep()`)
  and reopens it at CS low (`EpdCsLow()`). Left open over the refresh wait,
  the next command sat in a dead FIFO until the watchdog fired.
- **Debugging a stuck tag:** RAM survives the reset, so read it with one
  `dslite --mode memory --range=0x20000000,0x1000`. **dslite's length is in
  32-bit words** (0x1000 = all 16 KB; a byte count runs past RAM and fails).
  `#pragma NOINIT` records: `gWdtResetMagic` ("WDTR", watchdog fired),
  `gFault` (HardFault registers and stack), `gEpdTrace` (last 16 EPD events,
  `(ms << 8) | event`); `gEpdDiag` holds busy-wait timings and plane upload
  times. The `NOINIT` records are linked at `0x20001800` (`.TI.noinit` in
  `Chroma_CC1310.cmd`): the bootloader's startup zeroes `0x20000000`–
  `0x20000FFF` on every boot, which wiped them in FW 4–6 (watchdog resets
  were reported as first boots). After a JTAG session the debugger's vector catch halts the core on a
  HardFault and any reset halts in the boot ROM, so a stuck tag stays stuck
  until power cycled; without a probe the fault handler or watchdog resets it.
- **Panel script bands (decoded 2026-11-16):** body at script +0x2A starts
  with u32 offsets `seg1, band table, seg3` (script-relative to 0x202A).
  Each 9-byte band entry is `int8 threshold °C, u32 seg2 offset, 4 unused`,
  descending, ended by threshold < -99; stock runs seg1, the first band with
  threshold <= temperature, then seg3. This tag: seg1 and its only band's
  seg2 (>= -99 °C) are empty END records, seg3 = 0x2047..END, so the draw is
  temperature-independent and matches what `epd_74h.c` runs. Other panels'
  scripts may have real bands.
- **SPI flash timing:** a page program or erase runs on after the SPI
  command. Every read and every WREN must wait for the busy bit first
  (`eepromPrvBusyWait()`); without it, reads returned garbage and a sector
  erase right after a write was silently ignored.

## Factory NVRAM: don't destroy it

Learned on the CC1110 Chroma tags (`../Tag_FW_Chroma/chroma_60_files/pr_nvram_fixes.md`):

- Never erase/rewrite the factory NVRAM because one read failed. A lost SN
  turns the MAC into `…00000000`, and on Chroma29 an all-zero SN made the
  firmware back-power the switched-off EPD and killed panel rows.
- The live copy can be in either sector; any reset must handle both.
- Guard against zero-length records when scanning (one hung the CC1110 parser).

## CC13xx/CC26xx traps (from the CC2630 port)

- **CCFG:** setting `BOOTLOADER_ENABLE` (BL_CONFIG byte 51) to anything but
  `0xC5` disables the ROM bootloader; on a CC2630 with no JTAG access that was
  a brick. Never disable the CPU DAP (`CCFG_TAP_DAP_0`) either — JTAG is the
  only way into a tag whose bootloader is off, as the 74H's stock one is.
- **A JTAG connection keeps the debug power domain on until power is cut**, so
  standby current measured after attaching a debugger is meaningless. Power
  cycle with the probe detached before measuring.
- **Sleep current is mostly pin configuration.** Floating panel control lines
  cost tens of µA each (crowbar current in the panel's input stages); pulling
  them up during sleep took the CC2630 tag from 60 µA to 9 µA. Put the SPI
  flash in deep power-down with CS held high. Relevant to the README's 4 mA
  sleep to-do.
- A short power gap doesn't reset the tag; the board caps carry standby for
  seconds. Leave it off ≥ 15 s for a real cold boot.
- Bound every hardware wait loop. An infinite wait before logging is up looks
  exactly like a dead chip.

## OEPL / AP behaviour (applies to any tag)

- **The AP is the test oracle.** `http://<AP>/get_db?pos=N` (paginated) gives
  per-tag `ver`, `hash` (last image confirmed by XferComplete, all-zero =
  never) and `pending`. A check-in alone proves nothing about the receive path;
  always run an image push end to end.
- `AvailDataInfo.nextCheckIn` is minutes, or seconds in the low 15 bits when
  bit 15 is set.
- The AP picks the image format from the tag's **reported firmware version**,
  not its capabilities: a tagtype with `zlib_compression` serves
  `DATATYPE_IMG_ZLIB` once the reported version reaches that value. Keep the
  reported version below it until zlib decoding works.
- Report the real channel number in `currentChannel`; the AP drops check-ins
  from unknown MACs whose channel disagrees with its own.
- Send `XferComplete` after the refresh has finished and the panel is powered
  off, and retry it until acked. Sent during the refresh current spike it was
  lost about half the time.
- Retry a partly-received block with `BLOCK_PARTIAL_REQUEST` (served from the
  AP's buffer) rather than `BLOCK_REQUEST` (re-fetched from the ESP32).
- Keep AP `maxsleep` below 20 minutes: the AP drops pending data after 20
  housekeeping minutes. Queueing an OTA makes the AP hand out a huge
  `nextCheckIn`, so a tag that misses the offer sleeps its cap (an hour) and
  looks dead.
- The AP only renders content when the tag's expected check-in is within ~5
  minutes, and reboots now and then (a DB record jumping back is that).

## Conventions

- Vendor binaries (dumps, stock firmware) stay local in `dumps/`. Commit facts
  learned from them (tables, init sequences, decoders), not the blobs.
- Record what was measured and how; mark inferences as inferences.
- Match the surrounding code style (3-space indent, `LOG`/`ELOG`/`VLOG`
  logging from `common/logging.h`).
- Don't commit unless asked; the user's fork is `knyghtryda/Tag_FW_CC13xx`.
