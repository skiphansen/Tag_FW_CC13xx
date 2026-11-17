# Plan

Status legend: `[ ]` todo · `[~]` in progress · `[x]` done (verified)

## Chroma 74H (CC1310) port

Bench tag: SN `MS10085924C`, 800×480 BWY (black/white/yellow).

### 1. Extraction

- [x] SPI flash dumped with CH341A + flashrom, three identical reads (2026-10-30).
      `dumps/chroma74h/chroma74h_spi_flash.bin`
- [x] Factory data decoded: NVRAM (SN, panel serial, resolution), panel command
      script, stock screen slots. `tools/decode_factory.py`; facts in CLAUDE.md
- [x] Chip identified from its marking: **CC1310F64** (64 KB flash, 16 KB RAM)
- [x] CC1310 internal flash dumped over XDS110, two identical reads, debug port
      not locked (2026-11-15). `dumps/chroma74h/cc1310f64_stock_flash.bin`
- [x] CCFG saved and decoded: ROM bootloader disabled, CPU DAP enabled.
      `dumps/chroma74h/cc1310f64_stock_ccfg.bin`; table in CLAUDE.md

### 2. Board bring-up

- [x] `oepl_CC1310/Chroma_CC1310.cmd` sized for the F64 by default (flash
      0x10000, RAM 0x4000), F128 via linker `--define=CC1310F128`. Verified
      2026-11-15 with the WSL build: Debug and Proxy link, `.ccfg` at 0xFFA8
- [x] WSL command-line build: `oepl_CC1310/makefile` + `tools/wsl_build.sh`,
      TI ARM-CGT 18.12.8 and CC13X0 SDK 4.20.02.07 in `~/ti`
- [ ] **Code size:** the unoptimised ping test already uses 80% of the 64 KB
      flash (Proxy 91%). Try `--opt_level` / `--opt_for_speed=0` and see what
      the EasyLink + display libraries cost before the OEPL code goes in. The
      OEPL build won't need EasyLink (it uses the RF driver directly)
- [ ] Release configuration is stale (no board define); fix or drop it
- [x] CCFG matches stock byte for byte (2026-11-15). TI's default differed only
      in `MODE_CONF`: it enabled the DC/DC converter, which stock has off
      (no inductor on the board, presumably). `oepl_CC1310/ccfg.c` now
      overrides `DCDC_ACTIVE`/`DCDC_RECHARGE`. CPU DAP stays enabled
- [ ] RAM budget: an 800×480 two-plane frame is 96 KB against 16 KB of RAM, so
      the display path must stream from SPI flash (as the CC2630 port does)
- [x] Pin map recovered from the stock firmware by `../chroma74h_reverse_engineering`
      (2026-11-15): matches `oepl_CC1310/Chroma_CC1310.h` already, no change needed.
      Stock drives the EPD on hardware SSI1 and the SPI flash on SSI0
- [ ] Identify the remaining NVRAM records (VCOM? temperature calibration?)
- [ ] Identify the panel controller ("686" in the script name)
- [x] Stock EPD driver reverse engineered (RE project `docs/epd.md`, key points
      re-checked against the dump): 9-bit SPI with D/C = bit 8, EPD_PWR
      active-low, power/reset sequence, planes via `0x10`/`0x13` streamed from
      flash. Details in CLAUDE.md, "EPD interface"
- [x] Refresh trigger: `11 00` + wait busy, no `0x12` (RE project, spot-checked);
      script records `08` (status read) and `0a` (conditional power) decoded
- [x] B/W plane: the script's `07 00 00` (byte 2 = 0) uploads both planes,
      `0x10` then `0x13` (RE project)
- [ ] Open, sent to the RE project: VCOM key `0x17` → NVRAM record mapping (and
      the other boot-time records), the temperature/segment table at SPI
      `0x202A`, SSI1 clock rate and SPI mode, standby entry path and per-pin
      sleep states
- [x] EPD driver, first light (2026-11-16): `oepl_CC1310/epd_74h.c` runs the
      factory script from SPI flash with bit-banged 9-bit SPI and drew the stock
      checkmark screen. BUSY found to be active-low; refresh takes ~52 s.
      EPD control lines now idle low at boot (were high: back-powering risk)
- [x] Plane polarity (2026-11-16): B/W `1` = white; yellow `0` = yellow
- [x] EPD on SSI1 hardware SPI (2026-11-16, `EPD_HW_SPI`): driverlib SSI, 9-bit
      frames, mode 0, 4 MHz (stock clock/mode not located, mode 0 inferred);
      CS by hand. A 48000-byte plane takes ~185 ms. Run 4 (1 px lines,
      checkerboards) checked on the webcam. SSI1 must be closed only once:
      `EpdPower(false)` runs twice per draw, and the second close touched SSI1
      with its clock off and hung the tag
- [x] Temperature/segment table at 0x202A: not needed for this panel (RE
      project epd.md, 2026-11-16). The waveform is in the controller's OTP and
      the script enables its internal sensor (`TSE 41 00` at SPI 0x208E,
      checked here), so the controller compensates; stock runs 0x2047→END at
      every temperature, as we do. The table matters for external-LUT panels
      [inference, well supported]
- [x] Battery voltage and temperature reporting (AP shows 3273 mV, 19 °C)

### 3. OEPL

- [x] Radio target confirmed (2026-11-15): sub-GHz proprietary PHY identical to
      the CC1110 Chroma tags (250 kbps GFSK, sync D391D391, channels 100–105 /
      200–205), not 802.15.4. See CLAUDE.md, "Radio"
- [ ] Bring `common/radio.c`, `syncedproto.c`, `RFQueue.c`, `powermgt.c` into the
      CC1310 build (excluded today); map `ti_radio_config.h` names onto
      `oepl_CC1310/smartrf_settings/`
- [ ] 74H board config in `common/oepl_board.h`. Don't reuse the Aeon layout: it puts
      `EEPROM_SETTINGS_AREA_START` at SPI 0x0, which on the 74H is the factory
      NVRAM (SN!) — 0x0000–0x2FFF must never be written. Free on the 74H:
      0x3000–0x7FFF, 0x9000–0x14FFF, 0x45000+ (stock slots 0x15000–0x44FFF
      become reusable once OEPL owns the display). SN from NVRAM record 0x80,
      not internal flash 0x2000 as on the Aeon
- [x] zlib images (FW 8, 2026-11-17): 4 KB window borrowed from the block
      transfer buffer, decoded slot to slot (`common/zlib_decode.c`). With it
      the tag moved to hwType 0x81 (`81.json` sends zlib), as Skip suggested
      in the PR review
- [x] Stage 1, check-in (2026-11-16): makefile config `Chroma74H`
      (`oepl_CC1310/oepl_74h.c` main following the Aeon `main.c`, SN from
      NVRAM record 0x80; `ti_radio_config.h` / `ti_drivers_config.h` shims map
      `common/radio.c`'s SysConfig names onto `smartrf_settings/`; FW_VERSION 2
      = raw images; `drawImageAtAddress()` in `epd_74h.c`), 48.9 KB flash /
      9.7 KB RAM. **The tag appears in the AP DB** (AP 192.168.1.244): MAC
      `44670C1210085924`, hwType 0x86, ver 2, 3273 mV, 19 °C, RSSI −71.
      `common/syncedproto.c` didn't fill `currentChannel` in the check-in (AP
      showed `ch` 0); fixed, AP now shows `ch` 202
- [x] Stage 2, images (2026-11-16): G5 decoder from `../Tag_FW_Chroma/g5/`
      (0e17be9) in `g5/` with its BSL 1.1 licence files, rows widened to 800
      px, private 396-byte workspace. `common/syncedproto.c` (`G5_SUPPORT`
      from `oepl_board.h`, 74H only): G5 file decoded into a second slot that skips
      the displayed one; `imageSize` now 32-bit (raw 74H image = 96000 bytes);
      downloads larger than a slot rejected; a failed decode clears
      `curDataInfo` so it isn't acknowledged as shown. FW_VERSION 3.
      51.6 KB flash / 10.2 KB RAM.
      **HW_TYPE 0x87**, not 0x86: 0x86 is `CHROMA_74_BWY` (the 640×384
      CC1110 Chroma74 BWY, `../OpenEPaperLink-chroma74-bwy` 151f772d and the
      shared-definitions branch `chroma74-bwy-hwid`); 0x81 `CHROMA_AEON_74`
      ("also Chroma 74H+") is BWR. Tagtype `87.json` (800×480 BWY,
      `usetemplate` 54, `g5_compression` "3") written in the AP fork and
      uploaded to the AP (`/edit`; `/littlefs_put` answered "Disk full?").
      The AP sent the first image as G5 (imgtype 0x31, 2477 bytes).
      First attempt: downloaded fine, decode failed every time (JTAG:
      `curDataInfo` cleared, `curBlock.blockId` 1). The same file decoded
      correctly on the host (`g5.c` + a RAM flash mock), so the cause was
      on the tag: `eepromRead()` didn't wait for the page program
      `eepromWrite()` leaves running. Fixed in `common/eeprom.c`
- [x] End-to-end image push (2026-11-16 09:21): AP shows `hash`
      `bc8825c3691439f6`, `pending` 0, `updatecount` 1. Panel: black and
      yellow both right, but rows 0–~40 (the slot's first 4 KB sector)
      corrupted: `eepromErase()` sent WREN while the download's last page
      program was running, so the first sector's erase was ignored and the
      decoded rows were ANDed onto old data. Fixed (busy wait before each
      WREN). Run 2 (09:28, hash `10f73018ed450883`): borders, a 2 px black
      and a 4 px yellow line under the top border all correct (user)
- [~] Main loop: sleep what the AP's `AvailDataInfo.nextCheckIn` says, not a
      fixed 40 s capped by DEBUG_MAX_SLEEP (done in `oepl_74h.c`, DEBUG_MAX_SLEEP
      dropped; 2026-11-16 09:31 the AP offered 59 min and the tag stopped
      checking in, as expected; confirm the check-in near 10:31).
      `ChromaAeon74/main.c` still has this commented out. The
      AP models the tag's sleep from what it offered and holds new content
      until the expected check-in is < 5 min away; with the fixed loop it
      held run 2 for an hour. With AP "Shorten latency during config" on and
      the web UI open, the AP offers short sleeps
- [x] Watchdog (2026-11-16): TI driver, reset on the first timeout (NMI
      callback), debug stall on. `wdt10s`/`wdt60s`/`wdtSet()`; sleeps go in
      ≤ 20 min chunks with their own period; each EPD busy wait sets 160 s.
      The NMI leaves a `#pragma NOINIT` marker, so the next boot reports
      WAKEUP_REASON_WDT_RESET (a watchdog reset is turned into a pin reset,
      the reset source can't tell). Verified with a `WDT_TEST` build: the AP
      logged "Watchdog reset". After a JTAG session the reset halts in the boot
      ROM until a power cycle (bench only)
- [x] XferComplete after the refresh (run 3, 09:58: acknowledged ~1 min after
      the download, i.e. after the draw)
- [x] TX power 10 dBm (`0x38D3`, the CC13x0 PA table entry; the stock PA
      table at flash 0xF1A0 tops out there too). Was `0x3E92`, in no TI table
- [x] Code size: Chroma74H builds with `--opt_level=2 --opt_for_speed=0
      --printf_support=nofloat`: 52.5 KB → 43.7 KB. Run 3 drawn correctly with
      it (bit-banged EPD SPI still fine). `Release` config removed
- [x] Wake reason: TIMED after any AvailDataInfo, not only NOUPDATE (a
      boot reason sent again after a transfer made the AP clear the hash and
      re-render)
- [x] AP commands (`OEPL_COMMANDS`): reboot and scan verified (AP logged
      "Booting" / "Network scan"); deep sleep and erase images untested. The
      command byte must be copied before XferComplete (its ACK overwrites the
      radio buffer `avail` points into)
- [~] Preloads / special images (`OEPL_PRELOADS`): header stores the argument,
      downloads go to `xferImgSlot` and skip the displayed slot and stored
      specials, a new special replaces the old one, the main loop draws the
      NOAPFOUND / LOST_CONNECTION image and redraws the current one when the AP
      is back. Verified: an `/imgupload` with `preloadtype=2` was stored and
      acknowledged without a redraw. Note: the AP keeps that upload as the
      tag's content and resends it as a plain image after a boot check-in
- [~] AP: `g5Compress()` takes the buffer size as `uint16_t`, so an 800×960
      two-plane frame (96000 bytes) wraps to 30464: images compressing above
      ~30 KB fail with `G5_DATA_OVERFLOW` and go out raw (96000 bytes, handled).
      Fixed on AP fork branch `fix/g5-compress-size` (e17e3931), not compiled
      yet (local PlatformIO fails with CreateProcess errors)
- [x] Upstream IDs: `CHROMA_74H 0x87` on shared-definitions branch
      `chroma74h-hwid` (836000e, in `../Tag_FW_Chroma/shared`); tagtype on AP
      fork branch `feat/chroma74h-tagtype` (be436cc1). Local only
- [~] RE session answers (2026-11-16): temperature table not needed (above);
      stock TX power defaults to the PA table's last entry 0x38D3 = ours; SSI1
      uses 9-bit frames with CS on DIO2 by hand, clock and SPI mode not found
      yet (mode 0 inferred); OTA stages at SPI 0xB000 and writes internal flash
      through ROM driverlib, the validation / copy / power-loss design still
      to trace (asked)
- [x] Built-in screens (webcam-checked 2026-11-16): EPD layers (rectangles +
      bb_epaper 8x8 font, scaled) rendered row by row while streaming, no
      frame buffer. Info screen (MAC, FW, battery, temperature, AP status) on
      a first boot with no image or with no AP; "NO AP" badge over the current
      image when the AP is lost and no preload exists
- [x] HardFault handler: records CFSR/HFSR/BFAR/stack in `gFault` (NOINIT),
      resets, next boot reports a watchdog reset
- [x] OTA through a bootloader (2026-11-16): `bootloader_CC1310` (3.7 KB in
      sector 0 + the stock-identical CCFG) and the app linked at 0x1000
      (`--define=OEPL_APP`). The app checks the downloaded file (magic, hwType,
      header and image CRC32, vectors, version != running, battery >= 2400 mV),
      writes an apply record in SPI flash (0x1F000, armed last) and resets;
      the bootloader re-checks the CRC, copies sectors 1..14 with read-back and
      retries, verifies, marks the record done and jumps. A reset mid-copy
      repeats it. Verified: v3 (JTAG, `chroma74h_full`) -> v4 by OTA from the AP
      (content mode 5); internal flash read back equals the v4 image, AP shows
      ver 4. Stock has no internal-flash self-update (RE project firmware.md 7)
- [x] Boot screens, as on the CC1110 Chroma tags (v5, webcam-checked
      2026-11-16): splash (MAC, FW, battery, temperature) on power-up or
      reboot, then "Waiting for data" with AP MAC, channel, RSSI, LQI; a
      preloaded SPLASHSCREEN / APFOUND image replaces either. Skipped after a
      watchdog reset or crash (a reset loop would refresh the panel each time)
- [x] Quick re-check after a boot check-in (v6, verified 2026-11-16): the AP's
      radio answers the boot check-in with a sleep offered before the reboot
      (up to maxsleep), so the tag checks in again after 40 s instead; image
      back ~3 min after power-up instead of after the old sleep. AP side
      (drop that offer on a boot / scan / watchdog check-in, so the AP renders
      at once even without "Shorten latency"): branch
      `fix/boot-checkin-sleep-offer` in worktree `../OEPL-boot-sleep-fix`,
      not compiled yet (same PlatformIO problem as above)
- [~] Panel gradient (2026-11-16): at 3.0 V and above the panel draws black
      in the top part and increasingly yellow/brown below; the driven part
      shrinks as the supply rises (~1/2 of the panel at 2.6 V on batteries,
      1/3 at 3.3 V, even at ~2.3 V). Stock does the same; command stream,
      SPI path, temperature and booster strength ruled out (CLAUDE.md).
      Suspects left: the panel's power rails / reservoir caps on the board,
      the flex's hot-bar bond, or the panel. Rail measurements need finer
      probes than are at hand; not pursued for now
- [x] Code review of the branch (2026-11-17), fixed in FW 7 (OTA):
      the bootloader's startup zeroed the RAM holding the reset records
      (`gWdtResetMagic`, `gFault`, `gEpdTrace` at 0x20000100), so since FW 4
      a watchdog reset or crash reported as a first boot and drew the boot
      screens; now linked at 0x20001800. Firmware downloads bigger than the
      OTA area are refused (they'd overwrite image slot 0). An OTA file whose
      image is already running is refused whatever version it claims. A
      preload no longer leaves its version in `curDataInfo`, where the same
      pixels sent later as the regular image were acked without a draw.
      FW 6 → 7 over OTA on batteries, tag closed: installed 2026-11-17 05:04
      (queued at 00:15, but the AP queues no content inside its night window,
      sleeptime 0–5, so it went out at the first check-in after 05:00)
- [ ] Bootloader (needs JTAG, not OTA-able): if a copy fails 5 times on a
      sector it still jumps to the app when the vectors look valid, i.e. into
      a half-written image that may hang before its watchdog starts. Better:
      reset and retry while the record is armed. Low risk (only seen if
      internal flash programming fails; OTA needs >= 2400 mV)
- [x] Supply voltage during draws: sampled from BATMON in the busy waits
      (`gEpdDiag.MinMv/MaxMv/Samples`); `REPORT_REFRESH_MV` reports the minimum
      to the AP on the next check-in. The 74H
      runs on CR2450 lithium coin cells, 6 in parallel. 2 cells: 2558 mV at
      the lowest during a boot screen, idle ~2.87-3.2 V. 4 cells, FW 7, hourly weather
      draws (2026-11-17): rest 2738-2781 mV at 05:00, refresh minimum 2468 mV
      at 00:25 and 2292 mV at 09:05. With the switch on, the AP's battery
      field alternates between the two kinds of reading; a PC logger
      (polling `get_db`, one row per check-in, marked rest/refresh) separates them
- [ ] Sleep current: never measured. Only the battery trend shows it for now

## README to-do list (all CC13xx tags)

- [x] gzip support for image downloads
- [x] Battery voltage and temperature measurement and reporting
- [x] UI: startup info screens and AP-lost screen (74H)
- [ ] Status icons on normal screens (AP lost, low battery)
- [ ] Static screens
- [x] EPD interface on SPI hardware instead of bit banging (74H)
- [ ] Replace the TI NVS driver with something simpler (probably needed for OTA)
- [x] OTA (74H, bootloader)
- [ ] Check Tx power
- [ ] Audit current consumption (Aeon 74 sleeps at 4 mA). Start with pin
      states during sleep; see CLAUDE.md, "CC13xx/CC26xx traps"
