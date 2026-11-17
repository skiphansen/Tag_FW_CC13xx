## OEPL firmware for tags based on CC13xx SOCs

The repo contains source for [OEPL](https://openepaperlink.org/) 
firmware for various price tags based on the TI CC13xx.

So far the following tags are known to be based on CC13xx SOCs

| Tag | SOC | [bb_epaper_type](https://github.com/skiphansen/bb_epaper) |
| -| - | - |
|Chroma 21, SN starting with 'MJ' | CC1310 ||
|Chroma 21, SN starting with 'LD' | CC1310 ||
|Chroma 29, SN starting with 'ME' | CC1310 |EP_CHROMA29_CC1310 (38) |
|Chroma 74H, SN starting with 'MS' | CC1310-F64 | own driver, `oepl_CC1310/epd_74h.c` |
|Chroma Aeon 74, SN starting with 'SR'| CC1311 | EP75R_800x480 (33) |
|Solum 2.9" EL029D2WRA| CC1310-F128 ||

It is very likely that CC13xx based tags exist for the entire Chroma product 
line.

## Port Status

Currently a port is underway for the [Aeon 74](https://github.com/OpenEPaperLink/OpenEPaperLink/wiki/Chroma-Aeon-74) and is progressing nicely ...

<a href="https://github.com/user-attachments/assets/281b216f-07ac-4771-9a7c-0b6f70877d65"> <img src="https://github.com/user-attachments/assets/281b216f-07ac-4771-9a7c-0b6f70877d65" width=70%></a>

This port is currently able to download screen updates from the AP and display
them on Chroma Aeon 74 displays.  

<a href="https://github.com/user-attachments/assets/3b35ceea-7a70-47ed-a8db-4ed0c87e9f34"> <img src="https://github.com/user-attachments/assets/3b35ceea-7a70-47ed-a8db-4ed0c87e9f34" width=70%></a>

> [!CAUTION]
> This code is **NOT** ready for prime time nor is it advisable to run the tag from 
>batteries yet.<br>Currently the "sleep" current consumption is currently **4 ma**,
>i.e. broken.

Since these displays are currently rare and only a few have made their way
into the hands of hobbyists it's a low priority project.  

To Do List:

- [X] Add gzip support for image downloads
- [X] Add support for battery voltage and temperature measurements and reporting.
- [ ] Add UI support (initial startup info screens and AP lost screen)
- [ ] Add status icons to normal displays (AP lost and low battery)
- [ ] Add support for static screens
- [ ] Modify EPD interface to use SPI hardware rather than bit banging.
- [ ] Replace TI NVS driver with something simpler (probably needed for OTA support)
- [ ] Add OTA support
- [ ] Check Tx power
- [ ] Audit current consumption


### Chroma 74H (alpha)

`oepl_CC1310/` (make config `Chroma74H`) is an OEPL port for the Chroma 74H
(CC1310F64, 800×480 black/white/yellow). It checks in with a sub-GHz AP,
draws zlib, G5 and raw images, shows the OEPL boot screens, survives
watchdog resets and updates over the air through its own bootloader
(`bootloader_CC1310/`). It reports hwType 0x81 like the Aeon 74 and uses
the AP's `81.json` (whose red shows as yellow on this panel).

> [!WARNING]
> **Alpha, for testing only.** It runs on its CR2450 coin cells, but its
> sleep current hasn't been measured, so battery life is unknown.

Notes:

- The panel draws evenly only at a low supply. On a 3.0–3.3 V bench supply,
  black pixels shade from black at the top to yellow lower down; white areas
  stay white. The stock firmware does the same. On its coin cells, which sag
  during a refresh, black is even or close to it.
- Builds, image layout, flashing and the hardware notes are in `CLAUDE.md`;
  progress and open items in `PLAN.md`.
- Flash it over JTAG (take the vendor dumps first, the stock firmware can't be
  recovered otherwise): `chroma74h_full_<ver>.bin` from
  `tools/make_74h_images.py`, then OTA with `chroma74h_ota_<ver>.bin`.

## Building the firmware

1. Download and install Ti [CCSTUDIO version 20](https://www.ti.com/tool/CCSTUDIO?utm_source=google&utm_medium=cpc&utm_campaign=epd-der-null-58700007779115352_code_composer_rsa-cpc-evm-google-ww_en_int&utm_content=code_composer&ds_k=code+composer&gclsrc=aw.ds&gad_source=1&gad_campaignid=12788839648&gbraid=0AAAAAC068F0RJTf4-Dd0hRx-eqaFCMEmV&gclid=CjwKCAiA8vXIBhAtEiwAf3B-g42xwAPozrqZLLbJRnWgzASeJD60egSehWx-9DEA2JrL0XwsF2CUERoCHesQAvD_BwE#downloads).
1. Download and install [SIMPLELINK-CC13XX-CC26XX-SDK version 8.30.01.01](https://www.ti.com/tool/download/SIMPLELINK-LOWPOWER-F2-SDK/8.30.01.01) for CC1311 based tags.
1. Download and install [SIMPLELINK_CC13X0_SDK version 4.20.2](https://www.ti.com/tool/download/SIMPLELINK-CC13X0-SDK/4.20.02.07) for CC1310 based tags.
1. Download and install [Arm compiler (ARM-CGT) version 18.12.8](https://www.ti.com/tool/download/ARM-CGT/18.12.8.LTS) for CC1310 based tags.
1. Clone this repo recursively (or Run git submodule init and git submodule update after cloning).
1. Run CCSTUDIO and select "Add Folder to Workspace...".
1. Navigate to the desired project and click "Open".
1. Select "Build Projects" from the Project menu".

Warnings are expected, but there shouldn't be any errors.

### Chroma 74H release images from the command line (macOS, Linux, WSL)

Install ARM-CGT 18.12.8 and SIMPLELINK_CC13X0_SDK 4.20.02.07 into `~/ti`,
then run `tools/build_release.sh`. It builds the bootloader and the app and
writes `chroma74h_full_<ver>.bin` (JTAG) and `chroma74h_ota_<ver>.bin` (OTA)
to `oepl_CC1310/build/release/`. In WSL, with the repo on the Windows
filesystem (`/mnt/c/...`), it builds in a copy under `~/build` for speed and
copies the results back.

On Apple Silicon the TI installers refuse to start. With Rosetta installed
(`softwareupdate --install-rosetta`), run the installer binary directly:

```
arch -x86_64 <installer>.app/Contents/MacOS/osx-x86_64 --mode unattended --prefix ~/ti
```

## Flashing tags

An xds110 or other cJTAG compatible programmer is needed to program CC13xx based tags.
Ti's CCSTUDIO includes direct support xds110 debuggers.

See this Wiki [page](https://github.com/OpenEPaperLink/OpenEPaperLink/wiki/Chroma-Aeon-74#programming)
for the Chroma Aeon74 and other tags based on the CC1311 for programmer connections.

See this Wiki [page](https://github.com/OpenEPaperLink/OpenEPaperLink/wiki/CC1310-Based-Chroma-Tags#pin-connections)
for Chroma tags based on the CC1310 for programmer connections.
