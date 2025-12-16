## OEPL firmware for tags based on CC13xx SOCs

The repo contains source for [OEPL](https://openepaperlink.org/) 
firmware for various price tags based on the TI CC13xx.

So far the following tags are known to be based on CC13xx SOCs

| Tag | SOC | [bb_epaper_type](https://github.com/skiphansen/bb_epaper) |
| -| - | - |
|Chroma 21, SN starting with 'MJ' | CC1310 ||
|Chroma 29, SN starting with 'ME' | CC1310 |EP_CHROMA29_CC1310 (38) |
|Chroma 74H, SN starting with 'MS' | CC1310 ||
|Chroma Aeon 74, SN starting with 'SR'| CC1311 | EP75R_800x480 (33) |
|Solum 2.9" EL029D2WRA| ? |

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

## Flashing tags

An xds110 or other cJTAG compatible programmer is needed to program CC13xx based tags.
Ti's CCSTUDIO includes direct support xds110 debuggers.

See this Wiki [page](https://github.com/OpenEPaperLink/OpenEPaperLink/wiki/Chroma-Aeon-74#programming)
for the Chroma Aeon74 and other tags based on the CC1311 for programmer connections.

See this Wiki [page](https://github.com/OpenEPaperLink/OpenEPaperLink/wiki/CC1310-Based-Chroma-Tags#pin-connections)
for Chroma tags based on the CC1310 for programmer connections.

