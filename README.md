The repo will eventually contain source for [OEPL](https://openepaperlink.org/) firmware for various price tags based on the TI CC13xx.

Currently a port is underway for the Aeon 74 and it progressing nicely ...

<a href="https://github.com/user-attachments/assets/281b216f-07ac-4771-9a7c-0b6f70877d65"> <img src="https://github.com/user-attachments/assets/281b216f-07ac-4771-9a7c-0b6f70877d65" width=70%></a>

See the OEPL [Wiki](https://github.com/OpenEPaperLink/OpenEPaperLink/wiki/Chroma-Aeon-74) for more information.

## Known tags base on CC13xx SOCs

1. Chroma 21 (SN starting with 'MJ')
2. Chroma 29 (SN starting with 'ME')
3. Chroma 74H+ (SN starts with 'MS')
4. Chroma Aeon 74 (SN starts with 'SR')
5. Solum 2.9″ EL029D2WRA

## Port Status

This port is currently able to download screen updates from the AP and display
them.  It is **NOT** ready for prime time nor is it advisable to run the
tag from batteries yet.  Currently the "sleep" current consumption is **4 ma**.

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


