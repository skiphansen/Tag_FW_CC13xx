#pragma once

#include <stdint.h>
#include <stdbool.h>

extern uint8_t mSelfMac[];
extern uint8_t currentChannel;
extern uint8_t APmac[];

extern uint8_t curImgSlot;

extern void setupRadio(void);
extern void killRadio(void);

extern struct AvailDataInfo *getAvailDataInfo();
extern struct AvailDataInfo *getShortAvailDataInfo();
extern void drawImageFromEeprom(const uint8_t imgSlot);
extern bool processAvailDataInfo(struct AvailDataInfo *avail);
extern void initializeProto();
extern uint8_t detectAP(const uint8_t channel);
void write_ota_firmware_to_flash(void);
void eraseImageSlots(void);
// OEPL_PRELOADS: images the AP preloaded for events (CUSTOM_IMAGE_*)
bool drawSpecialImage(const uint8_t Type);
bool drawCurrentImage(void);
// DATATYPE_COMMAND_DATA, provided by the board's main when OEPL_COMMANDS is
// defined in oepl_board.h
void executeCommand(uint8_t Cmd);
