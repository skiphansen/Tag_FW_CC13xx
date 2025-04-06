#ifndef _RADIO_H_
#define _RADIO_H_

#define COMMS_MAX_RADIO_WAIT_MSEC		200

#define COMMS_RX_ERR_NO_PACKETS			(-1)
#define COMMS_RX_ERR_INVALID_PACKET		(-2)

#define COMMS_MAX_PACKET_SZ				(127)

extern uint8_t  mLastLqi;
extern int8_t  mLastRSSI;
extern uint8_t gChannel;

extern uint8_t gSubGhzBand;
#define BAND_UNKNOWN    0
#define BAND_868        1
#define BAND_915        2

uint8_t *commsRxUnencrypted(uint32_t Wait4Ms);
bool commsTxUnencrypted(const void  *packetP, uint8_t len);
bool radioRxEnable(bool on);
void radioRxFlush(void);
bool radioInit(void);
bool radioSetChannel(uint8_t channel);
bool commsTxNoCpy(const void  *packetp);

#endif   // _RADIO_H_
