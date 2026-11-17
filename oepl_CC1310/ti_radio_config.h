// CC13x0 stand-in for the SysConfig generated ti_radio_config.h that
// common/radio.c expects on CC13x1. Maps its "custom868" names onto the
// SmartRF Studio settings in smartrf_settings/ (same PHY: 250 kbps GFSK,
// 165 kHz deviation, sync word 0xD391D391).
#ifndef _TI_RADIO_CONFIG_H_
#define _TI_RADIO_CONFIG_H_

// common/RFQueue.h (included first by radio.c) maps the plain names onto the
// custom868 ones; drop that while the SmartRF declarations are read, or they
// would declare the custom868 names. radio.c defines the same mapping again.
#undef RF_prop
#undef RF_cmdPropRadioDivSetup
#undef RF_cmdFs
#undef RF_cmdPropTx
#undef RF_cmdPropRx
#include "smartrf_settings/smartrf_settings.h"

#define RF_prop_custom868_0                  RF_prop
#define RF_cmdPropRadioDivSetup_custom868_0  RF_cmdPropRadioDivSetup
#define RF_cmdFs_custom868_0                 RF_cmdFs
#define RF_cmdPropTx_custom868_0             RF_cmdPropTx
#define RF_cmdPropRx_custom868_0             RF_cmdPropRx

#endif   // _TI_RADIO_CONFIG_H_
