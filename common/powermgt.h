#ifndef _POWERMGT_H_
#define _POWERMGT_H_
#include <stdint.h>

// power saving algorithm

// interval (in seconds) (when 1 packet is sent/received) for target current
#define INTERVAL_BASE 40              

// interval (in seconds) (at max attempts) for target average current
// Revisit this later ... a quick test shows a response time of 37 ms, 
// set timeout for much bigger for now
#define INTERVAL_AT_MAX_ATTEMPTS 600  

// How many milliseconds we should wait for a packet during the data_request.
// If the AP holds a long list of data for tags, it may need a little more 
// time to lookup the mac address
#define DATA_REQ_RX_WINDOW_SIZE 200UL  

// How many attempts (at most) we should do to get something back from the AP
#define DATA_REQ_MAX_ATTEMPTS    14      

// How many samples we should use to smooth the data request interval
#define POWER_SAVING_SMOOTHING   8      

//IMPORTANT: Minimum interval for check-in; this determines overal battery life!
#define MINIMUM_INTERVAL         45

// How many attempts to discover an AP the tag should do
#define MAXIMUM_PING_ATTEMPTS    3

// Now long to wait for Ping replies in milliseconds
#define PING_REPLY_WINDOW        150UL

// Now long to wait for transfer complete acks in milliseconds
#define XFER_COMPLETE_REPLY_WINDOW        150UL

// How often (in seconds, approximately) the tag should do a long datareq (including temperature)
#define LONG_DATAREQ_INTERVAL 300

#define BATTERY_VOLTAGE_MINIMUM 2600

#define MAX_RETURN_DATA_ATTEMPTS 15

// power saving when no AP's were found (scanning every X)

// how often we should read voltages; this is done every scan attempt in 
// interval bracket 3
#define VOLTAGEREADING_DURING_SCAN_INTERVAL 2  

#define INTERVAL_1_TIME 3600UL   // Try every hour
#define INTERVAL_1_ATTEMPTS 24   // for 24 attempts (an entire day)
#define INTERVAL_2_TIME 7200UL   // Try every 2 hours
#define INTERVAL_2_ATTEMPTS 12   // for 12 attempts (an additional day)
#define INTERVAL_3_TIME 86400UL  // Finally, try every day

// slideshow power settings
#define SLIDESHOW_FORCE_FULL_REFRESH_EVERY 16  // force a full refresh every X screen draws
#define SLIDESHOW_INTERVAL_FAST 15             // interval for 'fast'
#define SLIDESHOW_INTERVAL_MEDIUM 60  
#define SLIDESHOW_INTERVAL_SLOW 300  
#define SLIDESHOW_INTERVAL_GLACIAL 1800  

#endif
