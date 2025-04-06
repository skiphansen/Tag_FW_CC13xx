#ifndef _CC13XX_H_
#define _CC13XX_H_

#include <ti/drivers/dpl/ClockP.h>

#define clock_time() ClockP_getSystemTicks()
#define WaitMs(x) ClockP_usleep(x * 1000)

static inline unsigned clock_time_exceed(uint32_t ref, uint32_t us)
{
	return ((unsigned int)(clock_time() - ref) > us * 10);
}


#endif // _CC13XX_H_

