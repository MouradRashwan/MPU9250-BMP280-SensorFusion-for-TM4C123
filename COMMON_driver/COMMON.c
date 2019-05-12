/*
 * COMMON.c
 *
 *  Created on: Mar 25, 2019
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "COMMON_driver.h"

void delayMillis(uint32_t ui32Millis)
{
    uint32_t ix;
    uint32_t ui32Period = (SysCtlClockGet() / 1000U) / 3U;
    for (ix = 0; ix < ui32Millis; ix++)
    {
        SysCtlDelay(ui32Period);
    }
}

void getValueWithinLimits(uint32_t * const pui32Value, const uint32_t ui32Max,
                          const uint32_t ui32Min)
{
    if (*pui32Value > ui32Max)
    {
        *pui32Value = ui32Max;
    }
    else if (*pui32Value < ui32Min)
    {
        *pui32Value = ui32Min;
    }
}
