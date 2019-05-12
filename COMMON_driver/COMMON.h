/*
 * COMMON.h
 *
 *  Created on: Mar 25, 2019
 *      Author: Administrator
 */

#ifndef COMMON_H_
#define COMMON_H_

void delayMillis(uint32_t ui32Millis);

void getValueWithinLimits(uint32_t * const pui32Value, const uint32_t ui32Max,
                          const uint32_t ui32Min);

#endif /* COMMON_H_ */
