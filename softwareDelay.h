/*
 * softwareDelay.h
 *
 *  Created on: Oct 21, 2019
 *      Author: Sprints
 */

#ifndef SOFTWAREDELAY_H_
#define SOFTWAREDELAY_H_
#include "std_types.h"
/**
 * Description: this delay function operate in a polling manner
 * 				don't use it with RTOSs
 * @param a_delay_in_ms: the milli-seconds
 */
void softwareDelayMs(uint32_t u32_delay_in_ms);
void softwareDelayus(uint32_t u32_delay_in_us);

#endif /* SOFTWAREDELAY_H_ */