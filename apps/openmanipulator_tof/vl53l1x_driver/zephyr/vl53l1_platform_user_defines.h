/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _VL53L1_PLATFORM_USER_DEFINES_H_
#define _VL53L1_PLATFORM_USER_DEFINES_H_

#define VL53L1DevStructGetLLDriverHandle(Dev) (&Dev->Data.LLData)
#define VL53L1DevStructGetLLResultsHandle(Dev) (&Dev->Data.llresults)

#define DISABLE_WARNINGS()
#define ENABLE_WARNINGS()

#define WARN_OVERRIDE_STATUS(__X__) (void)0

#endif  /* _VL53L1_PLATFORM_USER_DEFINES_H_ */
