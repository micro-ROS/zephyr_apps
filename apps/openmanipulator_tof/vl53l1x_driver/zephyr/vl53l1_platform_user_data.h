/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _VL53L1_PLATFORM_USER_DATA_H_
#define _VL53L1_PLATFORM_USER_DATA_H_

#define VL53L1DevDataSet(Dev, field, data) ((Dev->Data.field) = (data))
#define VL53L1DevDataGet(Dev, field) (Dev->Data.field)

#endif  /* _VL53L1_PLATFORM_USER_DATA_H_ */
