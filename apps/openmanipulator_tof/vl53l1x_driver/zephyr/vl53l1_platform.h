/* vl53l1x_platform.h - Zephyr customization of ST vl53l1x library.
 * (library is located in ext/hal/st/lib/sensor/vl53l1x/)
 */

/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_VL53L1X_VL53L1X_PLATFORM_H_
#define ZEPHYR_DRIVERS_SENSOR_VL53L1X_VL53L1X_PLATFORM_H_

#include "vl53l1_def.h"
#include "vl53l1_platform_log.h"
#include "vl53l1_platform_user_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct VL53L1X_Dev_t
 * @brief  Generic PAL device type that does link between API and platform
 * abstraction layer
 *
 */
typedef struct {
	VL53L1_DevData_t Data;    /* embed ST Ewok Dev  data as "Data"*/
	/*!< user specific field */
	uint8_t   I2cDevAddr;      /* i2c device address user specific field */
	uint8_t   comms_type;      /* VL53L1X_COMMS_I2C or VL53L1X_COMMS_SPI */
	uint16_t  comms_speed_khz; /* Comms speed [kHz] */
	struct device *i2c;
} VL53L1X_Dev_t;


/**
 * @brief Declare the device Handle as a pointer of the structure VL53L1X_Dev_t
 *
 */
typedef VL53L1X_Dev_t *VL53L1_DEV;

/**
 * @def PALDevDataGet
 * @brief Get ST private structure @a VL53L1_DevData_t data access
 *
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * It maybe used and as real data "ref" not just as "get" for sub-structure item
 * like PALDevDataGet(FilterData.field)[i]
 * or PALDevDataGet(FilterData.MeasurementIndex)++
 */
#define PALDevDataGet(Dev, field) (Dev->Data.field)

/**
 * @def PALDevDataSet(Dev, field, data)
 * @brief  Set ST private structure @a VL53L1_DevData_t data field
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * @param data      Data to be set
 */
#define PALDevDataSet(Dev, field, data) ((Dev->Data.field) = (data))


/**
 * @defgroup VL53L1X_registerAccess_group PAL Register Access Functions
 * @brief    PAL Register Access Functions
 *  @{
 */

/**
 * Writes the supplied byte buffer to the device
 * @param Dev    Device Handle
 * @param index  The register index
 * @param pdata  Pointer to uint8_t buffer containing the data to be written
 * @param count  Number of bytes in the supplied byte buffer
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, int index, uint8_t *pdata,
				 uint32_t count);

/**
 * Reads the requested number of bytes from the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to the uint8_t buffer to store read data
 * @param   count     Number of uint8_t's to read
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, int index, uint8_t *pdata,
				uint32_t count);

/**
 * Write single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      8 bit register data
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, int index, uint8_t data);

/**
 * Write word register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      16 bit register data
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, int index, uint16_t data);

/**
 * Write double word (4 byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      32 bit register data
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, int index, uint32_t data);

/**
 * Read single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 8 bit data
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, int index, uint8_t *data);

/**
 * Read word (2byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 16 bit data
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, int index, uint16_t *data);

/**
 * Read dword (4byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 32 bit data
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, int index, uint32_t *data);

/**
 * Threat safe Update (read/modify/write) single byte register
 *
 * Final_reg = (Initial_reg & and_data) |or_data
 *
 * @param   Dev        Device Handle
 * @param   index      The register index
 * @param   AndData    8 bit and data
 * @param   OrData     8 bit or data
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, int index,
				 uint8_t AndData, uint8_t OrData);

/** @} end of VL53L1X_registerAccess_group */


/**
 * @brief execute delay in all polling API call
 *
 * A typical multi-thread or RTOs implementation is to sleep the task
 * for some 5ms (with 100Hz max rate faster polling is not needed)
 * if nothing specific is need you can define it as an empty/void macro
 * @code
 * #define VL53L1X_PollingDelay(...) (void)0
 * @endcode
 * @param Dev       Device Handle
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_PollingDelay(VL53L1_DEV Dev);
/* usually best implemented as a real function */

/**
 * @brief execute delay
 *
 * Sleeps for certain microseconds
 * @param Dev       Device Handle
 * @param wait_us   Time to sleep in us
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_WaitUs(VL53L1_DEV Dev, int32_t wait_us);

/**
 * @brief execute delay
 *
 * Sleeps for certain miliseconds
 * @param Dev       Device Handle
 * @param wait_ms   Time to sleep in ms
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_WaitMs(VL53L1_DEV Dev, int32_t wait_ms);

/**
 * Modifies a bit in a register with a certain timeout
 * @param Dev       		Device Handle
 * @param timeout_ms   		Timeout in ms
 * @param index       		Register index
 * @param value       		Expected bit value
 * @param mask      		Bit mask
 * @param poll_delay_ms  	Poll delay in ms
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_DEV Dev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms);

/**
 * @brief gets system tickcount
 *
 * Returns system tickcount
 * @param ptick_count_ms       Pointer to uint32_t where data is goint to be written
 * @return  VL53L1_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms);

/** @} end of VL53L1X_platform_group */

#ifdef __cplusplus
}
#endif

#endif  /* ZEPHYR_DRIVERS_SENSOR_VL53L1X_VL53L1X_PLATFORM_H_ */
