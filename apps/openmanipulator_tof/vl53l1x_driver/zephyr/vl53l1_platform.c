/* vl53l1x_platform.c - Zephyr customization of ST vl53l1x library.
 * (library is located in ext/hal/st/lib/sensor/vl53l1x/)
 */

/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "vl53l1_platform.h"

#include <drivers/sensor.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/i2c.h>
#include <logging/log.h>

LOG_MODULE_DECLARE(VL53L1X, CONFIG_SENSOR_LOG_LEVEL);

#define U16_MSB(X) ((X >> 8) & 0xFF)
#define U16_LSB(X) (X & 0x00FF)
#define INVERT_U16(X) (X << 8 & 0xFF00) | (X >> 8 & 0x00FF)

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, int index, uint8_t *pdata,
				 uint32_t count)
{

	VL53L1_Error Status = VL53L1_ERROR_NONE;
	int32_t status_int = 0;
	uint8_t I2CBuffer[count+2];

	I2CBuffer[0] = (index >> 8);
	I2CBuffer[1] = index;
	memcpy(&I2CBuffer[2], pdata, count);

	status_int = i2c_write(Dev->i2c, I2CBuffer, count+2, Dev->I2cDevAddr);

	if (status_int < 0) {
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
		LOG_ERR("Failed to write");
	}

	return Status;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, int index, uint8_t *pdata,
				uint32_t count)
{
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	int32_t status_int;
	u16_t index_inv = INVERT_U16(index);

	status_int = i2c_write_read(Dev->i2c, Dev->I2cDevAddr, &index_inv, 2, pdata, count);

	if (status_int < 0) {
		LOG_ERR("Failed to read");
		return -EIO;
	}

	return Status;
}


VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, int index, uint8_t data)
{
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	int32_t status_int;

	u8_t tx_buf[3] = {U16_LSB(index), U16_MSB(index), data};

	status_int = i2c_write(Dev->i2c, tx_buf, 3, Dev->I2cDevAddr);

	if (status_int < 0) {
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
		LOG_ERR("i2c_reg_write_byte failed (%d)", Status);
	}

	return Status;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, int index, uint16_t data)
{
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	int32_t status_int;
	uint8_t I2CBuffer[4];

	I2CBuffer[0] = U16_LSB(index);
	I2CBuffer[1] = U16_MSB(index);
	I2CBuffer[2] = data >> 8;
	I2CBuffer[3] = data & 0x00FF;

	status_int = i2c_write(Dev->i2c, I2CBuffer, 4, Dev->I2cDevAddr);
	if (status_int < 0) {
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
		LOG_ERR("i2c_write failed (%d)", Status);
	}

	return Status;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, int index, uint32_t data)
{
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	int32_t status_int;
	uint8_t I2CBuffer[6];

	I2CBuffer[0] = U16_LSB(index);
	I2CBuffer[1] = U16_MSB(index);
	I2CBuffer[2] = (data >> 24) & 0xFF;
	I2CBuffer[3] = (data >> 16) & 0xFF;
	I2CBuffer[4] = (data >> 8)  & 0xFF;
	I2CBuffer[5] = (data >> 0) & 0xFF;

	status_int = i2c_write(Dev->i2c, I2CBuffer, 6, Dev->I2cDevAddr);
	if (status_int < 0) {
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
		LOG_ERR("i2c_write failed (%d)", Status);
	}

	return Status;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, int index,
				 uint8_t AndData, uint8_t OrData)
{
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	int32_t status_int;
	uint8_t deviceAddress;
	uint8_t data;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L1_RdByte(Dev, index, &data);
	if (status_int < 0) {
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
		LOG_ERR("VL53L1X_RdByte failed (%d)", Status);
	}

	if (Status == VL53L1_ERROR_NONE) {
		data = (data & AndData) | OrData;
		status_int = VL53L1_WrByte(Dev, index, data);
		if (status_int != 0) {
			Status = VL53L1_ERROR_CONTROL_INTERFACE;
			LOG_DBG("VL53L1X_WrByte failed.(%d)", Status);
		}
	}

	return Status;
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, int index, uint8_t *data)
{
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	int32_t status_int;

	u16_t index_inv = INVERT_U16(index);


	status_int = i2c_write_read(Dev->i2c, Dev->I2cDevAddr, &index_inv, 2, data, 1);

	if (status_int < 0) {
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
		LOG_ERR("i2c_reg_read_byte failed (%d)", Status);
	}

	return Status;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, int index, uint16_t *data)
{
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	int32_t status_int;
	uint8_t buf[2];

	u16_t index_inv = INVERT_U16(index);

	status_int = i2c_write_read(Dev->i2c, Dev->I2cDevAddr, &index_inv, 2, buf, 2);

	if (status_int < 0) {
		LOG_ERR("i2c_burst_read failed");
		return -EIO;
	}
	*data = ((uint16_t)buf[0]<<8) + (uint16_t)buf[1];

	return Status;
}

VL53L1_Error  VL53L1_RdDWord(VL53L1_DEV Dev, int index, uint32_t *data)
{
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	int32_t status_int;
	u8_t buf[4];

	u16_t index_inv = INVERT_U16(index);

	status_int = i2c_write_read(Dev->i2c, Dev->I2cDevAddr, &index_inv, 2, buf, 4);

	if (status_int < 0) {
		LOG_ERR("i2c_burst_read failed");
		return -EIO;
	}
	*data = ((uint32_t)buf[0]<<24) + ((uint32_t)buf[1]<<16) +
		((uint32_t)buf[2]<<8) + (uint32_t)buf[3];

	return Status;
}

VL53L1_Error VL53L1_PollingDelay(VL53L1_DEV Dev)
{
	k_sleep(K_MSEC(2));
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_DEV Dev, int32_t wait_us)
{	
	k_sleep(K_MSEC(wait_us/USEC_PER_MSEC));
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_DEV Dev, int32_t wait_ms)
{
	k_sleep(K_MSEC(wait_ms));
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_DEV Dev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
  uint8_t data;
  VL53L1_Error status;

  while (timeout_ms > 0) {
    status = VL53L1_RdByte(Dev, index, &data);
    if (status != VL53L1_ERROR_NONE) {
		return status;
	}
    if ((data & mask) == value) {
		return VL53L1_ERROR_NONE;
	}
	k_sleep(K_MSEC(poll_delay_ms));
    timeout_ms -= (poll_delay_ms < timeout_ms) ? poll_delay_ms : timeout_ms;
  }

  return VL53L1_ERROR_TIME_OUT;
}

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
  *ptick_count_ms = k_uptime_get();
	return VL53L1_ERROR_NONE;
}
