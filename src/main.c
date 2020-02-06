/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/util.h>
#include <string.h>
#include <sys/printk.h>

void main(void)
{	
	if (strlen(CONFIG_UART_CONSOLE_ON_DEV_NAME) !=
	    strlen("CDC_ACM_0") ||
	    strncmp(CONFIG_UART_CONSOLE_ON_DEV_NAME, "CDC_ACM_0",
		    strlen(CONFIG_UART_CONSOLE_ON_DEV_NAME))) {
		printk("Error: Console device name is not USB ACM\n");

	}

	printk("Hello World! %s\n", CONFIG_ARCH);


	struct device *dev = device_get_binding(DT_INST_0_ST_VL53L1X_LABEL);
	struct sensor_value value;
	int ret;

	if (dev == NULL) {
		printk("Could not get VL53L0X device\n");
		return;
	}

	while (1) {
		ret = sensor_sample_fetch(dev);
		if (ret) {
			printk("sensor_sample_fetch failed ret %d\n", ret);
		}

		ret = sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &value);
		// printf("distance is %.3fm\n", sensor_value_to_double(&value));
		printf("distance is %d mm\n", value.val1 + value.val2);

		// k_sleep(K_MSEC(200));
	}
}
