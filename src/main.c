/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <stdio.h>
#include <sys/util.h>
#include <string.h>
#include <sys/printk.h>

#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#ifndef DT_ALIAS_LED0_GPIOS_FLAGS
#define DT_ALIAS_LED0_GPIOS_FLAGS 0
#endif

static struct device *led;

void main(void)
{	
	led = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	gpio_pin_configure(led, DT_ALIAS_LED0_GPIOS_PIN, GPIO_OUTPUT_ACTIVE | DT_ALIAS_LED0_GPIOS_FLAGS);

	printk("Hello World! %s\n", CONFIG_ARCH);

	rcl_init_options_t options = rcl_get_zero_initialized_init_options();

	RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()))

	// Optional RMW configuration 
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
	RCCHECK(rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options))

	rcl_context_t context = rcl_get_zero_initialized_context();
	RCCHECK(rcl_init(0, NULL, &options, &context))

	rcl_node_options_t node_ops = rcl_node_get_default_options();

	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rcl_node_init(&node, "int32_publisher_rcl", "", &context, &node_ops))

	rcl_publisher_options_t publisher_measure_ops = rcl_publisher_get_default_options();
	rcl_publisher_t publisher_measure = rcl_get_zero_initialized_publisher();
	RCCHECK(rcl_publisher_init(&publisher_measure, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/tof/measure2", &publisher_measure_ops))

	rcl_subscription_options_t subscription_trigger_ops = rcl_subscription_get_default_options();
	rcl_subscription_t subscription_trigger = rcl_get_zero_initialized_subscription();
	RCCHECK(rcl_subscription_init(&subscription_trigger, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/tof/trigger", &subscription_trigger_ops))

	rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  	RCCHECK(rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()))

	struct device *dev = device_get_binding(DT_INST_0_ST_VL53L0X_LABEL);
	struct sensor_value value;

	if (dev == NULL) {
		printk("Could not get VL53L0X device\n");
		return;
	}

	uint32_t measure;

	while (1) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &value);
		measure = value.val1 + value.val2;

		// printf("Distance is %d mm\n", measure);
		
		// RCSOFTCHECK(rcl_wait_set_clear(&wait_set))
    
		// size_t index_subscription_trigger;
		// RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription_trigger, &index_subscription_trigger))

		// RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(50)))

		// if (wait_set.subscriptions[index_subscription_trigger]) {
		// 	std_msgs__msg__Bool msg;
		// 	rcl_take(wait_set.subscriptions[index_subscription_trigger], &msg, NULL, NULL);
		// 	gpio_pin_set(led, DT_ALIAS_LED0_GPIOS_PIN, (int)msg.data);
		// }

		
		std_msgs__msg__Int32 msg;
		msg.data = measure;
		rcl_publish(&publisher_measure, (const void*)&msg, NULL);		
		
		k_sleep(50);
	}
}
