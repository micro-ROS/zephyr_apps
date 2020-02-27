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
	RCCHECK(rcl_publisher_init(&publisher_measure, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/tof/measure", &publisher_measure_ops))

	rcl_publisher_options_t publisher_trigger_ops = rcl_publisher_get_default_options();
	rcl_publisher_t publisher_trigger = rcl_get_zero_initialized_publisher();
	RCCHECK(rcl_publisher_init(&publisher_trigger, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/tof/trigger", &publisher_trigger_ops))

	rcl_subscription_options_t subscription_debug_ops = rcl_subscription_get_default_options();
	rcl_subscription_t subscription_debug = rcl_get_zero_initialized_subscription();
	RCCHECK(rcl_subscription_init(&subscription_debug, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/tof/verbose", &subscription_debug_ops))

	rcl_subscription_options_t subscription_thr_ops = rcl_subscription_get_default_options();
	rcl_subscription_t subscription_thr = rcl_get_zero_initialized_subscription();
	RCCHECK(rcl_subscription_init(&subscription_thr, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/tof/threshold", &subscription_thr_ops))

	rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  	RCCHECK(rcl_wait_set_init(&wait_set, 2, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()))

	struct device *dev = device_get_binding(DT_INST_0_ST_VL53L1X_LABEL);
	struct sensor_value value;

	if (dev == NULL) {
		printk("Could not get VL53L0X device\n");
		return;
	}


	int32_t threshold = 300;
	uint32_t measure;
	int32_t debug = 2;
	bool state = false;

	while (1) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &value);
		measure = value.val1 + value.val2;
		
		RCSOFTCHECK(rcl_wait_set_clear(&wait_set))
    
		size_t index_subscription_debug;
		RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription_debug, &index_subscription_debug))

		size_t index_subscription_thr;
		RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription_thr, &index_subscription_thr))

		RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(50)))

		if (wait_set.subscriptions[index_subscription_debug]) {
			std_msgs__msg__Int32 msg;
			rcl_take(wait_set.subscriptions[index_subscription_debug], &msg, NULL, NULL);
			debug = msg.data;
		}

		if (wait_set.subscriptions[index_subscription_thr]) {
			std_msgs__msg__Int32 msg;
			rcl_take(wait_set.subscriptions[index_subscription_thr], &msg, NULL, NULL);
			threshold = msg.data;
		}


		if (debug == 2){
			std_msgs__msg__Int32 msg;
			msg.data = measure;
			rcl_publish(&publisher_measure, (const void*)&msg, NULL);
		}

		bool old_state = state;
		state = measure < threshold;

		if (state != old_state || debug >= 1){
			std_msgs__msg__Bool msg;
			msg.data = state;
			rcl_publish(&publisher_trigger, (const void*)&msg, NULL);
		}
		
		gpio_pin_set(led, DT_ALIAS_LED0_GPIOS_PIN, (int)state);


		// Serial print

		printk("%04d mm / %04d mm -> %s [", measure, threshold, state ? " ON" : "OFF");
		
		measure = (measure < 2000) ? measure : 1999;
		int graph = (60*measure) / (2000);
		graph = (graph > 0) ? graph : 0;
		for (size_t i = 0; i < graph; i++){
			printk("-");
		}
		printk("|");
		
		int spaces = (60-graph);
		spaces = (spaces > 0) ? spaces : 0;
		for (size_t i = 0; i < spaces; i++){
			printk(" ");
		}
		printk("]\r");

	}
}
