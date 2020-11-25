#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <stdio.h>
#include <sys/util.h>
#include <string.h>
#include <sys/printk.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define PIN	DT_GPIO_PIN(DT_ALIAS(led0), gpios)

static struct device *led;
int32_t debug = 2;

rcl_publisher_t tof_publisher;
rcl_publisher_t trigger_publisher;
rcl_subscription_t verbosity_subscription;
rcl_subscription_t thr_subscription;

std_msgs__msg__Bool verbosity_msg;
std_msgs__msg__Int32 thr_msg;

int32_t threshold = 300;

void verbosity_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	debug = msg->data;
}

void thr_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	threshold = msg->data;
}

void main(void)
{	
	// ---- Devices configuration ----
	led = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
	gpio_pin_configure(led, PIN, GPIO_OUTPUT_ACTIVE | 0);

	struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, st_vl53l1x)));
	struct sensor_value value;

	if (dev == NULL) {
		printk("Could not get VL53L0X device\n");
	}

	// ---- micro-ROS configuration ----
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "zephyr_sensors_node", "", &support));


	// Creating TOF publisher
	RCCHECK(rclc_publisher_init_default(&tof_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/tof/measure"));

	// Creating a trigger publisher
	RCCHECK(rclc_publisher_init_default(&trigger_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/tof/trigger"));
	// trigger_publisher_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

	// Creating a verbosity subscriber
	RCCHECK(rclc_subscription_init_default(&verbosity_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/tof/verbose"));

	// Creating a thresold subscriber
	RCCHECK(rclc_subscription_init_default(&thr_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/tof/threshold"));

	// Creating a executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &verbosity_subscription, &verbosity_msg, &verbosity_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &thr_subscription, &thr_msg, &thr_subscription_callback, ON_NEW_DATA));

	// ---- Main loop ----
	uint32_t measure;
	bool state = false;

	while (1) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &value);
		measure = value.val1 + value.val2;

		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

		if (debug == 2){
			std_msgs__msg__Int32 msg;
			msg.data = measure;
			rcl_publish(&tof_publisher, (const void*)&msg, NULL);
		}

		bool old_state = state;
		state = measure < threshold;

		if (state != old_state || debug >= 1){
			std_msgs__msg__Bool msg;
			msg.data = state;
			rcl_publish(&trigger_publisher, (const void*)&msg, NULL);
		}
		
		gpio_pin_set(led, PIN, (int)state);

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
