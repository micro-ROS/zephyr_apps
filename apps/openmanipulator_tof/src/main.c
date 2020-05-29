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
#include <std_msgs/msg/float32.h>


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#ifndef DT_ALIAS_LED0_GPIOS_FLAGS
#define DT_ALIAS_LED0_GPIOS_FLAGS 0
#endif

static struct device *led;
static struct device *dev;

rcl_publisher_t tof_publisher;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	UNUSED(last_call_time);
	if (timer != NULL) {
		struct sensor_value value;
		std_msgs__msg__Float32 msg;

		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &value);
		msg.data= ((float)value.val1 + (float)value.val2)/1000.0;
		
		rcl_publish(&tof_publisher, (const void*)&msg, NULL);
	}
}

void main(void)
{	
	// ---- Sensor configuration ----
	led = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	gpio_pin_configure(led, DT_ALIAS_LED0_GPIOS_PIN, GPIO_OUTPUT_ACTIVE | DT_ALIAS_LED0_GPIOS_FLAGS);

	*dev = device_get_binding(DT_INST_0_ST_VL53L1X_LABEL);

	if (dev == NULL) {
		printk("Could not get VL53L0X device\n");
		return;
	}

	// ---- micro-ROS configuration ----
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, argc, argv, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "zephyr_tof_node", "", &support));

	// Creating TOF publisher
	RCCHECK(rclc_publisher_init_default(&tof_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/sensors/tof"));
	// tof_publisher_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

	// Creating a timer
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

	// Creating a executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

	unsigned int rcl_wait_timeout = 10;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	// ---- Main loop ----
	while(1){
    	rclc_executor_spin_some(&executor, 10);
		k_sleep(10);
	}
}
