#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <drivers/led_strip.h>
#include <drivers/spi.h>
#include <stdio.h>
#include <sys/util.h>
#include <string.h>
#include <sys/printk.h>

#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/point32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); k_sleep(-1);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

void main(void)
{	
	// ---- Sensor configuration ----
	struct device *led = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	gpio_pin_configure(led, DT_ALIAS_LED0_GPIOS_PIN, GPIO_OUTPUT_ACTIVE | DT_ALIAS_LED0_GPIOS_FLAGS);

	struct device *tof_sensor = device_get_binding(DT_INST_0_ST_VL53L0X_LABEL);
	struct sensor_value tof_value;

	struct device *imu_sensor = device_get_binding(DT_INST_0_ST_LSM6DSL_LABEL);
	struct sensor_value imu_value;
	struct sensor_value accel_x, accel_y, accel_z;

	// set accel/gyro sampling frequency to 104 Hz
	imu_value.val1 = 104;
	imu_value.val2 = 0;

	sensor_attr_set(imu_sensor, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &imu_value);
	sensor_attr_set(imu_sensor, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &imu_value);

	// ---- micro-ROS configuration ----

	rcl_init_options_t options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()))

	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
	RCCHECK(rmw_uros_options_set_client_key(0xDEADBEEF, rmw_options))

	rcl_context_t context = rcl_get_zero_initialized_context();
	RCCHECK(rcl_init(0, NULL, &options, &context))

	// Creating node
	rcl_node_options_t node_ops = rcl_node_get_default_options();
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rcl_node_init(&node, "zephyr_sensors_node", "", &context, &node_ops))

	// Creating TOF publisher
	rcl_publisher_options_t tof_publisher_ops = rcl_publisher_get_default_options();
	// tof_publisher_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
	rcl_publisher_t tof_publisher = rcl_get_zero_initialized_publisher();
	RCCHECK(rcl_publisher_init(&tof_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/sensors/tof", &tof_publisher_ops))

	// Creating IMU publisher
	rcl_publisher_options_t imu_publisher_ops = rcl_publisher_get_default_options();
	// imu_publisher_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
	rcl_publisher_t imu_publisher = rcl_get_zero_initialized_publisher();
	RCCHECK(rcl_publisher_init(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32), "/sensors/imu", &imu_publisher_ops))

	// Creating LED subscriber
	rcl_subscription_options_t led_subscription_ops = rcl_subscription_get_default_options();
	led_subscription_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
	rcl_subscription_t led_subscription = rcl_get_zero_initialized_subscription();
	RCCHECK(rcl_subscription_init(&led_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/sensors/led", &led_subscription_ops))

	// Creating a trigger publisher
	rcl_publisher_options_t publisher_trigger_ops = rcl_publisher_get_default_options();
	publisher_trigger_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
	rcl_publisher_t publisher_trigger = rcl_get_zero_initialized_publisher();
	RCCHECK(rcl_publisher_init(&publisher_trigger, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/sensors/tof/trigger", &publisher_trigger_ops))

	// Creating a thresold subscriber
	rcl_subscription_options_t subscription_thr_ops = rcl_subscription_get_default_options();
	rcl_subscription_t subscription_thr = rcl_get_zero_initialized_subscription();
	RCCHECK(rcl_subscription_init(&subscription_thr, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/sensors/tof/threshold", &subscription_thr_ops))


	// Creating a wait set
	rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  	RCCHECK(rcl_wait_set_init(&wait_set, 2, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()))

	// ---- Main loop ----
	std_msgs__msg__Float32 tof_data;
	geometry_msgs__msg__Point32 imu_data;
	std_msgs__msg__Bool trigger_msg;
	gpio_pin_set(led, DT_ALIAS_LED0_GPIOS_PIN, 0);

	int32_t threshold = 300;

	do {

		// Publish TOF
		sensor_sample_fetch(tof_sensor);
		sensor_channel_get(tof_sensor, SENSOR_CHAN_DISTANCE, &tof_value);
		tof_data.data = sensor_value_to_double(&tof_value);
		printf("TOF sensor: %.3f m \t", tof_data.data);

		rcl_publish(&tof_publisher, (const void*)&tof_data, NULL);

		// Publish IMU
		sensor_sample_fetch_chan(imu_sensor, SENSOR_CHAN_ACCEL_XYZ);
		sensor_channel_get(imu_sensor, SENSOR_CHAN_ACCEL_X, &accel_x);
		sensor_channel_get(imu_sensor, SENSOR_CHAN_ACCEL_Y, &accel_y);
		sensor_channel_get(imu_sensor, SENSOR_CHAN_ACCEL_Z, &accel_z);
		imu_data.x = out_ev(&accel_x);
		imu_data.y = out_ev(&accel_y);
		imu_data.z = out_ev(&accel_z);
		printf("IMU: [%.2f, %.2f, %.2f] m/s^2\n",  imu_data.x, imu_data.y, imu_data.z);
	
		rcl_publish(&imu_publisher, (const void*)&imu_data, NULL);

		// Prepare the wait set and receive the subscription
		RCSOFTCHECK(rcl_wait_set_clear(&wait_set))
    
		size_t index_subscription_led;
		RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &led_subscription, &index_subscription_led))

		size_t index_subscription_thr;
		RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription_thr, &index_subscription_thr))

		RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(50)))
		
		// LED subscription
		if (wait_set.subscriptions[index_subscription_led]) {
			std_msgs__msg__Bool msg;
			rcl_take(wait_set.subscriptions[index_subscription_led], &msg, NULL, NULL);

			gpio_pin_set(led, DT_ALIAS_LED0_GPIOS_PIN, (int)(msg.data) ? 1 : 0);
		}

		// Threshold subscription
		if (wait_set.subscriptions[index_subscription_thr]) {
			std_msgs__msg__Int32 msg;
			rcl_take(wait_set.subscriptions[index_subscription_thr], &msg, NULL, NULL);
			threshold = msg.data;
		}

		// Trigger publication
		trigger_msg.data = tof_data.data*1000 < threshold;
		rcl_publish(&publisher_trigger, (const void*)&trigger_msg, NULL);

		k_sleep(10);
	} while (true);

	RCCHECK(rcl_node_fini(&node))
}
