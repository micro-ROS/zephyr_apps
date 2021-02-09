#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <drivers/led_strip.h>
#include <drivers/spi.h>
#include <stdio.h>
#include <sys/util.h>
#include <string.h>
#include <sys/printk.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_uros/options.h>
#include <microros_transports.h>

#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/point32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#define PIN	DT_GPIO_PIN(DT_ALIAS(led0), gpios)

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

struct device *led;

rcl_publisher_t tof_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t trigger_publisher;
rcl_subscription_t led_subscription;
rcl_subscription_t thr_subscription;

int32_t threshold = 300;

std_msgs__msg__Bool led_msg;
std_msgs__msg__Int32 thr_msg;

std_msgs__msg__Float32 tof_data;
geometry_msgs__msg__Point32 imu_data;
std_msgs__msg__Bool trigger_msg;

static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&tof_publisher, &tof_data, NULL));
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_data, NULL));

	trigger_msg.data = tof_data.data*1000 < threshold;
    RCSOFTCHECK(rcl_publish(&trigger_publisher, &trigger_msg, NULL));
  }
}

void led_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
	gpio_pin_set(led, PIN, (int)(msg->data) ? 1 : 0);
}

void thr_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	threshold = msg->data;
}

void main(void)
{	
	// Set custom transports
	rmw_uros_set_custom_transport(
		MICRO_ROS_FRAMING_REQUIRED,
		(void *) &default_params,
		zephyr_transport_open,
		zephyr_transport_close,
		zephyr_transport_write,
		zephyr_transport_read
	);

	// ---- Sensor configuration ----
	led = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
	gpio_pin_configure(led, PIN, GPIO_OUTPUT_ACTIVE | 0);

	struct device *tof_sensor = device_get_binding(DT_LABEL(DT_INST(0, st_vl53l0x)));
	struct sensor_value tof_value;

	struct device *imu_sensor = device_get_binding(DT_LABEL(DT_INST(0, st_lsm6dsl)));
	struct sensor_value imu_value;
	struct sensor_value accel_x, accel_y, accel_z;

	// set accel/gyro sampling frequency to 104 Hz
	imu_value.val1 = 104;
	imu_value.val2 = 0;

	sensor_attr_set(imu_sensor, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &imu_value);
	sensor_attr_set(imu_sensor, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &imu_value);

	// ---- micro-ROS configuration ----
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "zephyr_sensors_node", "", &support));


	// Creating TOF publisher
	RCCHECK(rclc_publisher_init_default(&tof_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/sensors/tof"));
	
	// Creating IMU publisher
	RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32), "/sensors/imu"));

	// Creating LED subscriber
	RCCHECK(rclc_subscription_init_default(&led_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/sensors/led"));
	// led_subscription_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

	// Creating a trigger publisher
	RCCHECK(rclc_publisher_init_default(&trigger_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/sensors/tof/trigger"));
	// trigger_publisher_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

	// Creating a thresold subscriber
	RCCHECK(rclc_subscription_init_default(&thr_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/sensors/tof/threshold"));

	// Creating a timer
	rcl_timer_t timer;
	const unsigned int timer_timeout = 50;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

	// Creating a executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &led_subscription, &led_msg, &led_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &thr_subscription, &thr_msg, &thr_subscription_callback, ON_NEW_DATA));

	// ---- Main loop ----
	gpio_pin_set(led, PIN, 0);

	while(1){
		// Read ToF
		sensor_sample_fetch(tof_sensor);
		sensor_channel_get(tof_sensor, SENSOR_CHAN_DISTANCE, &tof_value);
		tof_data.data = sensor_value_to_double(&tof_value);

		// Read IMU
		sensor_sample_fetch_chan(imu_sensor, SENSOR_CHAN_ACCEL_XYZ);
		sensor_channel_get(imu_sensor, SENSOR_CHAN_ACCEL_X, &accel_x);
		sensor_channel_get(imu_sensor, SENSOR_CHAN_ACCEL_Y, &accel_y);
		sensor_channel_get(imu_sensor, SENSOR_CHAN_ACCEL_Z, &accel_z);
		imu_data.x = out_ev(&accel_x);
		imu_data.y = out_ev(&accel_y);
		imu_data.z = out_ev(&accel_z);

    	rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	  	usleep(100000);
	}

	RCCHECK(rcl_node_fini(&node))
}
