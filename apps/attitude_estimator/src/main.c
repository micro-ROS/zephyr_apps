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
#include <posix/time.h>
#include <posix/sys/time.h>
#include <posix/unistd.h>
#include <inttypes.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <std_msgs/msg/empty.h>

#include <sensfusion6.h>

#define STR_CAPACITY 50
#define LED_PIN		 DT_GPIO_PIN(DT_ALIAS(led0), gpios)

#define SW0_NODE	DT_ALIAS(sw0)
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
#define SW0_GPIO_LABEL	DT_GPIO_LABEL(SW0_NODE, gpios)
#define SW0_GPIO_PIN	DT_GPIO_PIN(SW0_NODE, gpios)
#define SW0_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(SW0_NODE, gpios))
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#define SW0_GPIO_LABEL	""
#define SW0_GPIO_PIN	0
#define SW0_GPIO_FLAGS	0
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); while(1){};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t tf_publisher;
rcl_publisher_t euler_publisher;
rcl_publisher_t trigger_publisher;

uint32_t get_millis_from_timespec(struct timespec tv){
	return tv.tv_sec*1000 + tv.tv_nsec/1e6;
}

void main(void)
{	
	//  LED configuration
	const struct device *led = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
	gpio_pin_configure(led, LED_PIN, GPIO_OUTPUT_ACTIVE | 0);

	//  IMU configuration
	const struct device *imu_sensor = device_get_binding(DT_LABEL(DT_INST(0, st_lsm6dsl)));
	struct sensor_value imu_value;
	struct sensor_value accel_xyz[3], gyro_xyz[3], magn_xyz[3];

	imu_value.val1 = 104; // set accel/gyro sampling frequency to 104 Hz
	imu_value.val2 = 0;
	sensor_attr_set(imu_sensor, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &imu_value);
	sensor_attr_set(imu_sensor, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &imu_value);

	//  Magnetometer configuration
	const struct device *lis3mdl = device_get_binding(DT_LABEL(DT_INST(0, st_lis3mdl_magn)));

	// Button
	const struct device *button = device_get_binding(SW0_GPIO_LABEL);
	gpio_pin_configure(button, SW0_GPIO_PIN, SW0_GPIO_FLAGS);

	// ---- micro-ROS configuration ----
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "zephyr_attitude_node", "", &support));

	// Creating IMU publisher
	RCCHECK(rclc_publisher_init_default(&tf_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf"));
	RCCHECK(rclc_publisher_init_default(&euler_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/euler"));
	RCCHECK(rclc_publisher_init_default(&trigger_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "/trigger_moveit2"));

	// ---- Main loop ----
	gpio_pin_set(led, LED_PIN, 0);

	sensfusion6Init();

	geometry_msgs__msg__Vector3 eurler_angles;

	geometry_msgs__msg__TransformStamped tf_stamped;

	tf_stamped.header.frame_id.data = (char*)malloc(STR_CAPACITY*sizeof(char));
	char string1[] = "/panda_link0";
	memcpy(tf_stamped.header.frame_id.data, string1, strlen(string1) + 1);
	tf_stamped.header.frame_id.size = strlen(tf_stamped.header.frame_id.data);
	tf_stamped.header.frame_id.capacity = STR_CAPACITY;

	char string2[] = "/inertial_unit";
	tf_stamped.child_frame_id.data =  (char*)malloc(STR_CAPACITY*sizeof(char));
	memcpy(tf_stamped.child_frame_id.data, string2, strlen(string2) + 1);
	tf_stamped.child_frame_id.size = strlen(tf_stamped.child_frame_id.data);
	tf_stamped.child_frame_id.capacity = STR_CAPACITY;
	
	tf_stamped.transform.translation.x = 0.3;
	tf_stamped.transform.translation.y = 0;
	tf_stamped.transform.translation.z = 0.3;
	
	tf2_msgs__msg__TFMessage tf_message;
	tf_message.transforms.data = &tf_stamped;
	tf_message.transforms.size = 1;
	tf_message.transforms.capacity = 1;

	struct timespec tv = {0};
	struct timespec tv_end = {0};
	struct timespec tv_debouncing = {0};
	float sample_rate = 0.3;

	uint32_t loop_delay_us = 10000;

	while(1){
		clock_gettime(CLOCK_MONOTONIC, &tv);

		// Read IMU
		sensor_sample_fetch_chan(imu_sensor, SENSOR_CHAN_ACCEL_XYZ);
		sensor_channel_get(imu_sensor, SENSOR_CHAN_ACCEL_XYZ, accel_xyz);

		sensor_sample_fetch_chan(imu_sensor, SENSOR_CHAN_GYRO_XYZ);
		sensor_channel_get(imu_sensor, SENSOR_CHAN_GYRO_XYZ, gyro_xyz);

		// sensor_sample_fetch(lis3mdl);
		// sensor_channel_get(lis3mdl, SENSOR_CHAN_MAGN_XYZ, magn_xyz);

		sensfusion6UpdateQ(	(float) sensor_value_to_double(&gyro_xyz[0]),
						    (float) sensor_value_to_double(&gyro_xyz[1]),
						    (float) sensor_value_to_double(&gyro_xyz[2]),
							(float) sensor_value_to_double(&accel_xyz[0]),
							(float) sensor_value_to_double(&accel_xyz[1]),
							(float) sensor_value_to_double(&accel_xyz[2]),
							sample_rate);

		// estimator.update(((float)loop_delay)/1e6, 
		// 					sensor_value_to_double(&gyro_xyz[0]),
		// 					sensor_value_to_double(&gyro_xyz[1]),
		// 					sensor_value_to_double(&gyro_xyz[2]),
		// 					sensor_value_to_double(&accel_xyz[0]),
		// 					sensor_value_to_double(&accel_xyz[1]),
		// 					sensor_value_to_double(&accel_xyz[2]), 
		// 					-sensor_value_to_double(&magn_xyz[0]),
		// 					-sensor_value_to_double(&magn_xyz[1]),
		// 					sensor_value_to_double(&magn_xyz[2]));
		// estimator.getAttitude(q);

		// tf_stamped.transform.rotation.w = q[0];
		// tf_stamped.transform.rotation.x = q[1];
		// tf_stamped.transform.rotation.y = q[2];
		// tf_stamped.transform.rotation.z = q[3];

		sensfusion6GetQuaternion(&tf_stamped.transform.rotation.x,
								 &tf_stamped.transform.rotation.y,
								 &tf_stamped.transform.rotation.z, 
								 &tf_stamped.transform.rotation.w);

		sensfusion6GetEulerRPY(	&eurler_angles.x,
								&eurler_angles.y,
								&eurler_angles.z);

		eurler_angles.z = sample_rate;

		tf_stamped.header.stamp.nanosec = tv.tv_nsec;
		tf_stamped.header.stamp.sec = tv.tv_sec;

		rcl_publish(&tf_publisher, &tf_message, NULL);
		rcl_publish(&euler_publisher, &eurler_angles, NULL);

		if (gpio_pin_get(button, SW0_GPIO_PIN) && (get_millis_from_timespec(tv) - get_millis_from_timespec(tv_debouncing) > 300))
		{
			std_msgs__msg__Empty trigger;
			rcl_publish(&trigger_publisher, &trigger, NULL);
			tv_debouncing = tv;
		}

		clock_gettime(CLOCK_MONOTONIC, &tv_end);
		sample_rate = (float) (get_millis_from_timespec(tv_end) - get_millis_from_timespec(tv))/1000;
	}
}
