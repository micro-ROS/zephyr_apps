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
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

static int print_samples;
static int lsm6dsl_trig_cnt;

static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;
#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
static struct sensor_value magn_x_out, magn_y_out, magn_z_out;
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
static struct sensor_value press_out, temp_out;
#endif

#ifdef CONFIG_LSM6DSL_TRIGGER
static void lsm6dsl_trigger_handler(struct device *dev,
				    struct sensor_trigger *trig)
{
	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;
#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
	static struct sensor_value magn_x, magn_y, magn_z;
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
	static struct sensor_value press, temp;
#endif
	lsm6dsl_trig_cnt++;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

	/* lsm6dsl gyro */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
	/* lsm6dsl external magn */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_MAGN_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_X, &magn_x);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Y, &magn_y);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Z, &magn_z);
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
	/* lsm6dsl external press/temp */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_PRESS);
	sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
	sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
#endif

	if (print_samples) {
		print_samples = 0;

		accel_x_out = accel_x;
		accel_y_out = accel_y;
		accel_z_out = accel_z;

		gyro_x_out = gyro_x;
		gyro_y_out = gyro_y;
		gyro_z_out = gyro_z;

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
		magn_x_out = magn_x;
		magn_y_out = magn_y;
		magn_z_out = magn_z;
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
		press_out = press;
		temp_out = temp;
#endif
	}

}
#endif

void main(void)
{	
	// Sensors init
	struct device *tof_sensor = device_get_binding(DT_INST_0_ST_VL53L0X_LABEL);
	struct sensor_value tof_value;

	struct device *imu_sensor = device_get_binding(DT_INST_0_ST_LSM6DSL_LABEL);
	struct sensor_value imu_value;

	/* set accel/gyro sampling frequency to 104 Hz */
	imu_value.val1 = 104;
	imu_value.val2 = 0;

	sensor_attr_set(imu_sensor, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &imu_value);
	sensor_attr_set(imu_sensor, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &imu_value);

#ifdef CONFIG_LSM6DSL_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	sensor_trigger_set(imu_sensor, &trig, lsm6dsl_trigger_handler);
#endif

	// micro-ROS init

	rcl_init_options_t options = rcl_get_zero_initialized_init_options();

	RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()))

	// Optional RMW configuration 
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
	RCCHECK(rmw_uros_options_set_client_key(0xDEADBEEF, rmw_options))

	rcl_context_t context = rcl_get_zero_initialized_context();
	RCCHECK(rcl_init(0, NULL, &options, &context))

	rcl_node_options_t node_ops = rcl_node_get_default_options();

	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rcl_node_init(&node, "zephyr_int32_publisher", "", &context, &node_ops))

	rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
	rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
	RCCHECK(rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "zephyr_int32_publisher", &publisher_ops))

	std_msgs__msg__Int32 msg;
	msg.data = 0;
	
	rcl_ret_t rc;
	do {

		sensor_sample_fetch(tof_sensor);
		sensor_channel_get(tof_sensor, SENSOR_CHAN_DISTANCE, &tof_value);
		printf("TOF sensor: %.3fm\n", sensor_value_to_double(&tof_value));

			struct device *imu_sensor = device_get_binding(DT_INST_0_ST_LSM6DSL_LABEL);
	struct sensor_value imu_value;

		if(print_samples == 0){
			printk("accel x:%f ms/2 y:%f ms/2 z:%f ms/2 \n",
							  out_ev(&accel_x_out),
							  out_ev(&accel_y_out),
							  out_ev(&accel_z_out));
			
			print_samples = 1;
		}

		rc = rcl_publish(&publisher, (const void*)&msg, NULL);
		msg.data++;
		k_sleep(1000);
	} while (true);

	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))
}
