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

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define PIN	DT_GPIO_PIN(DT_ALIAS(led0), gpios)

#define STRIP_NUM_PIXELS 8
#define DELAY_TIME K_MSEC(50)
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

struct led_rgb pixels[STRIP_NUM_PIXELS] = {0};
struct led_rgb old_pixels[STRIP_NUM_PIXELS];

static const struct led_rgb colors[] = {
	RGB(0x0f, 0x00, 0x00), /* red */
	RGB(0x00, 0x0f, 0x00), /* green */
	RGB(0x00, 0x00, 0x0f), /* blue */
};

static struct device *led;
static struct device *strip;
static struct device *tof;

static uint32_t measure_ext = 2000;
static bool trigger = false;

rcl_subscription_t tof_subscription;
rcl_subscription_t trigger_subscription;

std_msgs__msg__Bool led_msg;
std_msgs__msg__Int32 thr_msg;

void trigger_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
	trigger = msg->data;
	gpio_pin_set(led, PIN, (int)(msg->data) ? 1 : 0);

	if(trigger){
		struct led_rgb c = RGB(0x0f, 0x00, 0x00);
		for(size_t j = 0 ; j < STRIP_NUM_PIXELS ; j++){
			memcpy(&pixels[j], &c, sizeof(struct led_rgb));
		}
	}
}

void tof_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	measure_ext = msg->data;

	if(!trigger){
		float val = ((float) measure_ext/1300.0);
		val = (val > 1) ? STRIP_NUM_PIXELS : STRIP_NUM_PIXELS*val;
		int nval = (int)val;

		struct led_rgb c = RGB(0x00, 0x0f, 0x00);
		memset(&pixels, 0x00, sizeof(pixels));
		for(size_t j = 0 ; j < STRIP_NUM_PIXELS-nval ; j++){
			memcpy(&pixels[j], &c, sizeof(struct led_rgb));
		}
	}
}

void main(void)
{	
	// ---- Devices configuration ----
	strip = device_get_binding(DT_LABEL(DT_ALIAS(led_strip)));

	struct led_rgb c = RGB(0x00, 0x00, 0x0f);
	for(size_t j = 0 ; j < STRIP_NUM_PIXELS ; j++){
		memcpy(&pixels[j], &c, sizeof(struct led_rgb));
	}

	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);


	led = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
	gpio_pin_configure(led, PIN, GPIO_OUTPUT_ACTIVE | 0);

	tof = device_get_binding(DT_LABEL(DT_INST(0, st_vl53l0x)));
	struct sensor_value value;

	// ---- micro-ROS configuration ----
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "zephyr_tof_leds", "", &support));

	// Creating tof subscriber
	RCCHECK(rclc_subscription_init_default(&trigger_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/sensors/tof/trigger"));

	// Creating trigger subscriber
	RCCHECK(rclc_subscription_init_default(&tof_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/sensors/tof"));

	// Creating a executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &trigger_subscription, &led_msg, &trigger_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &tof_subscription, &thr_msg, &tof_subscription_callback, ON_NEW_DATA));


	// ---- Main loop ----
	gpio_pin_set(led, PIN, 0);

	uint32_t measure;

	while (1) {
		s64_t time_stamp;
		s64_t milliseconds_spent;

		time_stamp = k_uptime_get();

		sensor_sample_fetch(tof);
		sensor_channel_get(tof, SENSOR_CHAN_DISTANCE, &value);
		measure = value.val1;
		
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

		led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

		milliseconds_spent = k_uptime_delta(&time_stamp);
		// k_sleep(150-milliseconds_spent);

		// printf("Measure is %d mm   Measure external is %d mm\n", measure, measure_ext);
		
		// std_msgs__msg__Int32 msg;
		// msg.data = measure;
		// rcl_publish(&publisher_measure, (const void*)&msg, NULL);		
		// k_sleep(20);
	}
}
