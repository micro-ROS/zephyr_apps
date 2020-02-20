// #include <errno.h>
// #include <string.h>

// #define LOG_LEVEL 4
// #include <logging/log.h>
// LOG_MODULE_REGISTER(main);








// void main(void)
// {
// 	size_t cursor = 0, color = 0;
// 	int rc;

// 	LOG_INF("HOOOLA");

// 	strip = device_get_binding(DT_ALIAS_LED_STRIP_LABEL);
// 	if (strip) {
// 		LOG_INF("Found LED strip device %s", DT_ALIAS_LED_STRIP_LABEL);
// 	} else {
// 		LOG_ERR("LED strip device %s not found", DT_ALIAS_LED_STRIP_LABEL);
// 		return;
// 	}

// 	LOG_INF("Displaying pattern on strip");
// 	while (1) {
// 		memset(&pixels, 0x00, sizeof(pixels));
// 		memcpy(&pixels[cursor], &colors[color], sizeof(struct led_rgb));
// 		rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

// 		if (rc) {
// 			LOG_ERR("couldn't update strip: %d", rc);
// 		}

// 		cursor++;
// 		if (cursor >= STRIP_NUM_PIXELS) {
// 			cursor = 0;
// 			color++;
// 			if (color == ARRAY_SIZE(colors)) {
// 				color = 0;
// 			}
// 		}

// 		k_sleep(DELAY_TIME);
// 	}
// }


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

#ifndef DT_ALIAS_LED0_GPIOS_FLAGS
#define DT_ALIAS_LED0_GPIOS_FLAGS 0
#endif

#define STRIP_NUM_PIXELS	DT_ALIAS_LED_STRIP_CHAIN_LENGTH
#define DELAY_TIME K_MSEC(50)
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }
struct led_rgb pixels[STRIP_NUM_PIXELS];

static const struct led_rgb colors[] = {
	RGB(0x0f, 0x00, 0x00), /* red */
	RGB(0x00, 0x0f, 0x00), /* green */
	RGB(0x00, 0x00, 0x0f), /* blue */
};

static struct device *led;
static struct device *strip;

void main(void)
{	
	// struct device *strip;
	// size_t cursor = 0, color = 0;
	// int rc;

	// strip = device_get_binding(DT_ALIAS_LED_STRIP_LABEL);
	// if (strip) {
	// 	printk("Found LED strip device %s", DT_ALIAS_LED_STRIP_LABEL);
	// } else {
	// 	printk("LED strip device %s not found", DT_ALIAS_LED_STRIP_LABEL);
	// 	return;
	// }

	// while (1) {
	// 	memset(&pixels, 0x00, sizeof(pixels));

	// 	for (size_t i = 0 ; i < STRIP_NUM_PIXELS ; i++){
	// 		memcpy(&pixels[i], &colors[color], sizeof(struct led_rgb));
	// 	}	strip = device_get_binding(DT_ALIAS_LED_STRIP_LABEL);
	// if (strip) {
	// 	printk("Found LED strip device %s", DT_ALIAS_LED_STRIP_LABEL);
	// } else {
	// 	printk("LED strip device %s not found", DT_ALIAS_LED_STRIP_LABEL);
	// 	return;
	// }

	// while (1) {
	// 	memset(&pixels, 0x00, sizeof(pixels));

	// 	for (size_t i = 0 ; i < STRIP_NUM_PIXELS ; i++){
	// 		memcpy(&pixels[i], &colors[color], sizeof(struct led_rgb));
	// 	}
	// 	color = (color + 1) % 3;

	// 	rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

	// 	k_sleep(150);
	// }
	// 	color = (color + 1) % 3;

	// 	rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

	// 	k_sleep(150);
	// }

	led = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	gpio_pin_configure(led, DT_ALIAS_LED0_GPIOS_PIN, GPIO_OUTPUT_ACTIVE | DT_ALIAS_LED0_GPIOS_FLAGS);

	strip = device_get_binding(DT_ALIAS_LED_STRIP_LABEL);

	for(size_t i = 0 ; i < 3 ; i++){
		memset(&pixels, 0x00, sizeof(pixels));
		for(size_t j = 0 ; j < STRIP_NUM_PIXELS ; j++){
			memcpy(&pixels[j], &colors[i], sizeof(struct led_rgb));
		}
		led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		k_sleep(K_MSEC(600));
	}
	memset(&pixels, 0x00, sizeof(pixels));
	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

	rcl_init_options_t options = rcl_get_zero_initialized_init_options();

	RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()))

	// Optional RMW configuration 
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
	RCCHECK(rmw_uros_options_set_client_key(0xDEADBEEF, rmw_options))

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

	rcl_subscription_options_t subscription_measure_ops = rcl_subscription_get_default_options();
	rcl_subscription_t subscription_measure = rcl_get_zero_initialized_subscription();
	RCCHECK(rcl_subscription_init(&subscription_measure, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/tof/measure", &subscription_measure_ops))


	rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  	RCCHECK(rcl_wait_set_init(&wait_set, 2, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()))

	struct device *dev = device_get_binding(DT_INST_0_ST_VL53L0X_LABEL);
	struct sensor_value value;

	if (dev == NULL) {
		printk("Could not get VL53L0X device\n");
		return;
	}

	uint32_t measure;
	uint32_t measure_ext = 2000;
	bool trigger = false;

	gpio_pin_set(led, DT_ALIAS_LED0_GPIOS_PIN, 0);
	
	uint32_t k = 0;



	while (1) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &value);
		measure = value.val1;
		
		RCSOFTCHECK(rcl_wait_set_clear(&wait_set))
    
		size_t index_subscription_trigger;
		RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription_trigger, &index_subscription_trigger))

		size_t index_measure_ext;
		RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription_measure, &index_measure_ext))

		RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(10)))

		if (wait_set.subscriptions[index_subscription_trigger]) {
			std_msgs__msg__Bool msg;
			rcl_take(wait_set.subscriptions[index_subscription_trigger], &msg, NULL, NULL);
			trigger = msg.data;
			gpio_pin_set(led, DT_ALIAS_LED0_GPIOS_PIN, (int)(msg.data) ? 1 : 0);
		}

		if (wait_set.subscriptions[index_measure_ext]) {
			std_msgs__msg__Int32 msg;
			rcl_take(wait_set.subscriptions[index_measure_ext], &msg, NULL, NULL);
			measure_ext = msg.data;
		}

		printf("Measure is %d mm   Measure external is %d mm\n", measure, measure_ext);


		if(trigger){
			memset(&pixels, 0x00, sizeof(pixels));

			struct led_rgb c = RGB(0x0f, 0x00, 0x00);
			for(size_t j = 0 ; j < STRIP_NUM_PIXELS ; j++){
				memcpy(&pixels[j], &c, sizeof(struct led_rgb));
			}
			led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		}else{
			float val = ((float) measure_ext/1300.0);
			val = (val > 1) ? STRIP_NUM_PIXELS : STRIP_NUM_PIXELS*val;
			int nval = (int)val;
			memset(&pixels, 0x00, sizeof(pixels));

			struct led_rgb c = RGB(0x00, 0x0f, 0x00);
			memset(&pixels, 0x00, sizeof(pixels));
			for(size_t j = 0 ; j < STRIP_NUM_PIXELS-nval ; j++){
				memcpy(&pixels[j], &colors[1], sizeof(struct led_rgb));
			}

			led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		}
		
		// std_msgs__msg__Int32 msg;
		// msg.data = measure;
		// rcl_publish(&publisher_measure, (const void*)&msg, NULL);		
	}
}
