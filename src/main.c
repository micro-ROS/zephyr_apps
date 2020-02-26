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

static bool microros_init = false;
static uint32_t measure_ext = 2000;
static bool trigger = false;

K_SEM_DEFINE(updating_sem, 1, 1);

void led_update(void){
	strip = device_get_binding(DT_ALIAS_LED_STRIP_LABEL);
	while(true){
		k_sem_take(&updating_sem, K_FOREVER);
		// if(memcmp(pixels,old_pixels,sizeof(pixels)) != 0){
			// memcpy(old_pixels, pixels, sizeof(pixels));
			led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		// }
		k_sem_give(&updating_sem);
		k_sleep(K_MSEC(50));
	}
}
 

void led_startup(void){
	struct led_rgb c = RGB(0x00, 0x00, 0x0f);

	size_t i = 0;
	while(!microros_init){
		k_sem_take(&updating_sem, K_FOREVER);
		memset(&pixels, 0x00, sizeof(pixels));
		memcpy(&pixels[i], &c, sizeof(struct led_rgb));
		i = (i+1)%STRIP_NUM_PIXELS;
		k_sem_give(&updating_sem);
		led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		k_sleep(K_MSEC(100));
	}

	return;
}


// K_THREAD_DEFINE(my_tid, 500,
//                 led_startup, NULL, NULL, NULL,
//                 4, 0, K_NO_WAIT);

// K_THREAD_DEFINE(my_tid2, 500,
//                 led_update, NULL, NULL, NULL,
//                 -16, 0, K_NO_WAIT);

void main(void)
{	
	strip = device_get_binding(DT_ALIAS_LED_STRIP_LABEL);

	struct led_rgb c = RGB(0x00, 0x00, 0x0f);
	for(size_t j = 0 ; j < STRIP_NUM_PIXELS ; j++){
		memcpy(&pixels[j], &c, sizeof(struct led_rgb));
	}

	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);


	led = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	gpio_pin_configure(led, DT_ALIAS_LED0_GPIOS_PIN, GPIO_OUTPUT_ACTIVE | DT_ALIAS_LED0_GPIOS_FLAGS);

	tof = device_get_binding(DT_INST_0_ST_VL53L0X_LABEL);
	struct sensor_value value;

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

	rcl_subscription_options_t subscription_trigger_ops = rcl_subscription_get_default_options();
	rcl_subscription_t subscription_trigger = rcl_get_zero_initialized_subscription();
	RCCHECK(rcl_subscription_init(&subscription_trigger, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/tof/trigger", &subscription_trigger_ops))

	rcl_subscription_options_t subscription_measure_ops = rcl_subscription_get_default_options();
	rcl_subscription_t subscription_measure = rcl_get_zero_initialized_subscription();
	RCCHECK(rcl_subscription_init(&subscription_measure, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/tof/measure", &subscription_measure_ops))


	rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  	RCCHECK(rcl_wait_set_init(&wait_set, 2, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()))

	microros_init = true;

	gpio_pin_set(led, DT_ALIAS_LED0_GPIOS_PIN, 0);

	uint32_t measure;

	while (1) {
		s64_t time_stamp;
		s64_t milliseconds_spent;

		time_stamp = k_uptime_get();

		sensor_sample_fetch(tof);
		sensor_channel_get(tof, SENSOR_CHAN_DISTANCE, &value);
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

			if(trigger){
				struct led_rgb c = RGB(0x0f, 0x00, 0x00);
				for(size_t j = 0 ; j < STRIP_NUM_PIXELS ; j++){
					memcpy(&pixels[j], &c, sizeof(struct led_rgb));
				}
			}
		}

		if (wait_set.subscriptions[index_measure_ext]) {
			std_msgs__msg__Int32 msg;
			rcl_take(wait_set.subscriptions[index_measure_ext], &msg, NULL, NULL);
			measure_ext = msg.data;

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
