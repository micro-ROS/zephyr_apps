#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <sys/util.h>
#include <string.h>
#include <sys/printk.h>

#include <net/net_if.h>
#include <net/wifi_mgmt.h>
#include <net/net_event.h>

#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static struct net_mgmt_event_callback wifi_shell_mgmt_cb;
static bool connected = 0;

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				    u32_t mgmt_event, struct net_if *iface)
{	
	if(NET_EVENT_IPV4_ADDR_ADD == mgmt_event){
			printf("DHCP Connected\n");
			connected = 1;
	}
}

void main(void)
{	
	// Wifi Configuration
	net_mgmt_init_event_callback(&wifi_shell_mgmt_cb,
					wifi_mgmt_event_handler,
					NET_EVENT_IPV4_ADDR_ADD);

	net_mgmt_add_event_callback(&wifi_shell_mgmt_cb);

	struct net_if *iface = net_if_get_default();
	static struct wifi_connect_req_params cnx_params;


	cnx_params.ssid = "WIFI_SSID_HERE";
	cnx_params.ssid_length = strlen(cnx_params.ssid);
	cnx_params.channel = 0;
	cnx_params.psk = "WIFI_PSK_HERE";
	cnx_params.psk_length = strlen(cnx_params.psk);
	cnx_params.security = WIFI_SECURITY_TYPE_PSK;

	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &cnx_params, sizeof(struct wifi_connect_req_params))) {
		printf("Connection request failed\n");
	} else {
		printf("Connection requested\n");
	}

	while (!connected)
	{
		printf("Waiting for connection\n");
		usleep(10000);
	}
	printf("Connection OK\n");
	
		sleep(1);
	// micro-ROS
	rcl_init_options_t options = rcl_get_zero_initialized_init_options();

	RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()))

	// Optional RMW configuration 
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
	RCCHECK(rmw_uros_options_set_client_key(0xDEADBEEF, rmw_options))

	rcl_context_t context = rcl_get_zero_initialized_context();
	RCCHECK(rcl_init(0, NULL, &options, &context))

	rcl_node_options_t node_ops = rcl_node_get_default_options();

	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rcl_node_init(&node, "zephyr_int32_wifi_publisher", "", &context, &node_ops))

	rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
	publisher_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
	rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
	RCCHECK(rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "zephyr_int32_publisher", &publisher_ops))

	std_msgs__msg__Int32 msg;
	msg.data = 0;
	
	rcl_ret_t rc;
	do {
		rc = rcl_publish(&publisher, (const void*)&msg, NULL);
		msg.data++;
		// usleep(10000);
	} while (true);

	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))
}
