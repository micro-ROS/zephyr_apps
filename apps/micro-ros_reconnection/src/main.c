#include <zephyr.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include <microros_transports.h>
#include <std_msgs/msg/int32.h>

#include <logging/log.h>

#define MICRO_ROS_APP_STACK 16000
#define MICRO_ROS_APP_TASK_PRIO 5

LOG_MODULE_REGISTER(uros_transport, LOG_LEVEL_DBG);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Publishers
rcl_publisher_t publisher;

// Node support
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Timers
rcl_timer_t pub_timer;

// Messages
std_msgs__msg__Int32 msg;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void uros_transport_task();

K_THREAD_DEFINE(uros_transport, MICRO_ROS_APP_STACK, uros_transport_task, NULL, NULL, NULL, MICRO_ROS_APP_TASK_PRIO, 0,
                0);


/* Timer callback for publisher */
void pub_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

/* Create allocator, support, pub, sub, executor for node if agent connection is successful */
bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // Init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Node
  RCCHECK(rclc_node_init_default(&node, "uros_transport", "", &support));

  // Publisher
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                      "/counter"));

  // Timer
  const unsigned int timer_timeout_ms = 1000;
  RCCHECK(rclc_timer_init_default(&pub_timer, &support, RCL_MS_TO_NS(timer_timeout_ms), pub_timer_callback));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &pub_timer));

  return true;
}

/* Destroy allocator, support, pub, sub, executor for node if agent connection is
 * lost or unsuccessful */
bool destroy_entities()
{
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_timer_fini(&pub_timer));
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));

  return true;
}

/* Main transport task for transmitting and receiving data. Functions include
 * monitoring the agent connection, pinging and reconnecting. */
void uros_transport_task()
{

  while (1)
  {
    switch (state)
    {
      case WAITING_AGENT:
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
        LOG_INF("Waiting for uROS agent.\r\n");
        k_msleep(1000);
        break;

      case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
          destroy_entities();
          LOG_INF("Destroyed entities.");
        }
        else
        {
          /* Agent has connected and entities created. */
          LOG_INF("Connected to uROS agent and created entities.");
        }
        break;

      case AGENT_CONNECTED:
        state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;

        if (state == AGENT_CONNECTED)
        {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		  k_msleep(1000);
        }

        break;
      case AGENT_DISCONNECTED:
        LOG_INF("Disconnected from agent. Switching to waiting for uROS agent\r\n");
        destroy_entities();
        state = WAITING_AGENT;

        break;
      default:
        break;
    }
  }
}

void main()
{
  /* Setup custom transports for microros. */
  rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED, (void*)&default_params, zephyr_transport_open,
                                zephyr_transport_close, zephyr_transport_write, zephyr_transport_read);

  state = WAITING_AGENT;
}
