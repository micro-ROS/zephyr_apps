#include "olimex_e407_serial_transport.h"
#include "stm32f4xx_hal_dma.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include <usb/usb_device.h>

#define RING_BUF_SIZE 1024
u8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

static void interrupt_handler(struct device *dev)
{
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			u8_t buffer[64];
      rb_len++;
			// size_t len = MIN(ring_buf_space_get(&ringbuf), sizeof(buffer));

			// recv_len = uart_fifo_read(dev, buffer, len);

			// rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			// if (rb_len < recv_len) {
			// 	//LOG_ERR("Drop %u bytes", recv_len - rb_len);
			// }

			// //LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);

			// uart_irq_tx_enable(dev);
		}

		if (uart_irq_tx_ready(dev)) {
			u8_t buffer[64];
			int rb_len, send_len;
      rb_len++;
			// rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			// if (!rb_len) {
			// 	//LOG_DBG("Ring buffer empty, disable TX IRQ");
			// 	uart_irq_tx_disable(dev);
			// 	continue;
			// }

			// send_len = uart_fifo_fill(dev, buffer, rb_len);
			// if (send_len < rb_len) {
			// 	//LOG_ERR("Drop %d bytes", rb_len - send_len);
			// }

			//LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}

bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd, uint8_t remote_addr, uint8_t local_addr)
{  
  int ret;
  u32_t baudrate, dtr = 0U;
 
  platform->uart_dev = device_get_binding("CDC_ACM_0");
  if (!platform->uart_dev) {
    //LOG_ERR("CDC ACM device not found");
    return false;
  }

  ret = usb_enable(NULL);
  if (ret != 0) {
    //LOG_ERR("Failed to enable USB");
    return false;
  }

  ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);


  // while (true) {
	// 	uart_line_ctrl_get(platform->uart_dev, UART_LINE_CTRL_DTR, &dtr);
	// 	if (dtr) {
	// 		break;
	// 	} else {
	// 		/* Give CPU resources to low priority threads. */
	// 		k_sleep(K_MSEC(100));
	// 	}
	// }

  /* They are optional, we use them to test the interrupt endpoint */
  // ret = uart_line_ctrl_set(platform->uart_dev, UART_LINE_CTRL_DCD, 1);
  // if (ret) {
  //   //LOG_WRN("Failed to set DCD, ret code %d", ret);
  // }

  // ret = uart_line_ctrl_set(platform->uart_dev, UART_LINE_CTRL_DSR, 1);
  // if (ret) {
  //   //LOG_WRN("Failed to set DSR, ret code %d", ret);
  // }

  // uart_irq_callback_set(platform->uart_dev, interrupt_handler);

  // /* Enable rx interrupts */
  uart_irq_rx_enable(platform->uart_dev);
  
  return true;
}

bool uxr_close_serial_platform(struct uxrSerialPlatform* platform)
{   
//   HAL_UART_DMAStop(platform->uart);
  return true;
}

size_t uxr_write_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, uint8_t* errcode)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(platform->uart_dev, buf[i]);
	}

  return len;
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{ 
  s64_t time_stamp;
  s64_t milliseconds_spent;
  size_t write_index = 0;

  int ret;

  time_stamp = k_uptime_get();
  milliseconds_spent = k_uptime_delta(&time_stamp);

  size_t tries = 0;
  do{
    milliseconds_spent = k_uptime_delta(&time_stamp);

    ret = uart_poll_in(platform->uart_dev, &buf[write_index]);
    if (ret == 0){
      write_index++;
    }
    tries++;
  } while (write_index <= len && tries < 100);
 
  return write_index;
}