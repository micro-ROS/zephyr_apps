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

#define UART_BUFFER_SIZE 2048

static uint8_t uart_in_buffer[UART_BUFFER_SIZE];
static size_t in_head = 0, in_tail = 0;

static uint8_t uart_out_buffer[UART_BUFFER_SIZE];
static size_t out_head = 0, out_tail = 0;

static void uart_fifo_callback(struct device *dev)
{
	if (!uart_irq_update(dev)) {
		return;
	}

	if (uart_irq_tx_ready(dev)) {
    uart_fifo_fill(dev, &uart_out_buffer[out_head++], 1);
		if (out_head == out_tail) {
			uart_irq_tx_disable(dev);
		}
	}

	if (uart_irq_rx_ready(dev)) {
    uart_fifo_read(dev, &uart_in_buffer[in_tail], 1);
    in_tail = (in_tail + 1) % UART_BUFFER_SIZE;
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

	uart_irq_callback_set(platform->uart_dev, uart_fifo_callback);
  uart_irq_rx_enable(platform->uart_dev);
  
  return true;
}

bool uxr_close_serial_platform(struct uxrSerialPlatform* platform)
{   
  return true;
}

size_t uxr_write_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, uint8_t* errcode)
{

  memcpy(uart_out_buffer, buf, len);
  out_tail = len - 1;
  out_head = 0;

  uart_irq_tx_enable(platform->uart_dev);

  while (out_head != out_tail){
    k_sleep(K_MSEC(100));
  }

  return len;
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{ 
  k_sleep(K_MSEC(timeout));

  size_t wrote = 0;
  while ((in_head != in_tail) && (wrote < len)){
    buf[wrote] = uart_in_buffer[in_head];
    in_head = (in_head + 1) % UART_BUFFER_SIZE;
    wrote++;
  }
 
  return wrote;
 }