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

    int wrote = 1;
    while (out_head != out_tail && wrote != 0){
      wrote = uart_fifo_fill(dev, &uart_out_buffer[out_head],1);
      if (wrote){
        out_head++;
      }
    }    

		if (out_head == out_tail) {
			uart_irq_tx_disable(dev);
		}
	}

	if (uart_irq_rx_ready(dev)) {
    int read = 1;
    while (read != 0){
      read = uart_fifo_read(dev, &uart_in_buffer[in_tail], 1);

      if (read){
        in_tail = (in_tail + 1) % UART_BUFFER_SIZE;
      }    
    }
  }
}


bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd, uint8_t remote_addr, uint8_t local_addr)
{  
  int ret;
  u32_t baudrate, dtr = 0U;
 
  platform->uart_dev = device_get_binding(DT_INST_0_ST_STM32_UART_LABEL);
  if (!platform->uart_dev) {
    return false;
  }

  // ret = usb_enable(NULL);
  // if (ret != 0) {
  //   //LOG_ERR("Failed to enable USB");
  //   return false;
  // }

  uart_irq_rx_disable(platform->uart_dev);
  uart_irq_tx_disable(platform->uart_dev);

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
  out_head=0;
  out_tail= len;

  uart_irq_tx_enable(platform->uart_dev);

  while (out_head != out_tail){
    k_sleep(K_MSEC(5));
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

  // printk("Recv (%d): ", wrote);

  // for (size_t i = 0; i < wrote; i++)
  // {
  //   printk("0x%02X ", buf[i]);
  // }

  // printk("\n");
 
  return wrote;
 }