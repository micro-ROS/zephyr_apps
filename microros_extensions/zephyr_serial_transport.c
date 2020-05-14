#include <uxr/client/profile/transport/serial/serial_transport_external.h>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include <usb/usb_device.h>

#define RING_BUF_SIZE 2048

u8_t uart_in_buffer[RING_BUF_SIZE];
u8_t uart_out_buffer[RING_BUF_SIZE];

bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd, uint8_t remote_addr, uint8_t local_addr){  

    char uart_descriptor[8]; 
    sprintf(uart_descriptor,"UART_%d", fd);
    platform->uart_dev = device_get_binding(uart_descriptor);
    if (!platform->uart_dev) {
        printk("Serial device not found\n");
        return false;
    }

    return true;
}

bool uxr_close_serial_platform(struct uxrSerialPlatform* platform){   
      return true;
}

size_t uxr_write_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, uint8_t* errcode){ 

    for (size_t i = 0; i < len; i++)
    {
        uart_poll_out(platform->uart_dev, buf[i]);
    }
    
    return len;
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode){ 

    size_t index = 0;

    int ret = 0;
    while (index < len && ret == 0)
    {
        ret = uart_poll_in(platform->uart_dev, &buf[index]);
        if (ret == 0) {
            index++;
        }
    }
    
    return index;
 }