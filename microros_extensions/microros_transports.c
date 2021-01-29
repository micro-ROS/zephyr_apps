#include <uxr/client/transport.h>

#include <zephyr.h>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <microros_transports.h>

#if defined(MICRO_ROS_TRANSPORT_SERIAL)

#include <device.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>

#define RING_BUF_SIZE 2048

char uart_in_buffer[RING_BUF_SIZE];
char uart_out_buffer[RING_BUF_SIZE];

struct ring_buf out_ringbuf, in_ringbuf;

// --- micro-ROS Serial Transport for Zephyr ---

static void uart_fifo_callback(struct device *dev){ 
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            int recv_len;
            char buffer[64];
            size_t len = MIN(ring_buf_space_get(&in_ringbuf), sizeof(buffer));

            if (len > 0){
                recv_len = uart_fifo_read(dev, buffer, len);
                ring_buf_put(&in_ringbuf, buffer, recv_len);
            }

        }
    }
}

bool zephyr_transport_open(struct uxrCustomTransport * transport){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;
    
    char uart_descriptor[8]; 
    sprintf(uart_descriptor,"UART_%d", params->fd);
    params->uart_dev = device_get_binding(uart_descriptor);
    if (!params->uart_dev) {
        printk("Serial device not found\n");
        return false;
    }

    ring_buf_init(&in_ringbuf, sizeof(uart_in_buffer), uart_out_buffer);

    uart_irq_callback_set(params->uart_dev, uart_fifo_callback);

    /* Enable rx interrupts */
    uart_irq_rx_enable(params->uart_dev);

    return true;
}

bool zephyr_transport_close(struct uxrCustomTransport * transport){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;
    // TODO: close serial transport here
    return true;
}

size_t zephyr_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    for (size_t i = 0; i < len; i++)
    {
        uart_poll_out(params->uart_dev, buf[i]);
    }
    
    return len;
}

size_t zephyr_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    size_t read = 0;
    int spent_time = 0;

    while(ring_buf_is_empty(&in_ringbuf) && spent_time < timeout){
        usleep(1000);
        spent_time++;
    }

    uart_irq_rx_disable(params->uart_dev);
    read = ring_buf_get(&in_ringbuf, buf, len);
    uart_irq_rx_enable(params->uart_dev);

    return read;
}

#elif defined(MICRO_ROS_TRANSPORT_UDP)

#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <netdb.h>

bool zephyr_transport_open(struct uxrCustomTransport * transport){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    bool rv = false;
    params->poll_fd.fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (-1 != params->poll_fd.fd)
    {
        struct addrinfo hints;
        struct addrinfo* result;
        struct addrinfo* ptr;

        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;

        if (0 == getaddrinfo(params->ip, params->port, &hints, &result))
        {
            for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
            {
                if (0 == connect(params->poll_fd.fd, ptr->ai_addr, ptr->ai_addrlen))
                {
                    params->poll_fd.events = POLLIN;
                    rv = true;
                    break;
                }
            }
        }
        freeaddrinfo(result);
    }
    return rv;
}

bool zephyr_transport_close(struct uxrCustomTransport * transport){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    return (-1 == params->poll_fd.fd) ? true : (0 == close(params->poll_fd.fd));
}

size_t zephyr_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    size_t rv = 0;
    ssize_t bytes_sent = send(params->poll_fd.fd, (void*)buf, len, 0);
    if (-1 != bytes_sent)
    {
        rv = (size_t)bytes_sent;
        *err = 0;
    }
    else
    {
        *err = 1;
    }
    return rv;
}

size_t zephyr_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    size_t rv = 0;
    int poll_rv = poll(&params->poll_fd, 1, timeout);
    if (0 < poll_rv)
    {
        ssize_t bytes_received = recv(params->poll_fd.fd, (void*)buf, len, 0);
        if (-1 != bytes_received)
        {
            rv = (size_t)bytes_received;
            *err = 0;
        }
        else
        {
            *err = 1;
        }
    }
    else
    {
        *err = (0 == poll_rv) ? 0 : 1;
    }
    return rv;
}

#elif defined(MICRO_ROS_TRANSPORT_SERIALUSB)

#include <device.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include <usb/usb_device.h>

#define RING_BUF_SIZE 2048

char uart_in_buffer[RING_BUF_SIZE];
char uart_out_buffer[RING_BUF_SIZE];

struct ring_buf out_ringbuf, in_ringbuf;

static void uart_fifo_callback(struct device *dev){ 
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            int recv_len;
            char buffer[64];
            size_t len = MIN(ring_buf_space_get(&in_ringbuf), sizeof(buffer));

            if (len > 0){
                recv_len = uart_fifo_read(dev, buffer, len);
                ring_buf_put(&in_ringbuf, buffer, recv_len);
            }

        }

        if (uart_irq_tx_ready(dev)) {			
            char buffer[64];
            int rb_len;

            rb_len = ring_buf_get(&out_ringbuf, buffer, sizeof(buffer));

            if (rb_len == 0) {
                uart_irq_tx_disable(dev);
                continue;
            }

            uart_fifo_fill(dev, buffer, rb_len);
        }
    }
}

bool zephyr_transport_open(struct uxrCustomTransport * transport){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    int ret;
    uint32_t baudrate, dtr = 0U;

    params->uart_dev = device_get_binding("CDC_ACM_0");
    if (!params->uart_dev) {
        printk("CDC ACM device not found\n");
        return false;
    }

    ret = usb_enable(NULL);
    if (ret != 0) {
        printk("Failed to enable USB\n");
        return false;
    }

    ring_buf_init(&out_ringbuf, sizeof(uart_out_buffer), uart_out_buffer);
    ring_buf_init(&in_ringbuf, sizeof(uart_in_buffer), uart_out_buffer);

    printk("Waiting for agent connection\n");

    while (true) {
        uart_line_ctrl_get(params->uart_dev, UART_LINE_CTRL_DTR, &dtr);
        if (dtr) {
            break;
        } else {
            /* Give CPU resources to low priority threads. */
            k_sleep(K_MSEC(100));
        }
    }

    printk("Serial port connected!\n");

    /* They are optional, we use them to test the interrupt endpoint */
    ret = uart_line_ctrl_set(params->uart_dev, UART_LINE_CTRL_DCD, 1);
    if (ret) {
        printk("Failed to set DCD, ret code %d\n", ret);
    }

    ret = uart_line_ctrl_set(params->uart_dev, UART_LINE_CTRL_DSR, 1);
    if (ret) {
        printk("Failed to set DSR, ret code %d\n", ret);
    }

    /* Wait 1 sec for the host to do all settings */
    k_busy_wait(1000*1000);

    ret = uart_line_ctrl_get(params->uart_dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
    if (ret) {
        printk("Failed to get baudrate, ret code %d\n", ret);
    }

    uart_irq_callback_set(params->uart_dev, uart_fifo_callback);

    /* Enable rx interrupts */
    uart_irq_rx_enable(params->uart_dev);

    return true;
}

bool zephyr_transport_close(struct uxrCustomTransport * transport){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    return true;
}

size_t zephyr_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    size_t wrote;
    
    wrote = ring_buf_put(&out_ringbuf, buf, len);
    
    uart_irq_tx_enable(params->uart_dev);

    while (!ring_buf_is_empty(&out_ringbuf)){
        k_sleep(K_MSEC(5));
    }
    
    return wrote;
}

size_t zephyr_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    size_t read = 0;
    int spent_time = 0;

    while(ring_buf_is_empty(&in_ringbuf) && spent_time < timeout){
        k_sleep(K_MSEC(1));
        spent_time++;
    }

    uart_irq_rx_disable(params->uart_dev);
    read = ring_buf_get(&in_ringbuf, buf, len);
    uart_irq_rx_enable(params->uart_dev);

    return read;
}

#endif