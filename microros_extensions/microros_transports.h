// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _MICROROS_CLIENT_ZEPHYR_SERIAL_TRANSPORT_H_
#define _MICROROS_CLIENT_ZEPHYR_SERIAL_TRANSPORT_H_

#include <unistd.h>

#if defined(MICRO_ROS_TRANSPORT_SERIAL) || defined(MICRO_ROS_TRANSPORT_SERIALUSB)
#include <device.h>
#elif defined(MICRO_ROS_TRANSPORT_UDP)
#include <sys/types.h>
#include <sys/socket.h>
#include <poll.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
#if defined(MICRO_ROS_TRANSPORT_SERIAL) || defined(MICRO_ROS_TRANSPORT_SERIALUSB)
    size_t fd;
    struct device *uart_dev;
#elif defined(MICRO_ROS_TRANSPORT_UDP)
    struct pollfd poll_fd;
    char ip[16];
    char port[6];
#endif
} zephyr_transport_params_t;

#if defined(MICRO_ROS_TRANSPORT_SERIAL)
    #define MICRO_ROS_FRAMING_REQUIRED true
    // Set the UART used for communication. e.g. {.fd = 0} leads to UART_0 being used.
    static zephyr_transport_params_t default_params = {.fd = 0};
#elif defined(MICRO_ROS_TRANSPORT_UDP)
    #define MICRO_ROS_FRAMING_REQUIRED false
    static zephyr_transport_params_t default_params = {.ip = "192.168.1.100", .port = "8888"};
#elif defined(MICRO_ROS_TRANSPORT_SERIALUSB)
    #define MICRO_ROS_FRAMING_REQUIRED true
    static zephyr_transport_params_t default_params;
#endif

bool zephyr_transport_open(struct uxrCustomTransport * transport);
bool zephyr_transport_close(struct uxrCustomTransport * transport);
size_t zephyr_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t zephyr_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif //_MICROROS_CLIENT_ZEPHYR_SERIAL_TRANSPORT_H_
