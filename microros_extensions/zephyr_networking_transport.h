#ifndef UXR_CLIENT_PROFILE_TRANSPORT_IP_UDP_EXTERNAL_H_
#define UXR_CLIENT_PROFILE_TRANSPORT_IP_UDP_EXTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <sys/types.h>
#include <sys/socket.h>
#include <poll.h>

typedef struct uxrUDPPlatform
{
    struct pollfd poll_fd;
} uxrUDPPlatform;

#ifdef __cplusplus
}
#endif

#endif // UXR_CLIENT_PROFILE_TRANSPORT_IP_UDP_EXTERNAL_H_