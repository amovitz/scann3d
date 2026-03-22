/*
 * udp_out.c
 *
 * Opens a persistent UDP socket and blasts frames to the configured host:port.
 * WiFi association is handled separately (wifi_mgr); this module blocks in
 * udp_out_init() until a usable network interface exists.
 */

#include "udp_out.h"
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(udp_out, LOG_LEVEL_INF);

static int _sock = -1;
static struct sockaddr_in _dest;

int udp_out_init(void)
{
    /* Wait for network to come up (up to 30 s) */
    int retry = 300;
    while (retry-- > 0) {
        _sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (_sock >= 0) { break; }
        k_msleep(100);
    }
    if (_sock < 0) {
        LOG_ERR("Failed to create UDP socket");
        return -ENODEV;
    }

    memset(&_dest, 0, sizeof(_dest));
    _dest.sin_family = AF_INET;
    _dest.sin_port   = htons(CONFIG_SCANNER_UDP_PORT);
    if (zsock_inet_pton(AF_INET, CONFIG_SCANNER_UDP_HOST,
                        &_dest.sin_addr) != 1) {
        LOG_ERR("Invalid UDP host: %s", CONFIG_SCANNER_UDP_HOST);
        zsock_close(_sock);
        _sock = -1;
        return -EINVAL;
    }

    LOG_INF("UDP → %s:%d", CONFIG_SCANNER_UDP_HOST, CONFIG_SCANNER_UDP_PORT);
    return 0;
}

void udp_out_send(const uint8_t *buf, uint16_t len)
{
    if (_sock < 0) { return; }
    zsock_sendto(_sock, buf, len, 0,
                 (struct sockaddr *)&_dest, sizeof(_dest));
}
