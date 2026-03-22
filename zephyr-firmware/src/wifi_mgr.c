/*
 * wifi_mgr.c
 *
 * Uses Zephyr's WiFi management API (introduced in Zephyr 3.4) to connect
 * the ESP32-C3 to the configured AP.  Blocks until the IP stack is up or
 * a 30-second timeout expires.
 */

#include "wifi_mgr.h"
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(wifi_mgr, LOG_LEVEL_INF);

/* ── Event semaphore ─────────────────────────────────────────────────────── */
static K_SEM_DEFINE(wifi_connected_sem, 0, 1);
static K_SEM_DEFINE(ip_obtained_sem,    0, 1);

/* ── Net-mgmt callback ───────────────────────────────────────────────────── */
static struct net_mgmt_event_callback _wifi_cb;
static struct net_mgmt_event_callback _ipv4_cb;

static void wifi_event_handler(struct net_mgmt_event_callback *cb,
                                uint64_t mgmt_event,
                                struct net_if *iface)
{
    ARG_UNUSED(cb);
    ARG_UNUSED(iface);

    switch (mgmt_event) {
    case NET_EVENT_WIFI_CONNECT_RESULT:
        LOG_INF("WiFi associated");
        k_sem_give(&wifi_connected_sem);
        break;
    case NET_EVENT_WIFI_DISCONNECT_RESULT:
        LOG_WRN("WiFi disconnected");
        break;
    default:
        break;
    }
}

static void ipv4_event_handler(struct net_mgmt_event_callback *cb,
                                uint64_t mgmt_event,
                                struct net_if *iface)
{
    ARG_UNUSED(cb);

    if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
        /* Log obtained address */
        struct net_if_ipv4 *ipv4 = iface->config.ip.ipv4;
        if (ipv4) {
            char addr_str[NET_IPV4_ADDR_LEN];
            net_addr_ntop(AF_INET,
                          &ipv4->unicast[0].ipv4.address.in_addr,
                          addr_str, sizeof(addr_str));
            LOG_INF("IP address: %s", addr_str);
        }
        k_sem_give(&ip_obtained_sem);
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */
int wifi_mgr_connect(void)
{
    struct net_if *iface = net_if_get_default();
    if (!iface) {
        LOG_ERR("No network interface");
        return -ENODEV;
    }

    /* Register event callbacks */
    net_mgmt_init_event_callback(&_wifi_cb, wifi_event_handler,
                                  NET_EVENT_WIFI_CONNECT_RESULT |
                                  NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&_wifi_cb);

    net_mgmt_init_event_callback(&_ipv4_cb, ipv4_event_handler,
                                  NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&_ipv4_cb);

    /* Issue connect request */
    struct wifi_connect_req_params params = {
        .ssid        = (const uint8_t *)CONFIG_SCANNER_WIFI_SSID,
        .ssid_length = strlen(CONFIG_SCANNER_WIFI_SSID),
        .psk         = (const uint8_t *)CONFIG_SCANNER_WIFI_PSK,
        .psk_length  = strlen(CONFIG_SCANNER_WIFI_PSK),
        .channel     = WIFI_CHANNEL_ANY,
        .security    = WIFI_SECURITY_TYPE_PSK,
        .band        = WIFI_FREQ_BAND_2_4_GHZ,
        .mfp         = WIFI_MFP_OPTIONAL,
    };

    LOG_INF("Connecting to SSID: %s", CONFIG_SCANNER_WIFI_SSID);
    int rc = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
    if (rc != 0) {
        LOG_ERR("net_mgmt connect failed: %d", rc);
        return rc;
    }

    /* Wait for association (10 s) */
    if (k_sem_take(&wifi_connected_sem, K_SECONDS(10)) != 0) {
        LOG_ERR("WiFi association timeout");
        return -ETIMEDOUT;
    }

    /* Start DHCP */
    net_dhcpv4_start(iface);

    /* Wait for IP (20 s) */
    if (k_sem_take(&ip_obtained_sem, K_SECONDS(20)) != 0) {
        LOG_ERR("DHCP timeout");
        return -ETIMEDOUT;
    }

    return 0;
}
