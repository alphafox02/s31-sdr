/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * Gigabit Ethernet bring-up: on-chip EMAC + YT8531 over RGMII via
 * espressif/ethernet_init, static IPv4 from local_net.h (gitignored; see
 * local_net.h.example). Call after esp_netif_init() and the default event loop.
 */
#include <stdio.h>
#include <string.h>
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "ethernet_init.h"
#include "net.h"
#include <inttypes.h>
#include "local_net.h"

static esp_netif_t *s_netif;
static esp_eth_handle_t s_eth;
static volatile int s_link_up;

static void on_eth_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_eth_handle_t h = *(esp_eth_handle_t *)data;
    if (id == ETHERNET_EVENT_CONNECTED) {
        eth_speed_t speed; eth_duplex_t duplex;
        esp_eth_ioctl(h, ETH_CMD_G_SPEED, &speed);
        esp_eth_ioctl(h, ETH_CMD_G_DUPLEX_MODE, &duplex);
        printf("eth: link up, %s, %s duplex\n",
               speed == ETH_SPEED_1000M ? "1000 Mbps" : speed == ETH_SPEED_100M ? "100 Mbps" : "10 Mbps",
               duplex == ETH_DUPLEX_FULL ? "full" : "half");
        s_link_up = 1;
    } else if (id == ETHERNET_EVENT_DISCONNECTED) {
        printf("eth: link down\n");
        s_link_up = 0;
    }
}

esp_netif_t *s31_net_start(void)
{
    esp_eth_handle_t *handles; uint8_t n = 0;
    ESP_ERROR_CHECK(ethernet_init_all(&handles, &n));
    if (n < 1) { printf("eth: no interface\n"); return NULL; }
    s_eth = handles[0];

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    s_netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(s_netif));
    esp_netif_ip_info_t ip = {0};
    ip.ip.addr = esp_ip4addr_aton(S31_IP);
    ip.netmask.addr = esp_ip4addr_aton(S31_NETMASK);
    ip.gw.addr = esp_ip4addr_aton(S31_GW);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_netif, &ip));
    ESP_ERROR_CHECK(esp_netif_attach(s_netif, esp_eth_new_netif_glue(handles[0])));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, on_eth_event, NULL));
    ESP_ERROR_CHECK(esp_eth_start(handles[0]));
    printf("eth: started, static address configured\n");
    return s_netif;
}

int s31_net_link_up(void) { return s_link_up; }

esp_eth_handle_t s31_net_eth_handle(void) { return s_eth; }
