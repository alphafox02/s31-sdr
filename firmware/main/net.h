/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "esp_netif.h"
#include "esp_eth.h"

esp_netif_t *s31_net_start(void);
int s31_net_link_up(void);
esp_eth_handle_t s31_net_eth_handle(void);
