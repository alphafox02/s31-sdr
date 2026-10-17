# Copyright 2026 CEMAXECUTER LLC
# SPDX-License-Identifier: Apache-2.0
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_phy)
