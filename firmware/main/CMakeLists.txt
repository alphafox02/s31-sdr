# Copyright 2026 CEMAXECUTER LLC
# SPDX-License-Identifier: Apache-2.0
# WHOLE_ARCHIVE keeps dump_bank.c (no exported symbols) so its memory
# reservation is not dropped by the linker.
idf_component_register(SRCS "main.c" "dump_bank.c"
                       INCLUDE_DIRS "." "../include"
                       REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_phy esp_hw_support
                       WHOLE_ARCHIVE)
