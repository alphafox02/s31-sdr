# Copyright 2026 CEMAXECUTER LLC
# SPDX-License-Identifier: Apache-2.0
set(srcs "main.c" "dump_bank.c" "net.c" "stream.c" "cam_lsb.c" "dma_pos.c")
if(CONFIG_S31SDR_RTL_SDR)
    list(APPEND srcs "usb_rtl.c" "rtl_drv.c")
endif()

# WHOLE_ARCHIVE keeps dump_bank.c (no exported symbols) so its memory
# reservation is not dropped by the linker.
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "." "../include"
                       REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_phy heap esp_timer
                                esp_rom esp_driver_uart esp_driver_gpio esp_hw_support esp_driver_parlio esp_eth lwip esp_driver_dma esp_hal_cam
                       WHOLE_ARCHIVE)
