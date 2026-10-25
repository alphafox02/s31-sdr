# Copyright 2026 CEMAXECUTER LLC
# SPDX-License-Identifier: Apache-2.0
set(srcs "main.c" "net.c" "stream.c" "cam_lsb.c" "dma_pos.c")
if(NOT CONFIG_S31SDR_FM_RADIO)
    list(APPEND srcs "dump_bank.c")
endif()
if(CONFIG_S31SDR_RTL_SDR)
    list(APPEND srcs "usb_rtl.c" "rtl_drv.c")
endif()
set(embed "")
if(CONFIG_S31SDR_FM_RADIO)
    list(APPEND srcs "radio.c")
endif()
if(CONFIG_S31SDR_FM_RADIO AND CONFIG_S31SDR_RADIO_VOICE)
    list(APPEND srcs "voice.c")
endif()
if(CONFIG_S31SDR_RADIO_WIFI)
    list(APPEND srcs "web.c")
    list(APPEND embed "radio.html")
endif()

# WHOLE_ARCHIVE keeps dump_bank.c (no exported symbols) so its memory
# reservation is not dropped by the linker.
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "." "../include"
                       REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_phy heap esp_timer
                                esp_rom esp_driver_uart esp_driver_gpio esp_hw_support esp_driver_parlio esp_eth lwip esp_driver_dma esp_hal_cam
                                esp_driver_i2s esp_driver_i2c esp_http_server
                       EMBED_TXTFILES ${embed}
                       WHOLE_ARCHIVE)
