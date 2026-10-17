/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * s31-sdr: bring up the ESP32-S31's 2.4 GHz receiver.
 *
 * The RF test library's capture path needs the PHY receiving normally: Wi-Fi
 * STA up, promiscuous, fixed channel, no power save.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

/* The cert-test library references a command-console parser we don't use. */
int cmd_parse(int argc, char **argv) { (void)argc; (void)argv; return 0; }

/* Start-up channel (2437 MHz). */
#define START_CHANNEL 6

static void bring_up_phy(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(START_CHANNEL, WIFI_SECOND_CHAN_NONE));
    vTaskDelay(pdMS_TO_TICKS(300));
}

void app_main(void)
{
    bring_up_phy();
    printf("s31-sdr: receiver up on channel %d (%d MHz)\n", START_CHANNEL, 2407 + 5 * START_CHANNEL);
}
