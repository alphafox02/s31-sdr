/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * s31-sdr: stream the receiver's I/Q as VITA-49 over Gigabit Ethernet.
 *
 * The RF test library's capture path needs the PHY receiving normally: Wi-Fi
 * STA up, promiscuous, fixed channel, no power save. rftest_open_clk() turns
 * on the modem clocks (including the LP timer adctrig's done-poll uses).
 * stream.c does the rest.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_task_wdt.h"
#include "net.h"
#include "stream.h"

extern void rftest_open_clk(void);

/* The cert-test library references a command-console parser we don't use. */
int cmd_parse(int argc, char **argv) { (void)argc; (void)argv; return 0; }

/* Stream format: 10 = 10-bit I/Q as CI16 at 20 MS/s (20 MHz filter),
 * 8 = CI8 at 40 MS/s (40 MHz filter). */
#define STREAM_BITS 10
/* Start-up channel (2437 MHz); retune over VITA 49.2 or the text port. */
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

static void stream_task(void *arg)
{
    rftest_open_clk();
    s31_net_start();
    while (!s31_net_link_up()) vTaskDelay(pdMS_TO_TICKS(100));
    s31_stream_main(START_CHANNEL, STREAM_BITS);
    vTaskDelete(NULL);
}

void app_main(void)
{
    s31_stream_reserve(STREAM_BITS);
    esp_task_wdt_deinit();
    bring_up_phy();
    xTaskCreatePinnedToCore(stream_task, "stream", 8192, NULL, 10, NULL, 1);
    for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
}
