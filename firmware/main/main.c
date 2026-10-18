/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * s31-sdr: capture 10-bit I/Q from the ESP32-S31's receiver, and bring up
 * Gigabit Ethernet (net.c).
 *
 * The RF test library's capture path needs the PHY receiving normally: Wi-Fi
 * STA up, promiscuous, fixed channel, no power save. rftest_open_clk() turns
 * on the modem clocks (including the LP timer adctrig's done-poll uses).
 * adctrig(count, 0, ...) pulses the software trigger and the engine writes
 * count + 1 words at 80 MS/s to S31_DUMP_SRAM (s31_dump.h). It owns the top
 * 128 KB of SRAM while it runs, so it is called with interrupts off and the
 * other core stalled.
 */
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_task_wdt.h"
#include "esp_ipc_isr.h"
#include "s31_dump.h"
#include "net.h"

extern void rftest_open_clk(void);
extern void adctrig(int count, int mode, int a2, int a3, int a4,
                    int a5, int a6, int a7, int s9, int s10);

/* The cert-test library references a command-console parser we don't use. */
int cmd_parse(int argc, char **argv) { (void)argc; (void)argv; return 0; }

/* Start-up channel (2437 MHz). */
#define START_CHANNEL 6
#define SNAP          16383          /* adctrig writes SNAP + 1 words: the 64 KB bank */
#define MARKER        0x5EED1234u

static portMUX_TYPE cap_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t *const bank = (volatile uint32_t *)S31_DUMP_SRAM;

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

/* One capture of `count` samples into the bank; false if the engine did not
 * write the last word. */
static bool capture(int count)
{
    bank[count] = MARKER;
    taskENTER_CRITICAL(&cap_mux);
    esp_ipc_isr_stall_other_cpu();
    adctrig(count, 0, 0, 1, 0, 0, 0, 0, 0, 0);
    esp_ipc_isr_release_other_cpu();
    taskEXIT_CRITICAL(&cap_mux);
    return bank[count] != MARKER;
}

/* Sample word: I[9:0] | Q[19:10] | AGC gain index[27:20], I and Q signed. */
static int word_i(uint32_t w) { return (int32_t)((w & 0x3FF) << 22) >> 22; }
static int word_q(uint32_t w) { return (int32_t)(((w >> 10) & 0x3FF) << 22) >> 22; }
static int word_gain(uint32_t w) { return (w >> 20) & 0xFF; }

static void report(int k, int n)
{
    int32_t si = 0, sq = 0;
    uint32_t sg = 0;
    for (int i = 0; i < n; i++) { si += word_i(bank[i]); sq += word_q(bank[i]); sg += word_gain(bank[i]); }
    int32_t mi = si / n, mq = sq / n;
    int64_t p = 0;
    for (int i = 0; i < n; i++) {
        int32_t di = word_i(bank[i]) - mi, dq = word_q(bank[i]) - mq;
        p += di * di + dq * dq;
    }
    printf("capture %d: %d samples, power %.1f, mean gain index %.1f\n",
           k, n, (double)p / n, (double)sg / n);
}

static void capture_task(void *arg)
{
    rftest_open_clk();
    for (int k = 0; ; k++) {
        if (capture(SNAP)) report(k, SNAP + 1);
        else printf("capture %d: the engine wrote nothing\n", k);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    esp_task_wdt_deinit();
    bring_up_phy();
    printf("s31-sdr: receiver up on channel %d (%d MHz)\n", START_CHANNEL, 2407 + 5 * START_CHANNEL);
    s31_net_start();
    xTaskCreatePinnedToCore(capture_task, "capture", 8192, NULL, 10, NULL, 1);
}
