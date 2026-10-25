/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * Voice control for the radio, with Espressif's ESP-SR: say "Hi ESP", the
 * radio ducks and chimes, then say a command within five seconds: "next
 * station", "volume up", "ninety nine point five" and so on (the list is
 * below). A rising chime means listening, a single note means done, a
 * falling one means nothing was understood.
 *
 * The ES8311 records the microphone in its left slot and, in its right, a
 * copy of what its DAC is playing: the reference that lets the front end
 * cancel the radio's own sound from the speaker. feed_task takes both at
 * 48 kHz, filters them down to 16 kHz and feeds the front end (echo
 * cancellation, voice activity, WakeNet); detect_task fetches its output
 * and, after a wake-up, runs MultiNet on it.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_speech_commands.h"
#include "model_path.h"
#include "stream.h"
#include "radio.h"
#include "voice.h"

#define PI            3.14159265f
#define DECIM         3             /* 48 kHz -> 16 kHz */
#define TAPS          48
#define LISTEN_MS     5000

enum { CMD_NEXT = 1, CMD_PREV, CMD_UP, CMD_DOWN, CMD_MUTE, CMD_UNMUTE, CMD_SCAN, CMD_TUNE = 100 };

static const struct { int id; const char *say; } phrases[] = {
    { CMD_NEXT, "next station" }, { CMD_NEXT, "station up" }, { CMD_NEXT, "seek up" },
    { CMD_PREV, "previous station" }, { CMD_PREV, "station down" }, { CMD_PREV, "seek down" },
    { CMD_UP, "volume up" }, { CMD_UP, "louder" },
    { CMD_DOWN, "volume down" }, { CMD_DOWN, "quieter" },
    { CMD_MUTE, "mute" }, { CMD_MUTE, "radio off" },
    { CMD_UNMUTE, "unmute" }, { CMD_UNMUTE, "radio on" },
    { CMD_SCAN, "scan" }, { CMD_SCAN, "scan stations" },
};

static const esp_afe_sr_iface_t *afe;
static esp_afe_sr_data_t *afe_data;
static esp_mn_iface_t *mn;
static model_iface_data_t *mn_data;
static bool listening;
static volatile float mic_db = -99, ref_db = -99;

/* ---- the command list ---- */

static const char *ones[] = { "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine" };
static const char *tens[] = { "", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety" };

/* The US FM channels as spoken: "eighty eight point one" .. "one oh seven
 * point nine", and "one hundred seven point nine" too. */
static int add_channels(void)
{
    int n = 0;
    char s[64];
    for (int c = 0; c < 100; c++) {
        int tenth_hz = 881 + 2 * c, mhz = tenth_hz / 10, tenth = tenth_hz % 10;
        if (mhz < 100) {
            snprintf(s, sizeof(s), "%s%s%s point %s", tens[mhz / 10], mhz % 10 ? " " : "", mhz % 10 ? ones[mhz % 10] : "",
                     ones[tenth]);
            n += esp_mn_commands_add(CMD_TUNE + c, s) == ESP_OK;
        } else if (mhz == 100) {
            snprintf(s, sizeof(s), "one hundred point %s", ones[tenth]);
            n += esp_mn_commands_add(CMD_TUNE + c, s) == ESP_OK;
        } else {
            snprintf(s, sizeof(s), "one oh %s point %s", ones[mhz - 100], ones[tenth]);
            n += esp_mn_commands_add(CMD_TUNE + c, s) == ESP_OK;
            snprintf(s, sizeof(s), "one hundred %s point %s", ones[mhz - 100], ones[tenth]);
            n += esp_mn_commands_add(CMD_TUNE + c, s) == ESP_OK;
        }
    }
    return n;
}

static void act(int id)
{
    char reply[256], c[24];
    if (id >= CMD_TUNE && id < CMD_TUNE + 100) {
        int tenth_hz = 881 + 2 * (id - CMD_TUNE);
        snprintf(c, sizeof(c), "f %d.%d", tenth_hz / 10, tenth_hz % 10);
    } else {
        const char *map[] = { [CMD_NEXT] = "seek up", [CMD_PREV] = "seek down", [CMD_UP] = "vol up",
                              [CMD_DOWN] = "vol down", [CMD_MUTE] = "off", [CMD_UNMUTE] = "on", [CMD_SCAN] = "scan" };
        if (id <= 0 || id > CMD_SCAN) return;
        snprintf(c, sizeof(c), "%s", map[id]);
    }
    s31_radio_command(c, reply, sizeof(reply));
    printf("@@VOICE %s -> %s\n", c, reply);
}

/* ---- audio in ---- */

static float taps[TAPS];

static void feed_task(void *arg)
{
    int chunk = afe->get_feed_chunksize(afe_data);      /* 16 kHz samples per channel */
    int n48 = chunk * DECIM;
    int16_t *in = heap_caps_malloc(n48 * 2 * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    int16_t *feed = heap_caps_malloc(chunk * 2 * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    float *h[2] = { heap_caps_calloc(TAPS - 1 + n48, sizeof(float), MALLOC_CAP_INTERNAL),
                    heap_caps_calloc(TAPS - 1 + n48, sizeof(float), MALLOC_CAP_INTERNAL) };
    if (!in || !feed || !h[0] || !h[1]) { printf("@@VOICE out of memory\n"); vTaskDelete(NULL); }
    for (int k = 0; k < TAPS; k++) {                    /* 7 kHz low-pass at 48 kHz (Blackman) */
        float m = k - (TAPS - 1) / 2.0f, fc = 7000.0f / 48000;
        float s = fabsf(m) < 1e-6f ? 2 * fc : sinf(2 * PI * fc * m) / (PI * m);
        taps[k] = s * (0.42f - 0.5f * cosf(2 * PI * k / (TAPS - 1)) + 0.08f * cosf(4 * PI * k / (TAPS - 1)));
    }
    float sum = 0;
    for (int k = 0; k < TAPS; k++) sum += taps[k];
    for (int k = 0; k < TAPS; k++) taps[k] /= sum;

    for (;;) {
        if (s31_radio_read_mic(in, n48 * 2 * sizeof(int16_t))) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        double p[2] = { 0, 0 };
        for (int k = 0; k < n48; k++)
            for (int c = 0; c < 2; c++) {
                float v = in[2 * k + c];
                h[c][TAPS - 1 + k] = v;
                p[c] += v * v;
            }
        for (int j = 0; j < chunk; j++)
            for (int c = 0; c < 2; c++) {
                const float *x = h[c] + j * DECIM;
                float v = 0;
                for (int k = 0; k < TAPS; k++) v += taps[k] * x[k];
                feed[2 * j + c] = v > 32767 ? 32767 : v < -32768 ? -32768 : (int16_t)v;
            }
        for (int c = 0; c < 2; c++) memmove(h[c], h[c] + n48, (TAPS - 1) * sizeof(float));
        mic_db = 10 * log10f(p[0] / n48 / (32768.0f * 32768.0f) + 1e-12f);
        ref_db = 10 * log10f(p[1] / n48 / (32768.0f * 32768.0f) + 1e-12f);
        afe->feed(afe_data, feed);
    }
}

/* ---- recognition ---- */

static void detect_task(void *arg)
{
    int64_t report_at = 0;
    for (;;) {
        afe_fetch_result_t *r = afe->fetch(afe_data);
        if (!r || r->ret_value == ESP_FAIL) continue;
        if (!listening) {
            if (r->wakeup_state == WAKENET_DETECTED) {
                printf("@@VOICE wake word\n");
                listening = true;
                s31_radio_duck(true);
                s31_radio_chime(CHIME_LISTEN);
                mn->clean(mn_data);
                afe->disable_wakenet(afe_data);
            }
            if (esp_timer_get_time() > report_at) {     /* every 30 s: the levels, for setting up */
                report_at = esp_timer_get_time() + 30000000;
                printf("@@VOICE waiting for \"Hi ESP\": mic %.0f dBFS, playback %.0f dBFS\n", mic_db, ref_db);
            }
            continue;
        }
        esp_mn_state_t st = mn->detect(mn_data, r->data);
        if (st == ESP_MN_STATE_DETECTING) continue;
        if (st == ESP_MN_STATE_DETECTED) {
            esp_mn_results_t *res = mn->get_results(mn_data);
            printf("@@VOICE heard \"%s\" (%.2f)\n", res->string, res->num ? res->prob[0] : 0);
            s31_radio_chime(CHIME_DONE);
            s31_radio_duck(false);
            if (res->num) act(res->command_id[0]);
        } else {
            printf("@@VOICE no command heard\n");
            s31_radio_chime(CHIME_MISSED);
            s31_radio_duck(false);
        }
        listening = false;
        afe->enable_wakenet(afe_data);
    }
}

/* ---- start ---- */

void s31_voice_start(void)
{
    srmodel_list_t *models = esp_srmodel_init("model");
    char *wn = models ? esp_srmodel_filter(models, ESP_WN_PREFIX, "hiesp") : NULL;
    char *mname = models ? esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH) : NULL;
    if (!wn || !mname) { printf("@@VOICE models not found in the \"model\" partition\n"); return; }

    afe_config_t *cfg = afe_config_init("MR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (!cfg) { printf("@@VOICE front end config failed\n"); return; }
    cfg->wakenet_model_name = wn;
    cfg->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    cfg->afe_perferred_core = 0;                        /* core 1 is the radio's */
    cfg->afe_perferred_priority = 4;
    afe = esp_afe_handle_from_config(cfg);
    afe_data = afe ? afe->create_from_config(cfg) : NULL;
    afe_config_free(cfg);
    if (!afe_data) { printf("@@VOICE front end failed to start\n"); return; }

    mn = esp_mn_handle_from_name(mname);
    mn_data = mn ? mn->create(mname, LISTEN_MS) : NULL;
    if (!mn_data) { printf("@@VOICE command model failed to load\n"); return; }
    esp_mn_commands_alloc(mn, mn_data);
    int n = 0;
    for (size_t i = 0; i < sizeof(phrases) / sizeof(phrases[0]); i++)
        n += esp_mn_commands_add(phrases[i].id, phrases[i].say) == ESP_OK;
    n += add_channels();
    esp_mn_error_t *err = esp_mn_commands_update();
    printf("@@VOICE ready: say \"Hi ESP\", then a command (%d phrases, %d refused; %s, %s)\n", n,
           err ? err->num : 0, wn, mname);

    s31_spawn(feed_task, "voicein", 4096, NULL, 5, NULL, 0);
    s31_spawn(detect_task, "voice", 8192, NULL, 4, NULL, 0);
}
