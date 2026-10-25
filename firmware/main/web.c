/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * The radio's Wi-Fi hotspot and web remote. Join the hotspot and open
 * http://192.168.4.1: the page (radio.html) tunes, seeks and scans with
 * /cmd?c=<radio command>, polls /status, and can play the radio on the
 * phone itself: /audio is a WebSocket carrying 48 kHz 16-bit mono PCM,
 * 40 ms per message, which the page schedules with Web Audio (no HTTPS
 * needed, unlike an AudioWorklet).
 *
 * radio_task hands each audio block to s31_web_audio, which fills frames
 * in a small ring; audio_task sends them to every listening phone, so a
 * slow phone holds up only this task, never the radio.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "stream.h"
#include "radio.h"
#include "web.h"

#define MAX_LISTENERS 4
#define FRAME_SAMPLES 1920              /* 40 ms at 48 kHz */
#define FRAMES_N      8

extern const char radio_html_start[] asm("_binary_radio_html_start");
extern const char radio_html_end[] asm("_binary_radio_html_end");

static httpd_handle_t server;
static TaskHandle_t audio_tid;

/* WebSocket listeners, by socket; under lock */
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static int listeners[MAX_LISTENERS];
static volatile int n_listeners;

/* audio frames: radio_task fills, audio_task sends */
static int16_t frames[FRAMES_N][FRAME_SAMPLES];
static uint32_t frame_head, frame_tail;
static int fill;

/* ---- audio ---- */

void s31_web_audio(const int16_t *stereo, int n)
{
    if (!n_listeners) { fill = 0; return; }
    for (int k = 0; k < n; k++) {
        if (frame_head - __atomic_load_n(&frame_tail, __ATOMIC_ACQUIRE) >= FRAMES_N) { fill = 0; return; }
        frames[frame_head % FRAMES_N][fill++] = stereo[2 * k];
        if (fill == FRAME_SAMPLES) {
            fill = 0;
            __atomic_store_n(&frame_head, frame_head + 1, __ATOMIC_RELEASE);
            xTaskNotifyGive(audio_tid);
        }
    }
}

static void drop_listener(int fd)
{
    taskENTER_CRITICAL(&lock);
    for (int i = 0; i < n_listeners; i++)
        if (listeners[i] == fd) { listeners[i] = listeners[--n_listeners]; break; }
    taskEXIT_CRITICAL(&lock);
}

static void audio_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (frame_tail != __atomic_load_n(&frame_head, __ATOMIC_ACQUIRE)) {
            int fds[MAX_LISTENERS], n;
            taskENTER_CRITICAL(&lock);
            n = n_listeners;
            memcpy(fds, listeners, sizeof(fds));
            taskEXIT_CRITICAL(&lock);
            httpd_ws_frame_t f = {
                .type = HTTPD_WS_TYPE_BINARY, .final = true,
                .payload = (uint8_t *)frames[frame_tail % FRAMES_N], .len = sizeof(frames[0]),
            };
            for (int i = 0; i < n; i++)
                if (httpd_ws_send_frame_async(server, fds[i], &f) != ESP_OK) {
                    drop_listener(fds[i]);
                    httpd_sess_trigger_close(server, fds[i]);
                }
            __atomic_store_n(&frame_tail, frame_tail + 1, __ATOMIC_RELEASE);
        }
    }
}

/* ---- HTTP ---- */

static esp_err_t page(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, radio_html_start, radio_html_end - radio_html_start - 1);
}

/* %xx and '+' in a query value, in place */
static void url_decode(char *s)
{
    char *o = s;
    for (; *s; s++, o++) {
        unsigned v;
        if (*s == '+') *o = ' ';
        else if (*s == '%' && sscanf(s + 1, "%2x", &v) == 1) { *o = (char)v; s += 2; }
        else *o = *s;
    }
    *o = 0;
}

static esp_err_t cmd(httpd_req_t *req)
{
    char q[160], c[128], reply[512];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK || httpd_query_key_value(q, "c", c, sizeof(c)) != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "c=<command>");
    url_decode(c);
    if (!s31_radio_command(c, reply, sizeof(reply))) snprintf(reply, sizeof(reply), "unknown command");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, reply);
}

static esp_err_t status(httpd_req_t *req)
{
    char j[768];
    s31_radio_status_json(j, sizeof(j));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, j);
}

/* After the WebSocket handshake: a new listener. */
static esp_err_t audio_open(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    bool added = false;
    taskENTER_CRITICAL(&lock);
    if (n_listeners < MAX_LISTENERS) { listeners[n_listeners++] = fd; added = true; }
    taskEXIT_CRITICAL(&lock);
    printf("@@WEB listener %s (%d)\n", added ? "joined" : "refused, too many", n_listeners);
    return added ? ESP_OK : ESP_FAIL;
}

static esp_err_t audio(httpd_req_t *req)
{
    httpd_ws_frame_t f = { 0 };                         /* the page sends nothing; read and drop */
    uint8_t buf[64];
    if (httpd_ws_recv_frame(req, &f, 0) != ESP_OK) return ESP_FAIL;
    if (f.len > sizeof(buf)) return ESP_FAIL;
    f.payload = buf;
    return httpd_ws_recv_frame(req, &f, f.len);
}

static void on_close(httpd_handle_t hd, int fd)
{
    bool was = false;
    taskENTER_CRITICAL(&lock);
    for (int i = 0; i < n_listeners; i++) if (listeners[i] == fd) was = true;
    taskEXIT_CRITICAL(&lock);
    drop_listener(fd);
    if (was) printf("@@WEB listener left (%d)\n", n_listeners);
    close(fd);
}

/* ---- start ---- */

static int start_ap(void)
{
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&ic) != ESP_OK) return -1;
    wifi_config_t c = { .ap = { .channel = 6, .max_connection = MAX_LISTENERS, .pmf_cfg.required = false } };
    strlcpy((char *)c.ap.ssid, CONFIG_S31SDR_RADIO_SSID, sizeof(c.ap.ssid));
    c.ap.ssid_len = strlen((char *)c.ap.ssid);
    strlcpy((char *)c.ap.password, CONFIG_S31SDR_RADIO_PASSWORD, sizeof(c.ap.password));
    c.ap.authmode = strlen((char *)c.ap.password) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK || esp_wifi_set_config(WIFI_IF_AP, &c) != ESP_OK ||
        esp_wifi_start() != ESP_OK)
        return -1;
    printf("@@WEB hotspot \"%s\" (%s), page at http://192.168.4.1\n", (char *)c.ap.ssid,
           c.ap.authmode == WIFI_AUTH_OPEN ? "open" : "WPA2");
    return 0;
}

void s31_web_start(void)
{
    if (start_ap()) { printf("@@WEB hotspot failed to start\n"); return; }
    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    hc.core_id = 0;
    hc.max_open_sockets = MAX_LISTENERS + 3;
    hc.lru_purge_enable = true;
    hc.close_fn = on_close;
    if (httpd_start(&server, &hc) != ESP_OK) { printf("@@WEB server failed to start\n"); return; }
    const httpd_uri_t uris[] = {
        { .uri = "/", .method = HTTP_GET, .handler = page },
        { .uri = "/cmd", .method = HTTP_GET, .handler = cmd },
        { .uri = "/status", .method = HTTP_GET, .handler = status },
        { .uri = "/audio", .method = HTTP_GET, .handler = audio, .is_websocket = true,
          .ws_post_handshake_cb = audio_open },
    };
    for (int i = 0; i < 4; i++) httpd_register_uri_handler(server, &uris[i]);
    s31_spawn(audio_task, "webaudio", 3072, NULL, 4, &audio_tid, 0);
}
