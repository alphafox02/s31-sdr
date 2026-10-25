/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * An RTL-SDR dongle on the board's USB-A host port (5 V always on, limited
 * to 500 mA), streamed to the host as a second VITA-49 stream: stream ID
 * RTL_SID on UDP port RTL_PORT, CI8, with context packets ten times a
 * second. Frequency, sample rate and gain are set over VITA 49.2 on the
 * same port, or with "rtl ..." text commands (s31_rtl_command).
 *
 * Devices are reported as "@@USB" console lines and the dongle's rate as
 * "@@RTL" lines once a second. Unplugging it ends its session cleanly;
 * plugging it back in starts a new one. Everything here runs on core 0;
 * apart from the USB host library's own task, the dongle's tasks run below
 * the S31's control tasks, and its packets leave through the S31 sender's
 * queue (s31_vrt_claim / s31_vrt_commit).
 *
 * usb_host_client_handle_events() delivers transfer completions, so the
 * client task only pumps events and the driver runs in its own task.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "usb/usb_host.h"
#include "rtl_drv.h"
#include "stream.h"
#include "usb_rtl.h"
#include "vrt.h"

#define RTL_RATE      2400000               /* at start-up */
#define RTL_FREQ      100000000
#define RTL_MIN_HZ    24000000u
#define RTL_MAX_HZ    1766000000u
#define GAIN_MAX_DB   52.5
#define BULK_EP       0x81
#define BULK_BYTES    16384                 /* 32 x 512-byte packets */
#define BULK_N        8
#define RTL_PORT      4993
#define RTL_SID       0x53310002u
#define RTL_PAYLOAD   1024                  /* 512 CI8 pairs, 16 packets per bulk transfer */
#define VRT_HDR       16

/* USB events, queued by the client callback for client_task */
struct usb_event { bool gone; uint8_t addr; usb_device_handle_t hdl; };
static QueueHandle_t usb_events;

static usb_host_client_handle_t client;
static usb_device_handle_t rtl_hdl;          /* the dongle, while rtl_task runs */
static usb_device_handle_t pending_hdl;      /* plugged in before the last session ended */
static volatile bool rtl_stop;               /* the dongle was unplugged (set and cleared by client_task) */
static int bulk_live;                        /* bulk transfers owned by the USB stack (atomic) */

/* Statistics, updated in the transfer callback (in integers: this CPU has
 * no double-precision FPU) and read and reset once a second by rtl_task,
 * both under stats_lock. */
static portMUX_TYPE stats_lock = portMUX_INITIALIZER_UNLOCKED;
static uint64_t rx_bytes;
static uint32_t rx_errors;
static int64_t sum_i, sum_q;
static uint64_t sum_pow;
static uint32_t n_stat;

/* Stream state. Data and context packets are both built in the transfer
 * callback, the sender queue's only producer. */
static uint64_t sample_count;
static int64_t next_ctx;
static uint32_t data_count, ctx_count;
static volatile bool ctx_now;                /* send a context packet at once */
static volatile uint32_t cur_freq, cur_rate;
static volatile float cur_gain = -1;         /* dB; negative = the tuner's AGC (a float: read in one word) */
static float manual_gain = 30;               /* restored when AGC is switched off */
static rtl_dev_t *rtl;                       /* set while a dongle is streaming */
static SemaphoreHandle_t rtl_mutex;          /* one caller of the driver at a time */

/* ---- packets ---- */

/* One data packet from `len` bytes of samples (unsigned I, Q), sent as
 * signed CI8. If the queue is full the packet is dropped, but the sample
 * count still advances, so the host sees the gap. */
static void send_data(const uint8_t *src, int len)
{
    uint8_t *v = s31_vrt_claim();
    if (v) {
        be32(v, (0x1u << 28) | (1u << 20) | ((data_count++ & 0xF) << 16) | (uint32_t)((VRT_HDR + len) / 4));
        be32(v + 4, RTL_SID);
        be64(v + 8, sample_count);
        const uint32_t *s = (const uint32_t *)src;
        uint32_t *d = (uint32_t *)(v + VRT_HDR);
        for (int k = 0; k < len / 4; k++) d[k] = s[k] ^ 0x80808080u;     /* unsigned -> signed */
        s31_vrt_commit(RTL_PORT, VRT_HDR + len);
    }
    sample_count += len / 2;
}

/* gain field: stage 1, signed, 1/128 dB; -1 dB = AGC */
static uint32_t gain_word(void) { return (uint16_t)(int16_t)(cur_gain < 0 ? -128 : cur_gain * 128 + 0.5); }
static uint32_t se_word(void) { return (1u << 28) | (cur_gain < 0 ? (1u << 16) : 0); }

/* The fields for the CIF0 bits in `want`, in CIF0 order; returns words. */
static int put_fields(uint8_t *p, uint32_t want)
{
    int w = 0;
    if (want & CIF0_BW)   { be64(p, fix20(cur_rate)); p += 8; w += 2; }
    if (want & CIF0_RF)   { be64(p, fix20(cur_freq)); p += 8; w += 2; }
    if (want & CIF0_GAIN) { be32(p, gain_word()); p += 4; w += 1; }
    if (want & CIF0_RATE) { be64(p, fix20(cur_rate)); p += 8; w += 2; }
    if (want & CIF0_SE)   { be32(p, se_word()); p += 4; w += 1; }
    if (want & CIF0_FMT)  { be32(p, (1u << 29) | (7u << 6) | 7u); be32(p + 4, 0); w += 2; }
    return w;
}

#define CTX_FIELDS (CIF0_BW | CIF0_RF | CIF0_GAIN | CIF0_RATE | CIF0_SE | CIF0_FMT)

static void send_context(void)
{
    uint8_t *p = s31_vrt_claim();
    if (!p) return;
    int w = 4;
    be32(p + 4, RTL_SID);
    be64(p + 8, sample_count);
    be32(p + 4 * w, CTX_FIELDS); w++;
    w += put_fields(p + 4 * w, CTX_FIELDS);
    be32(p, (0x4u << 28) | (1u << 20) | ((ctx_count++ & 0xF) << 16) | (uint32_t)w);
    s31_vrt_commit(RTL_PORT, 4 * w);
}

/* ---- USB ---- */

static void host_task(void *arg)
{
    usb_host_config_t cfg = { .skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LOWMED, .peripheral_map = BIT0 };
    esp_err_t err = usb_host_install(&cfg);
    printf("@@USB host %s\n", err == ESP_OK ? "started" : esp_err_to_name(err));
    xTaskNotifyGive((TaskHandle_t)arg);
    if (err != ESP_OK) vTaskDelete(NULL);
    for (;;) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

/* Runs inside usb_host_client_handle_events(), in client_task. */
static void on_client_event(const usb_host_client_event_msg_t *msg, void *arg)
{
    struct usb_event e = { 0 };
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) e.addr = msg->new_dev.address;
    else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) { e.gone = true; e.hdl = msg->dev_gone.dev_hdl; }
    else return;
    if (xQueueSend(usb_events, &e, 0) != pdTRUE) printf("@@USB event queue full\n");
}

/* UTF-16LE string descriptor to ASCII ('?' for anything else). */
static void str_desc(const usb_str_desc_t *d, char *out, size_t n)
{
    size_t k = 0;
    if (d)
        for (int i = 0; i < (d->bLength - 2) / 2 && k + 1 < n; i++)
            out[k++] = d->wData[i] < 0x80 ? (char)d->wData[i] : '?';
    out[k] = 0;
}

/* Open and describe a new device; returns its handle if it is an RTL2832U,
 * otherwise closes it and returns NULL. */
static usb_device_handle_t report(uint8_t addr)
{
    usb_device_handle_t dev;
    if (usb_host_device_open(client, addr, &dev) != ESP_OK) {
        printf("@@USB device at address %d: open failed\n", addr);
        return NULL;
    }
    const usb_device_desc_t *dd;
    usb_device_info_t info;
    usb_host_get_device_descriptor(dev, &dd);
    usb_host_device_info(dev, &info);
    char prod[48], manu[48];
    str_desc(info.str_desc_product, prod, sizeof(prod));
    str_desc(info.str_desc_manufacturer, manu, sizeof(manu));
    printf("@@USB device %04x:%04x, %s speed, \"%s\" \"%s\"\n", dd->idVendor, dd->idProduct,
           (const char *[]){ "low", "full", "high" }[info.speed], manu, prod);
    if (dd->idVendor == 0x0bda && (dd->idProduct == 0x2832 || dd->idProduct == 0x2838)) return dev;
    usb_host_device_close(client, dev);
    return NULL;
}

/* A bulk transfer finished: count it, queue its samples as packets (and a
 * context packet when one is due), and resubmit it unless the dongle is
 * gone. Statistics use every 16th sample pair. */
static void bulk_done(usb_transfer_t *t)
{
    if (t->status == USB_TRANSFER_STATUS_COMPLETED) {
        int32_t si = 0, sq = 0;
        uint32_t sp = 0;
        int n = 0;
        for (int k = 0; k + 1 < t->actual_num_bytes; k += 32, n++) {
            int i = t->data_buffer[k] - 128, q = t->data_buffer[k + 1] - 128;
            si += i; sq += q; sp += i * i + q * q;
        }
        taskENTER_CRITICAL(&stats_lock);
        rx_bytes += t->actual_num_bytes;
        sum_i += si; sum_q += sq; sum_pow += sp; n_stat += n;
        taskEXIT_CRITICAL(&stats_lock);
        int64_t now = esp_timer_get_time();
        if (ctx_now || now >= next_ctx) {
            ctx_now = false;
            next_ctx = now + 100000;
            send_context();
        }
        for (int k = 0; k < t->actual_num_bytes; k += RTL_PAYLOAD) {
            int len = t->actual_num_bytes - k < RTL_PAYLOAD ? t->actual_num_bytes - k : RTL_PAYLOAD;
            send_data(t->data_buffer + k, len & ~3);
        }
    } else if (t->status != USB_TRANSFER_STATUS_NO_DEVICE && t->status != USB_TRANSFER_STATUS_CANCELED) {
        taskENTER_CRITICAL(&stats_lock);
        rx_errors++;
        taskEXIT_CRITICAL(&stats_lock);
    }
    if (rtl_stop || t->status == USB_TRANSFER_STATUS_NO_DEVICE || t->status == USB_TRANSFER_STATUS_CANCELED ||
        usb_host_transfer_submit(t) != ESP_OK)
        __atomic_fetch_sub(&bulk_live, 1, __ATOMIC_ACQ_REL);
}

static void rtl_control_task(void *arg);

/* Release the dongle and end rtl_task, which owns it. */
static void rtl_finish(usb_device_handle_t d, bool claimed, rtl_dev_t *r, const char *why)
{
    rtl_close(r);
    if (claimed) usb_host_interface_release(client, d, 0);
    usb_host_device_close(client, d);
    printf("@@RTL %s\n", why);
    __atomic_store_n(&rtl_hdl, NULL, __ATOMIC_RELEASE);
    vTaskDelete(NULL);
}

/* One dongle session: set it up, keep BULK_N transfers in flight until it
 * is unplugged, then clean up. */
static void rtl_task(void *arg)
{
    static bool control_started;
    usb_device_handle_t d = (usb_device_handle_t)arg;
    if (usb_host_interface_claim(client, d, 0, 0) != ESP_OK) rtl_finish(d, false, NULL, "interface claim failed");
    int64_t t0 = esp_timer_get_time();
    rtl_dev_t *r = rtl_open(client, d);
    if (!r) rtl_finish(d, true, NULL, "init failed");
    uint32_t rate = 0, freq = 0;
    int e = rtl_set_sample_rate(r, RTL_RATE, &rate);
    e |= rtl_set_freq(r, RTL_FREQ, &freq);
    e |= rtl_reset_buffer(r);
    cur_rate = rate;
    cur_freq = freq;
    cur_gain = -1;
    rtl = r;
    printf("@@RTL %s ready in %lld ms%s: %lu S/s, %lu Hz\n", rtl_tuner_name(r),
           (long long)(esp_timer_get_time() - t0) / 1000, e ? " with errors" : "",
           (unsigned long)rate, (unsigned long)freq);
    if (!control_started)                        /* the VITA 49.2 port opens with the first dongle */
        control_started = s31_spawn(rtl_control_task, "rtlctl", 3072, NULL, 5, NULL, 0);

    taskENTER_CRITICAL(&stats_lock);
    rx_errors = 0;
    taskEXIT_CRITICAL(&stats_lock);
    usb_transfer_t *xf[BULK_N] = { 0 };
    for (int k = 0; k < BULK_N; k++) {
        if (usb_host_transfer_alloc(BULK_BYTES, 0, &xf[k]) != ESP_OK) { printf("@@RTL bulk alloc failed\n"); break; }
        xf[k]->device_handle = d;
        xf[k]->bEndpointAddress = BULK_EP;
        xf[k]->num_bytes = BULK_BYTES;
        xf[k]->callback = bulk_done;
        __atomic_fetch_add(&bulk_live, 1, __ATOMIC_ACQ_REL);
        if (usb_host_transfer_submit(xf[k]) != ESP_OK) {
            __atomic_fetch_sub(&bulk_live, 1, __ATOMIC_ACQ_REL);
            printf("@@RTL bulk submit failed\n");
        }
    }
    int64_t tp = esp_timer_get_time();
    taskENTER_CRITICAL(&stats_lock);
    uint64_t b0 = rx_bytes;
    sum_i = sum_q = sum_pow = 0;
    n_stat = 0;
    taskEXIT_CRITICAL(&stats_lock);
    while (!rtl_stop) {
        for (int k = 0; k < 10 && !rtl_stop; k++) vTaskDelay(pdMS_TO_TICKS(100));
        if (rtl_stop) break;
        int64_t now = esp_timer_get_time();
        taskENTER_CRITICAL(&stats_lock);
        uint64_t b = rx_bytes;
        int64_t si = sum_i, sq = sum_q;
        uint64_t sp = sum_pow;
        uint32_t n = n_stat, errs = rx_errors;
        sum_i = sum_q = sum_pow = 0;
        n_stat = 0;
        taskEXIT_CRITICAL(&stats_lock);
        double mi = n ? (double)si / n : 0, mq = n ? (double)sq / n : 0, p = n ? (double)sp / n : 0;
        printf("@@RTL %.3f MS/s, mean I %+.1f Q %+.1f, rms %.1f, USB errors %lu, queue drops %lu\n",
               (b - b0) / 2.0 / ((now - tp) / 1e6) / 1e6, mi, mq, sqrt(p), (unsigned long)errs,
               (unsigned long)s31_vrt_queue_drops());
        b0 = b;
        tp = now;
    }

    /* Unplugged: the USB stack retires the transfers (NO_DEVICE). Give it
     * time, then cancel whatever it still holds. */
    xSemaphoreTake(rtl_mutex, portMAX_DELAY);
    rtl = NULL;
    xSemaphoreGive(rtl_mutex);
    for (int i = 0; i < 50 && __atomic_load_n(&bulk_live, __ATOMIC_ACQUIRE) > 0; i++) vTaskDelay(pdMS_TO_TICKS(10));
    if (__atomic_load_n(&bulk_live, __ATOMIC_ACQUIRE) > 0) {
        usb_host_endpoint_halt(d, BULK_EP);
        usb_host_endpoint_flush(d, BULK_EP);
        for (int i = 0; i < 50 && __atomic_load_n(&bulk_live, __ATOMIC_ACQUIRE) > 0; i++) vTaskDelay(pdMS_TO_TICKS(10));
    }
    bool held = __atomic_load_n(&bulk_live, __ATOMIC_ACQUIRE) > 0;
    if (!held)
        for (int k = 0; k < BULK_N; k++) if (xf[k]) usb_host_transfer_free(xf[k]);
    rtl_finish(d, true, r, held ? "stopped (transfers still held by the USB stack)" : "stopped");
}

static void client_task(void *arg)
{
    usb_host_client_config_t cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = { .client_event_callback = on_client_event, .callback_arg = NULL },
    };
    if (usb_host_client_register(&cfg, &client) != ESP_OK) {
        printf("@@USB client register failed\n");
        vTaskDelete(NULL);
    }
    for (;;) {
        usb_host_client_handle_events(client, pdMS_TO_TICKS(100));
        struct usb_event e;
        while (xQueueReceive(usb_events, &e, 0) == pdTRUE) {
            if (e.gone) {
                printf("@@USB device removed\n");
                if (e.hdl == rtl_hdl) rtl_stop = true;
                if (e.hdl == pending_hdl) { usb_host_device_close(client, pending_hdl); pending_hdl = NULL; }
                continue;
            }
            usb_device_handle_t d = report(e.addr);
            if (!d) continue;
            if (pending_hdl) usb_host_device_close(client, pending_hdl);
            pending_hdl = d;                         /* one dongle at a time; this one is next */
            if (rtl_hdl) printf("@@USB RTL-SDR waiting: one streams at a time\n");
        }
        /* start the waiting dongle once no session is running */
        if (pending_hdl && !__atomic_load_n(&rtl_hdl, __ATOMIC_ACQUIRE)) {
            rtl_hdl = pending_hdl;
            pending_hdl = NULL;
            rtl_stop = false;
            if (!s31_spawn(rtl_task, "rtl", 3072, rtl_hdl, 3, NULL, 0)) {
                usb_host_device_close(client, rtl_hdl);
                rtl_hdl = NULL;
            }
        }
    }
}

/* ---- control ---- */

static bool rate_ok(uint32_t r) { return (r > 225000 && r <= 300000) || (r > 900000 && r <= 3200000); }
static bool in_u32(double v) { return v >= 0 && v < 4294967295.0; }

/* Each returns 0, or -1 with the dongle unchanged. Called with rtl_mutex. */
static int do_freq(uint32_t hz)
{
    if (!rtl || hz < RTL_MIN_HZ || hz > RTL_MAX_HZ) return -1;
    uint32_t act;
    if (rtl_set_freq(rtl, hz, &act)) { rtl_set_freq(rtl, cur_freq, NULL); return -1; }
    cur_freq = act;
    ctx_now = true;
    return 0;
}

static int do_rate(uint32_t r)
{
    uint32_t act;
    if (!rtl || !rate_ok(r) || rtl_set_sample_rate(rtl, r, &act)) return -1;
    cur_rate = act;
    ctx_now = true;
    return 0;
}

/* Gain in dB, clamped to the tuner's range; negative = AGC. cur_gain gets
 * the gain actually set: the tuner's step at or below `db`. */
static int do_gain(double db)
{
    if (!rtl || isnan(db)) return -1;
    if (db > GAIN_MAX_DB) db = GAIN_MAX_DB;
    double act;
    if (rtl_set_gain(rtl, db < 0 ? -1 : db, &act)) return -1;
    cur_gain = act;
    if (act >= 0) manual_gain = act;
    ctx_now = true;
    return 0;
}

bool s31_rtl_command(const char *cmd, char *reply, size_t n)
{
    if (!rtl) { snprintf(reply, n, "no RTL-SDR"); return true; }
    double v;
    int e = 0;
    xSemaphoreTake(rtl_mutex, portMAX_DELAY);
    if (sscanf(cmd, "f %lf", &v) == 1) e = in_u32(v) ? do_freq((uint32_t)(v + 0.5)) : -1;
    else if (strncmp(cmd, "gain auto", 9) == 0) e = do_gain(-1);
    else if (sscanf(cmd, "gain %lf", &v) == 1) e = do_gain(v < 0 ? 0 : v);
    else if (sscanf(cmd, "rate %lf", &v) == 1) e = in_u32(v) ? do_rate((uint32_t)(v + 0.5)) : -1;
    else if (strncmp(cmd, "status", 6) != 0) { xSemaphoreGive(rtl_mutex); return false; }
    xSemaphoreGive(rtl_mutex);
    char g[16];
    if (cur_gain < 0) snprintf(g, sizeof(g), "auto");
    else snprintf(g, sizeof(g), "%.1f dB", cur_gain);
    snprintf(reply, n, "%s%lu Hz, %lu S/s, gain %s", e ? "refused, still " : "ok ",
             (unsigned long)cur_freq, (unsigned long)cur_rate, g);
    return true;
}

/* VITA 49.2 controllee for the RTL stream, as stream.c's is for the S31
 * stream (SoapyVITA49's Control.cpp is the reference). */
static void rtl_control_task(void *arg)
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in me = { .sin_family = AF_INET, .sin_port = htons(RTL_PORT),
                              .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (s < 0 || bind(s, (struct sockaddr *)&me, sizeof(me)) != 0) {
        printf("@@RTL VITA 49.2 control port %d unavailable\n", RTL_PORT);
        vTaskDelete(NULL);
    }
    static uint8_t in[512], out[256];
    uint32_t acount = 0;
    for (;;) {
        struct sockaddr_in from; socklen_t fl = sizeof(from);
        int n = recvfrom(s, in, sizeof(in), 0, (struct sockaddr *)&from, &fl);
        if (n < 16 || !rtl) continue;
        uint32_t w0 = rd_be32(in);
        unsigned type = w0 >> 28;
        if ((type != 6 && type != 7) || (w0 & (1u << 26))) continue;   /* commands only */
        int size = w0 & 0xFFFF;
        if (size * 4 > n) continue;
        int i = 2;                                    /* word0, stream ID */
        if (w0 & (1u << 27)) i += 2;                  /* class ID */
        if ((w0 >> 22) & 3) i += 1;
        if ((w0 >> 20) & 3) i += 2;
        if (i + 2 > size) continue;
        uint32_t cam = rd_be32(in + 4 * i), msg = rd_be32(in + 4 * (i + 1));
        int id0 = i + 2, ids = 0;
        if (cam & CAM_CONTROLLEE_EN) ids += (cam & CAM_CONTROLLEE_UID) ? 4 : 1;
        if (cam & CAM_CONTROLLER_EN) ids += (cam & CAM_CONTROLLER_UID) ? 4 : 1;
        int f = id0 + ids;
        uint32_t cif0 = f < size ? rd_be32(in + 4 * f) : 0;
        bool query = cam & CAM_REQ_QUERY, partial = false, bad = false;

        /* walk the fields */
        int w = f + 1;
        double rf = 0, rate = 0, bw = 0, gain = 0;
        bool have_rf = false, have_rate = false, have_bw = false, have_gain = false, have_agc = false, agc_on = false;
        for (int bit = 31; bit >= 0 && cif0; bit--) {
            if (!(cif0 & (1u << bit)) || bit == 31) continue;
            int nw = cif0_words(bit);
            if (nw < 0 || w + nw > size) { bad = true; break; }
            if (bit == 29) { bw = rd_fix20(in + 4 * w); have_bw = true; }
            if (bit == 27) { rf = rd_fix20(in + 4 * w); have_rf = true; }
            if (bit == 21) { rate = rd_fix20(in + 4 * w); have_rate = true; }
            if (bit == 23) { gain = (int16_t)(rd_be32(in + 4 * w) & 0xFFFF) / 128.0; have_gain = true; }
            if (bit == 16) {
                uint32_t se = rd_be32(in + 4 * w);
                if ((se >> 28) & 1) { have_agc = true; agc_on = (se >> 16) & 1; }
            }
            w += nw;
        }

        if (!query && !bad) {
            xSemaphoreTake(rtl_mutex, portMAX_DELAY);
            if (have_rf) {
                if (!in_u32(rf) || do_freq((uint32_t)(rf + 0.5))) bad = true;
                else if (fabs(cur_freq - rf) > 1e3) partial = true;
                printf("@@RTL vrt49.2: tune %.6f MHz -> %s, %.6f MHz\n", rf / 1e6, bad ? "refused" : "ok", cur_freq / 1e6);
            }
            if (have_rate) {
                if (!in_u32(rate) || do_rate((uint32_t)(rate + 0.5))) bad = true;
                else if (fabs(cur_rate - rate) > 1) partial = true;
            }
            if (have_bw && fabs(bw - cur_rate) > 1) partial = true;      /* bandwidth follows the rate */
            if (have_agc && do_gain(agc_on ? -1 : manual_gain)) bad = true;
            if (have_gain) {
                if (do_gain(gain < 0 ? 0 : gain)) bad = true;       /* AGC is the State/Event field's */
                else if (fabs(gain - cur_gain) > 0.01) partial = true;
            }
            xSemaphoreGive(rtl_mutex);
        }

        bool want_ack = query || (cam & (CAM_REQ_EXECUTION | CAM_REQ_VALIDATION));
        if ((cam & CAM_NACK_ONLY) && !bad && !partial) want_ack = false;
        if (!want_ack) continue;

        /* acknowledgement */
        uint32_t acam = cam & (CAM_CONTROLLEE_EN | CAM_CONTROLLEE_UID | CAM_CONTROLLER_EN | CAM_CONTROLLER_UID);
        if (query) acam |= CAM_REQ_QUERY;
        else acam |= (cam & (CAM_REQ_EXECUTION | CAM_REQ_VALIDATION)) | CAM_ACK_EXECUTED;
        if (partial) acam |= CAM_ACK_PARTIAL;
        if (bad) acam |= (1u << 16);                   /* errors */
        be32(out + 4, rd_be32(in + 4));                /* stream ID */
        int o = 8;
        be32(out + o, acam); o += 4;
        be32(out + o, msg); o += 4;
        memcpy(out + o, in + 4 * id0, 4 * ids); o += 4 * ids;
        if (query && !bad) {
            uint32_t want = cif0 & CTX_FIELDS;
            be32(out + o, want); o += 4;
            o += 4 * put_fields(out + o, want);
        }
        be32(out, (type << 28) | (1u << 26) | ((acount++ & 0xF) << 16) | (uint32_t)(o / 4));
        sendto(s, out, o, 0, (struct sockaddr *)&from, fl);
    }
}

void s31_usb_start(void)
{
    rtl_mutex = xSemaphoreCreateMutex();
    usb_events = xQueueCreate(8, sizeof(struct usb_event));
    if (!s31_spawn(host_task, "usbhost", 2560, xTaskGetCurrentTaskHandle(), 6, NULL, 0)) return;
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
    s31_spawn(client_task, "usbrtl", 3072, NULL, 4, NULL, 0);
}
