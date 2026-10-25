/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * FM broadcast radio on the board's speaker, from the RTL-SDR (mono).
 *
 * The dongle runs at 960 kS/s. The USB callback only copies its samples
 * into a PSRAM ring (s31_radio_feed); radio_task, alone on core 1, does the
 * rest: two half-band filters that keep the station (+-100 kHz) and
 * decimate to 480 and then 240 kS/s, the FM discriminator, 75 us
 * de-emphasis, and a 16 kHz FIR that decimates by 5 to 48 kHz for the
 * ES8311 codec and the NS4150B speaker amplifier. The codec's writes
 * block, so its clock paces the task; the ring takes up the difference
 * from the dongle's clock.
 *
 * Seek and scan work from spectra: an 800 kHz window holds four 200 kHz
 * channels, none at its centre, so the band takes 25 retunes. A channel
 * whose power stands clear of its window's noise floor, and above both
 * neighbours, is a station. The channel grid is the US one (88.1-107.9
 * MHz).
 *
 * Commands (s31_radio_command) come from the BOOT button (a short press
 * seeks up, a long one down), the text port (UDP 4990, over Ethernet),
 * the hotspot's web page (web.c) and voice control (voice.c); radio_task
 * carries them out between audio blocks, so the radio's state has one
 * owner. "listen" on the text port also sends the audio to the asking
 * computer, as 48 kHz 16-bit mono PCM on UDP port 4994. The station and
 * volume are kept in NVS.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"
#include "stream.h"
#include "usb_rtl.h"
#include "radio.h"
#if CONFIG_S31SDR_RADIO_WIFI
#include "web.h"
#endif

/* board wiring (ES8311 codec, NS4150B amplifier) */
#define PIN_I2C_SDA   51
#define PIN_I2C_SCL   50
#define PIN_MCLK      52
#define PIN_BCLK      53
#define PIN_WS        55
#define PIN_DOUT      56            /* to the codec's DSDIN */
#define PIN_DIN       54            /* from the codec's ASDOUT: microphone, playback reference */
#define PIN_PA        57
#define PIN_BOOT      61

#define PI            3.14159265f
#define RTL_RATE      960000
#define IF_RATE       (RTL_RATE / 4)            /* two half-band stages */
#define HB1_TAPS      23            /* 960 -> 480 kS/s: passes 100 kHz, stops from 380 kHz */
#define HB2_TAPS      39            /* 480 -> 240 kS/s: passes 98 kHz, stops from 142 kHz */
#define AUD_DECIM     5             /* 240 kS/s -> 48 kHz */
#define AUDIO_RATE    (IF_RATE / AUD_DECIM)
#define AUD_TAPS      96
#define CHUNK         960           /* input samples per DSP pass: 1 ms */
#define CHUNK_FRAMES  (CHUNK / 4 / AUD_DECIM)   /* 48 audio frames */
#define BLOCK_CHUNKS  10            /* 10 ms of audio per codec write */
#define BLOCK_FRAMES  (BLOCK_CHUNKS * CHUNK_FRAMES)

#define RING_N        (1u << 18)    /* input samples (I, Q byte pairs): 273 ms */
#define MS(n)         ((uint32_t)(n) * (RTL_RATE / 1000))
#define FILL          MS(60)        /* backlog to build up before playing */
#define TOO_LATE      MS(150)       /* beyond this, skip back to FILL */
#define SETTLE        MS(70)        /* samples discarded after a retune */

#define TUNE_LOW_HZ   76000000u
#define TUNE_HIGH_HZ  108000000u
#define FM_LOW_HZ     88100000u
#define FM_HIGH_HZ    107900000u
#define FM_STEP_HZ    200000u
#define N_CHAN        ((FM_HIGH_HZ - FM_LOW_HZ) / FM_STEP_HZ + 1)
#define NFFT          1024
#define SCAN_FFTS     16
#define SCAN_SNR_DB   10.0f
#define LONG_PRESS_MS 600
#define MIC_GAIN_DB   24.0f
#ifdef CONFIG_S31SDR_RADIO_VOICE
#define VOICE         1
#else
#define VOICE         0
#endif

/* ---- input ring: written by the USB callback, read by radio_task ---- */

static uint8_t *ring;
static uint32_t ring_head, ring_tail;       /* in samples */
static volatile uint32_t ring_overflows;

void s31_radio_feed(const uint8_t *iq, int len)
{
    uint32_t n = len / 2, head = ring_head;
    if (!ring || head - __atomic_load_n(&ring_tail, __ATOMIC_ACQUIRE) + n > RING_N) {
        ring_overflows++;
        return;
    }
    uint32_t at = head % RING_N, first = RING_N - at < n ? RING_N - at : n;
    memcpy(ring + 2 * at, iq, 2 * first);
    memcpy(ring, iq + 2 * first, 2 * (n - first));
    __atomic_store_n(&ring_head, head + n, __ATOMIC_RELEASE);
}

static IRAM_ATTR uint32_t backlog(void) { return __atomic_load_n(&ring_head, __ATOMIC_ACQUIRE) - ring_tail; }
static IRAM_ATTR void consume(uint32_t n) { __atomic_store_n(&ring_tail, ring_tail + n, __ATOMIC_RELEASE); }

/* Wait for n samples, up to half a second. */
static bool wait_for(uint32_t n)
{
    int64_t end = esp_timer_get_time() + 500000;
    while (backlog() < n && esp_timer_get_time() < end) vTaskDelay(1);
    return backlog() >= n;
}

/* ---- state, owned by radio_task ---- */

enum op { OP_TUNE, OP_SEEK, OP_SCAN, OP_VOL, OP_MUTE, OP_TONE };
struct req { enum op op; int arg; TaskHandle_t from; int *result; };

static QueueHandle_t reqs;
static esp_codec_dev_handle_t codec;
static volatile uint32_t station_hz = 99500000;
static volatile int volume = 60;
static volatile bool muted, live;
static uint32_t stations[N_CHAN];
static volatile int n_stations;
static uint32_t skip;                        /* samples still to discard */
static bool filling = true;                  /* building up FILL before playing */
static int64_t save_at;                      /* when to write NVS, 0 = nothing to save */
static volatile uint32_t underruns, late;
static int64_t dsp_us;                       /* time spent demodulating, for the load report */
static volatile uint32_t listener;           /* IPv4 address getting the audio, 0 = none */
#define LISTEN_PORT   4994

/* ---- audio output ---- */

static int audio_init(void)
{
    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t bc = {
        .i2c_port = I2C_NUM_0, .sda_io_num = PIN_I2C_SDA, .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bc, &bus) != ESP_OK) return -1;
    /* the codec's CE pin picks its address: 0x18 or 0x19 */
    uint8_t addr = 0;
    for (uint8_t a = 0x18; a <= 0x19 && !addr; a++)
        if (i2c_master_probe(bus, a, 50) == ESP_OK) addr = a;
    if (!addr) { printf("@@RADIO no ES8311 on I2C (0x18, 0x19)\n"); return -1; }
    printf("@@RADIO ES8311 at 0x%02x\n", addr);

    /* full duplex when listening for voice commands; 80 ms of DMA each way */
    i2s_chan_handle_t tx, rx = NULL;
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    cc.auto_clear = true;
    cc.dma_desc_num = 8;
    cc.dma_frame_num = 480;
    if (i2s_new_channel(&cc, &tx, VOICE ? &rx : NULL) != ESP_OK) return -1;
    i2s_std_config_t sc = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = { .mclk = PIN_MCLK, .bclk = PIN_BCLK, .ws = PIN_WS, .dout = PIN_DOUT,
                      .din = VOICE ? PIN_DIN : I2S_GPIO_UNUSED },
    };
    if (i2s_channel_init_std_mode(tx, &sc) != ESP_OK) return -1;
    if (rx && i2s_channel_init_std_mode(rx, &sc) != ESP_OK) return -1;

    audio_codec_i2c_cfg_t ic = { .port = I2C_NUM_0, .addr = addr << 1, .bus_handle = bus };
    audio_codec_i2s_cfg_t dc = { .port = I2S_NUM_0, .tx_handle = tx, .rx_handle = rx };
    const audio_codec_ctrl_if_t *ctrl = audio_codec_new_i2c_ctrl(&ic);
    const audio_codec_data_if_t *data = audio_codec_new_i2s_data(&dc);
    if (!ctrl || !data) return -1;
    es8311_codec_cfg_t ec = {
        .ctrl_if = ctrl, .gpio_if = audio_codec_new_gpio(),
        .codec_mode = VOICE ? ESP_CODEC_DEV_WORK_MODE_BOTH : ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = PIN_PA, .use_mclk = true,
    };
    const audio_codec_if_t *es = es8311_codec_new(&ec);
    if (!es) return -1;
    esp_codec_dev_cfg_t dev = { .dev_type = VOICE ? ESP_CODEC_DEV_TYPE_IN_OUT : ESP_CODEC_DEV_TYPE_OUT,
                                .codec_if = es, .data_if = data };
    codec = esp_codec_dev_new(&dev);
    if (!codec) return -1;
    esp_codec_dev_sample_info_t fs = { .bits_per_sample = 16, .channel = 2, .sample_rate = AUDIO_RATE };
    if (esp_codec_dev_open(codec, &fs) != ESP_CODEC_DEV_OK) return -1;
    esp_codec_dev_set_out_vol(codec, volume);
    if (VOICE) esp_codec_dev_set_in_gain(codec, MIC_GAIN_DB);
    return 0;
}

int s31_radio_read_mic(void *buf, int bytes)
{
    return codec && VOICE ? esp_codec_dev_read(codec, buf, bytes) : -1;
}

/* ---- output: ducking and chimes, then the codec and any listeners ---- */

static int16_t out[2 * BLOCK_FRAMES];
static volatile bool ducked;
static volatile int chime_req;
static int chime, chime_pos;                 /* the chime playing, and how far in (frames) */

void s31_radio_duck(bool on) { ducked = on; }
void s31_radio_chime(int kind) { chime_req = kind; }

/* A chime's pitch at frame `pos`: two 90 ms notes (one for CHIME_DONE),
 * 0 when it is over. */
static float chime_hz(int kind, int pos)
{
    static const float notes[][2] = { [CHIME_LISTEN] = { 660, 990 }, [CHIME_DONE] = { 880, 0 }, [CHIME_MISSED] = { 660, 440 } };
    int note = pos / (AUDIO_RATE * 90 / 1000);
    return note < 2 ? notes[kind][note] : 0;
}

static void send_to_listener(void);

static void emit(void)
{
    if (ducked)
        for (int k = 0; k < 2 * BLOCK_FRAMES; k++) out[k] /= 5;
    if (chime_req) { chime = chime_req; chime_req = 0; chime_pos = 0; }
    for (int k = 0; chime && k < BLOCK_FRAMES; k++, chime_pos++) {
        float f = chime_hz(chime, chime_pos);
        if (!f) { chime = 0; break; }
        int in_note = chime_pos % (AUDIO_RATE * 90 / 1000);
        float env = in_note < 240 ? in_note / 240.0f : 1.0f;            /* 5 ms attack */
        int v = out[2 * k] + (int)(9000 * env * sinf(2 * PI * f * chime_pos / AUDIO_RATE));
        out[2 * k] = out[2 * k + 1] = v > 32767 ? 32767 : v < -32768 ? -32768 : v;
    }
    esp_codec_dev_write(codec, out, sizeof(out));
    send_to_listener();
#if CONFIG_S31SDR_RADIO_WIFI
    s31_web_audio(out, BLOCK_FRAMES);
#endif
}

static void play_silence(void)
{
    memset(out, 0, sizeof(out));
    emit();
}

/* ---- demodulation ---- */

/* Half-band filters keep only their non-zero taps, folded: tap pair p
 * weighs the samples 2p+1 either side of the centre, which weighs 1/2. */
#define HB1_PAIRS     ((HB1_TAPS + 1) / 4)
#define HB2_PAIRS     ((HB2_TAPS + 1) / 4)
static float hb1[HB1_PAIRS], hb2[HB2_PAIRS], aud_taps[AUD_TAPS / 2];
static float x1i[HB1_TAPS - 1 + CHUNK], x1q[HB1_TAPS - 1 + CHUNK];            /* history + one chunk */
static float x2i[HB2_TAPS - 1 + CHUNK / 2], x2q[HB2_TAPS - 1 + CHUNK / 2];
static float bi[CHUNK / 4], bq[CHUNK / 4];
static float ya[AUD_TAPS - 1 + CHUNK / 4];
static float prev_i, prev_q, deemph_y;
static float deemph, audio_gain;

static float bessel_i0(float x)
{
    float sum = 1, term = 1;
    for (int k = 1; k < 25; k++) {
        term *= (x / (2 * k)) * (x / (2 * k));
        sum += term;
    }
    return sum;
}

static float kaiser(int k, int n, float beta)
{
    float r = 2.0f * k / (n - 1) - 1;
    return bessel_i0(beta * sqrtf(1 - r * r)) / bessel_i0(beta);
}

/* A half-band low-pass (cut-off a quarter of the sample rate), Kaiser
 * window for about 60 dB, unity gain at DC. */
static void halfband_design(float *pairs, int n)
{
    int c = (n - 1) / 2, np = (n + 1) / 4;
    float sum = 0;
    for (int p = 0; p < np; p++) {
        int m = 2 * p + 1;
        pairs[p] = sinf(PI * m / 2) / (PI * m) * kaiser(c + m, n, 5.65f);
        sum += 2 * pairs[p];
    }
    for (int p = 0; p < np; p++) pairs[p] *= 0.5f / sum;
}

/* A symmetric windowed-sinc low-pass (Blackman), stored folded: the first
 * half of its n taps; cut-off as a fraction of the sample rate. */
static void lowpass_design(float *half, int n, float cutoff)
{
    float sum = 0;
    for (int k = 0; k < n / 2; k++) {
        float m = k - (n - 1) / 2.0f;
        float s = sinf(2 * PI * cutoff * m) / (PI * m);
        half[k] = s * (0.42f - 0.5f * cosf(2 * PI * k / (n - 1)) + 0.08f * cosf(4 * PI * k / (n - 1)));
        sum += 2 * half[k];
    }
    for (int k = 0; k < n / 2; k++) half[k] /= sum;
}

/* Half-band decimate-by-2: n_out outputs from x, which holds the filter's
 * history and then 2 * n_out new samples. Two sums, to keep the FPU busy. */
static IRAM_ATTR void halfband(const float *x, float *y, int n_out, const float *pairs, int np, int c)
{
    for (int j = 0; j < n_out; j++) {
        const float *p = x + 2 * j + c;
        float s0 = 0.5f * p[0], s1 = 0;
        int k = 0;
        for (; k + 1 < np; k += 2) {
            s0 += pairs[k] * (p[-(2 * k + 1)] + p[2 * k + 1]);
            s1 += pairs[k + 1] * (p[-(2 * k + 3)] + p[2 * k + 3]);
        }
        if (k < np) s0 += pairs[k] * (p[-(2 * k + 1)] + p[2 * k + 1]);
        y[j] = s0 + s1;
    }
}

static void dsp_reset(void)
{
    memset(x1i, 0, sizeof(x1i));
    memset(x1q, 0, sizeof(x1q));
    memset(x2i, 0, sizeof(x2i));
    memset(x2q, 0, sizeof(x2q));
    memset(ya, 0, sizeof(ya));
    prev_i = prev_q = deemph_y = 0;
}

/* atan2 to about 0.005 rad: plenty for a discriminator */
static IRAM_ATTR float fast_atan2(float y, float x)
{
    float ax = fabsf(x), ay = fabsf(y);
    if (ax < 1e-9f && ay < 1e-9f) return 0;
    float a = ax > ay ? ay / ax : ax / ay, s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    if (ay > ax) r = 1.57079637f - r;
    if (x < 0) r = PI - r;
    return y < 0 ? -r : r;
}

/* One chunk from the ring to CHUNK_FRAMES stereo frames at `dst`. */
static IRAM_ATTR void demod_chunk(int16_t *dst)
{
    uint32_t at = ring_tail % RING_N;
    float *ni = x1i + HB1_TAPS - 1, *nq = x1q + HB1_TAPS - 1;
    for (int k = 0; k < CHUNK; k++, at = (at + 1) % RING_N) {
        ni[k] = ring[2 * at] - 127.5f;
        nq[k] = ring[2 * at + 1] - 127.5f;
    }
    consume(CHUNK);

    halfband(x1i, x2i + HB2_TAPS - 1, CHUNK / 2, hb1, HB1_PAIRS, (HB1_TAPS - 1) / 2);
    halfband(x1q, x2q + HB2_TAPS - 1, CHUNK / 2, hb1, HB1_PAIRS, (HB1_TAPS - 1) / 2);
    halfband(x2i, bi, CHUNK / 4, hb2, HB2_PAIRS, (HB2_TAPS - 1) / 2);
    halfband(x2q, bq, CHUNK / 4, hb2, HB2_PAIRS, (HB2_TAPS - 1) / 2);
    memmove(x1i, x1i + CHUNK, (HB1_TAPS - 1) * sizeof(float));
    memmove(x1q, x1q + CHUNK, (HB1_TAPS - 1) * sizeof(float));
    memmove(x2i, x2i + CHUNK / 2, (HB2_TAPS - 1) * sizeof(float));
    memmove(x2q, x2q + CHUNK / 2, (HB2_TAPS - 1) * sizeof(float));

    float *y = ya + AUD_TAPS - 1;
    for (int j = 0; j < CHUNK / 4; j++) {
        /* phase step: the angle of x[n] * conj(x[n-1]) */
        float d = fast_atan2(bq[j] * prev_i - bi[j] * prev_q, bi[j] * prev_i + bq[j] * prev_q);
        prev_i = bi[j];
        prev_q = bq[j];
        deemph_y = d + deemph * (deemph_y - d);
        y[j] = deemph_y;
    }

    for (int j = 0; j < CHUNK_FRAMES; j++) {
        const float *p = ya + j * AUD_DECIM;
        float s0 = 0, s1 = 0;
        for (int k = 0; k < AUD_TAPS / 2; k += 2) {
            s0 += aud_taps[k] * (p[k] + p[AUD_TAPS - 1 - k]);
            s1 += aud_taps[k + 1] * (p[k + 1] + p[AUD_TAPS - 2 - k]);
        }
        float v = (s0 + s1) * audio_gain;
        int16_t s = v > 32767 ? 32767 : v < -32768 ? -32768 : (int16_t)v;
        dst[2 * j] = dst[2 * j + 1] = s;
    }
    memmove(ya, ya + CHUNK / 4, (AUD_TAPS - 1) * sizeof(float));
}

/* ---- tuning, scanning ---- */

/* Start over on new samples: after a retune, or a new dongle. */
static void restart(void)
{
    __atomic_store_n(&ring_tail, __atomic_load_n(&ring_head, __ATOMIC_ACQUIRE), __ATOMIC_RELEASE);
    skip = SETTLE;
    filling = true;
    dsp_reset();
}

static void save_soon(void) { save_at = esp_timer_get_time() + 5000000; }

static int tune(uint32_t hz)
{
    if (hz < TUNE_LOW_HZ || hz > TUNE_HIGH_HZ || !s31_rtl_tune(hz)) return -1;
    station_hz = hz;
    restart();
    save_soon();
    return 0;
}

static float *fft_re, *fft_im, *spec, *twc, *tws, *hann, *sorted;

static void fft(float *re, float *im)
{
    for (int i = 1, j = 0; i < NFFT; i++) {
        int bit = NFFT >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { float t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (int len = 2; len <= NFFT; len <<= 1) {
        int step = NFFT / len;
        for (int i = 0; i < NFFT; i += len)
            for (int k = 0; k < len / 2; k++) {
                float wr = twc[k * step], wi = -tws[k * step];
                int a = i + k, b = a + len / 2;
                float xr = re[b] * wr - im[b] * wi, xim = re[b] * wi + im[b] * wr;
                re[b] = re[a] - xr; im[b] = im[a] - xim;
                re[a] += xr; im[a] += xim;
            }
    }
}

static int cmp_float(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return x < y ? -1 : x > y;
}

/* Averaged power spectrum at `hz`, bin NFFT/2 = centre. */
static int window_spectrum(uint32_t hz)
{
    if (!s31_rtl_tune(hz)) { printf("@@RADIO scan: tuning to %lu Hz refused\n", (unsigned long)hz); return -1; }
    if (!wait_for(SETTLE)) { printf("@@RADIO scan: no samples\n"); return -1; }
    consume(backlog());                                 /* everything up to here predates the retune */
    if (!wait_for(SCAN_FFTS * NFFT)) { printf("@@RADIO scan: no samples\n"); return -1; }
    memset(spec, 0, NFFT * sizeof(float));
    for (int f = 0; f < SCAN_FFTS; f++) {
        uint32_t at = ring_tail % RING_N;
        for (int k = 0; k < NFFT; k++, at = (at + 1) % RING_N) {
            fft_re[k] = (ring[2 * at] - 127.5f) * hann[k];
            fft_im[k] = (ring[2 * at + 1] - 127.5f) * hann[k];
        }
        consume(NFFT);
        fft(fft_re, fft_im);
        for (int k = 0; k < NFFT; k++)
            spec[(k + NFFT / 2) % NFFT] += fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k];
    }
    return 0;
}

static int scan(void)
{
    if (!fft_re) {
        float **bufs[] = { &fft_re, &fft_im, &spec, &hann, &sorted, &twc, &tws };
        for (int b = 0; b < 7; b++) {
            *bufs[b] = heap_caps_malloc(NFFT * sizeof(float), MALLOC_CAP_INTERNAL);
            if (!*bufs[b]) { printf("@@RADIO scan: out of memory\n"); return -1; }
        }
        for (int k = 0; k < NFFT / 2; k++) {
            twc[k] = cosf(2 * PI * k / NFFT);
            tws[k] = sinf(2 * PI * k / NFFT);
        }
        for (int k = 0; k < NFFT; k++) hann[k] = 0.5f - 0.5f * cosf(2 * PI * k / NFFT);
    }
    const float bin_hz = (float)RTL_RATE / NFFT;
    const int half = (int)(75000 / bin_hz);             /* +-75 kHz around each channel */
    static float chan_db[N_CHAN];
    int64_t t0 = esp_timer_get_time();
    for (uint32_t lo = FM_LOW_HZ; lo <= FM_HIGH_HZ; lo += 4 * FM_STEP_HZ) {
        uint32_t centre = lo + 3 * FM_STEP_HZ / 2;      /* channels at -300, -100, +100, +300 kHz */
        if (window_spectrum(centre)) return -1;
        /* noise floor: the 20th percentile bin (most bins can be stations) */
        memcpy(sorted, spec, NFFT * sizeof(float));
        qsort(sorted, NFFT, sizeof(float), cmp_float);
        float noise = sorted[NFFT / 5] * (2 * half + 1);
        for (int c = 0; c < 4; c++) {
            uint32_t ch = lo + c * FM_STEP_HZ;
            if (ch > FM_HIGH_HZ) break;
            int mid = NFFT / 2 + (int)lrintf(((float)ch - centre) / bin_hz);
            float p = 0;
            for (int k = mid - half; k <= mid + half; k++) p += spec[k];
            chan_db[(ch - FM_LOW_HZ) / FM_STEP_HZ] = 10 * log10f(p / noise + 1e-12f);
        }
    }
    int n = 0;
    for (int c = 0; c < (int)N_CHAN; c++) {
        float l = c > 0 ? chan_db[c - 1] : -99, r = c + 1 < (int)N_CHAN ? chan_db[c + 1] : -99;
        if (chan_db[c] >= SCAN_SNR_DB && chan_db[c] >= l && chan_db[c] >= r)
            stations[n++] = FM_LOW_HZ + c * FM_STEP_HZ;
    }
    n_stations = n;
    printf("@@RADIO scan: %d stations in %lld ms\n", n, (long long)(esp_timer_get_time() - t0) / 1000);
    return 0;
}

static int seek(int dir)
{
    if (!n_stations && scan()) { tune(station_hz); return -1; }
    int n = n_stations;
    if (!n) { tune(station_hz); return -1; }
    uint32_t here = station_hz;
    for (int k = 0; k < n; k++) {
        uint32_t s = stations[dir > 0 ? k : n - 1 - k];
        if ((dir > 0 && s > here + FM_STEP_HZ / 2) || (dir < 0 && s + FM_STEP_HZ / 2 < here)) return tune(s);
    }
    return tune(stations[dir > 0 ? 0 : n - 1]);         /* wrap around the band */
}

static int handle(enum op op, int arg)
{
    if (op == OP_VOL) {
        volume = arg < 0 ? 0 : arg > 100 ? 100 : arg;
        esp_codec_dev_set_out_vol(codec, volume);
        save_soon();
        return 0;
    }
    if (op == OP_MUTE) {
        muted = arg;
        esp_codec_dev_set_out_mute(codec, muted);
        return 0;
    }
    if (op == OP_TONE) {                                /* 1 kHz at half scale for two seconds */
        for (int b = 0; b < 200; b++) {
            for (int k = 0; k < BLOCK_FRAMES; k++)
                out[2 * k] = out[2 * k + 1] = (int16_t)(16384 * sinf(2 * PI * 1000 * (b * BLOCK_FRAMES + k) / AUDIO_RATE));
            esp_codec_dev_write(codec, out, sizeof(out));
        }
        restart();
        return 0;
    }
    if (!live) return -1;
    if (op == OP_TUNE) return tune((uint32_t)arg);
    if (op == OP_SEEK) return seek(arg);
    int e = scan();                                     /* OP_SCAN */
    tune(station_hz);
    return e;
}

/* ---- settings ---- */

static void load_settings(void)
{
    nvs_handle_t h;
    if (nvs_open("radio", NVS_READONLY, &h) != ESP_OK) return;
    uint32_t hz;
    uint8_t vol;
    if (nvs_get_u32(h, "station", &hz) == ESP_OK && hz >= TUNE_LOW_HZ && hz <= TUNE_HIGH_HZ) station_hz = hz;
    if (nvs_get_u8(h, "volume", &vol) == ESP_OK && vol <= 100) volume = vol;
    nvs_close(h);
}

static void save_settings(void)
{
    nvs_handle_t h;
    if (nvs_open("radio", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, "station", station_hz);
    nvs_set_u8(h, "volume", (uint8_t)volume);
    nvs_commit(h);
    nvs_close(h);
}

/* ---- the radio ---- */

/* BOOT button, polled once per audio block: act on release. */
static void button(void)
{
    static int64_t down_at;
    bool down = gpio_get_level(PIN_BOOT) == 0;
    int64_t now = esp_timer_get_time();
    if (down && !down_at) down_at = now;
    if (!down && down_at) {
        int64_t ms = (now - down_at) / 1000;
        down_at = 0;
        if (ms >= 30 && live) {
            int dir = ms >= LONG_PRESS_MS ? -1 : +1;
            int e = seek(dir);
            printf("@@RADIO button: seek %s -> %s %.1f MHz\n", dir > 0 ? "up" : "down", e ? "failed," : "",
                   station_hz / 1e6);
        }
    }
}

/* The block just played, as mono PCM, to the computer that asked. */
static void send_to_listener(void)
{
    static int sock = -1;
    static int16_t mono[BLOCK_FRAMES];
    uint32_t to = listener;
    if (!to) return;
    if (sock < 0 && (sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) return;
    for (int k = 0; k < BLOCK_FRAMES; k++) mono[k] = out[2 * k];
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(LISTEN_PORT), .sin_addr.s_addr = to };
    sendto(sock, mono, sizeof(mono), 0, (struct sockaddr *)&a, sizeof(a));
}

static void radio_task(void *arg)
{
    for (;;) {
        button();
        struct req r;
        if (xQueueReceive(reqs, &r, 0) == pdTRUE) {
            *r.result = handle(r.op, r.arg);
            xTaskNotifyGive(r.from);
        }
        if (save_at && esp_timer_get_time() > save_at) {
            save_at = 0;
            save_settings();
        }

        if (!s31_rtl_present()) {
            live = false;
            play_silence();
            continue;
        }
        if (!live) {                                    /* a dongle (again): it starts on station_hz */
            live = true;
            restart();
        }
        uint32_t have = backlog();
        if (skip) {
            uint32_t k = have < skip ? have : skip;
            consume(k);
            skip -= k;
            play_silence();
            continue;
        }
        if (filling) {
            if (have < FILL) { play_silence(); continue; }
            filling = false;
        }
        if (have > TOO_LATE) {                          /* the dongle's clock is ahead of the codec's */
            consume(have - FILL);
            late++;
        } else if (have < BLOCK_CHUNKS * CHUNK) {       /* behind: wait for a full FILL again */
            underruns++;
            filling = true;
            play_silence();
            continue;
        }
        int64_t t0 = esp_timer_get_time();
        for (int c = 0; c < BLOCK_CHUNKS; c++) demod_chunk(out + 2 * c * CHUNK_FRAMES);
        dsp_us += esp_timer_get_time() - t0;
        emit();
    }
}

static const char *usage =
    "radio on | off | f <MHz> | seek up | seek down | scan | vol <0-100> | vol up | vol down | tone | listen [off] | status; rtl ...";

bool s31_radio_command(const char *cmd, char *reply, size_t n)
{
    enum op op = OP_MUTE;
    int arg = 0;
    bool query = false;
    double v;
    if (strncmp(cmd, "on", 2) == 0) op = OP_MUTE;
    else if (strncmp(cmd, "off", 3) == 0) { op = OP_MUTE; arg = 1; }
    else if (sscanf(cmd, "f %lf", &v) == 1) {
        op = OP_TUNE;
        v = v < 1e4 ? v * 1e6 : v;                       /* MHz or Hz */
        arg = v > 0 && v < 2e9 ? (int)lrint(v) : 0;
    }
    else if (strncmp(cmd, "seek up", 7) == 0) { op = OP_SEEK; arg = +1; }
    else if (strncmp(cmd, "seek down", 9) == 0) { op = OP_SEEK; arg = -1; }
    else if (strncmp(cmd, "scan", 4) == 0) op = OP_SCAN;
    else if (strncmp(cmd, "tone", 4) == 0) op = OP_TONE;
    else if (strncmp(cmd, "vol up", 6) == 0) { op = OP_VOL; arg = volume + 10 > 100 ? 100 : volume + 10; }
    else if (strncmp(cmd, "vol down", 8) == 0) { op = OP_VOL; arg = volume - 10 < 0 ? 0 : volume - 10; }
    else if (sscanf(cmd, "vol %lf", &v) == 1) { op = OP_VOL; arg = v < 0 ? 0 : v > 100 ? 100 : (int)v; }
    else if (strncmp(cmd, "status", 6) == 0) query = true;
    else return false;

    /* hand it to radio_task and wait: every request ends (a scan within
     * seconds), and `e` must outlive it */
    int e = 0;
    if (!query) {
        struct req r = { .op = op, .arg = arg, .from = xTaskGetCurrentTaskHandle(), .result = &e };
        ulTaskNotifyTake(pdTRUE, 0);
        if (xQueueSend(reqs, &r, pdMS_TO_TICKS(100)) != pdTRUE) {
            snprintf(reply, n, "busy");
            return true;
        }
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    int k = snprintf(reply, n, "%s%s, %.1f MHz, volume %d, %d stations known", e ? "failed: " : "",
                     !live ? "no RTL-SDR" : muted ? "muted" : "playing", station_hz / 1e6, volume, n_stations);
    if (!query && op == OP_SCAN && !e)
        for (int i = 0; i < n_stations && k < (int)n - 8; i++)
            k += snprintf(reply + k, n - k, "%s%.1f", i ? " " : ": ", stations[i] / 1e6);
    return true;
}

int s31_radio_status_json(char *buf, size_t n)
{
    int k = snprintf(buf, n, "{\"live\":%s,\"muted\":%s,\"listening\":%s,\"mhz\":%.1f,\"vol\":%d,\"stations\":[",
                     live ? "true" : "false", muted ? "true" : "false", ducked ? "true" : "false",
                     station_hz / 1e6, volume);
    int ns = n_stations;
    for (int i = 0; i < ns && k < (int)n - 12; i++) k += snprintf(buf + k, n - k, "%s%.1f", i ? "," : "", stations[i] / 1e6);
    if (k < (int)n - 2) k += snprintf(buf + k, n - k, "]}");
    return k;
}

/* Text commands on UDP 4990 when an Ethernet cable is plugged in. */
static void control_task(void *arg)
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in me = { .sin_family = AF_INET, .sin_port = htons(4990), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (s < 0 || bind(s, (struct sockaddr *)&me, sizeof(me)) != 0) {
        printf("@@RADIO control port 4990 unavailable\n");
        vTaskDelete(NULL);
    }
    static char buf[128], reply[512];
    for (;;) {
        struct sockaddr_in from;
        socklen_t fl = sizeof(from);
        int len = recvfrom(s, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &fl);
        if (len <= 0) continue;
        buf[len] = 0;
        buf[strcspn(buf, "\r\n")] = 0;
        const char *c = strncmp(buf, "radio ", 6) == 0 ? buf + 6 : buf;
        if (strncmp(c, "listen", 6) == 0) {
            listener = strstr(c, "off") ? 0 : from.sin_addr.s_addr;
            snprintf(reply, sizeof(reply), listener ? "audio to this computer on UDP %d (48 kHz, 16-bit mono)"
                                                    : "audio to the speaker only", LISTEN_PORT);
            sendto(s, reply, strlen(reply), 0, (struct sockaddr *)&from, fl);
            continue;
        }
        bool ok = strncmp(c, "rtl ", 4) == 0 ? s31_rtl_command(c + 4, reply, sizeof(reply))
                                             : s31_radio_command(c, reply, sizeof(reply));
        if (!ok) snprintf(reply, sizeof(reply), "%s", usage);
        sendto(s, reply, strlen(reply), 0, (struct sockaddr *)&from, fl);
    }
}

/* Every five seconds: what the radio is doing, on the console. */
static void report_task(void *arg)
{
    int64_t last = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        int64_t d = dsp_us - last;
        last = dsp_us;
        printf("@@RADIO %s %.1f MHz, volume %d, backlog %lu ms, underruns %lu, skips %lu, overflows %lu, "
               "demodulation %lld%% of core 1\n",
               !live ? "waiting for an RTL-SDR," : muted ? "muted" : "playing", station_hz / 1e6, volume,
               (unsigned long)(backlog() / MS(1)), (unsigned long)underruns, (unsigned long)late,
               (unsigned long)ring_overflows, (long long)(d / 50000));
    }
}

void s31_radio_start(void)
{
    load_settings();
    s31_rtl_preset(station_hz, RTL_RATE);
    ring = heap_caps_malloc(2 * RING_N, MALLOC_CAP_SPIRAM);
    reqs = xQueueCreate(1, sizeof(struct req));
    halfband_design(hb1, HB1_TAPS);
    halfband_design(hb2, HB2_TAPS);
    lowpass_design(aud_taps, AUD_TAPS, 16000.0f / IF_RATE);
    deemph = expf(-1.0f / (IF_RATE * 75e-6f));
    audio_gain = 0.5f * 32767.0f / (2 * PI * 75000.0f / IF_RATE);   /* 75 kHz deviation to half scale */
    gpio_config_t g = { .pin_bit_mask = 1ULL << PIN_BOOT, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&g);
    if (!ring || !reqs || audio_init()) {
        printf("@@RADIO audio start failed\n");
        return;
    }
    printf("@@RADIO ready: %.1f MHz, volume %d\n", station_hz / 1e6, volume);
    s31_spawn(radio_task, "radio", 4096, NULL, 5, NULL, 1);
    s31_spawn(control_task, "radioctl", 3072, NULL, 4, NULL, 0);
    s31_spawn(report_task, "radiorep", 2560, NULL, 1, NULL, 0);
}
