/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * Continuous I/Q over Gigabit Ethernet as VITA-49 (VRT): CI8 at 40 MS/s, or
 * 10-bit I/Q as CI16 at 20 MS/s (s31_stream_main's `bits`).
 *
 * 10-bit: the LCD_CAM camera input captures I[1:0]/Q[1:0] next to PARLIO.
 * The camera unit drives a 20 MHz clock (PLL_F160M / 8) onto GPIO49 and both
 * units sample on that pad, so they latch the same bus sample and cannot
 * drift; the rings' fixed offset is measured once from a dump shot both rings
 * record, and the merge is checked bit-exact before streaming. 20 MHz takes
 * every 4th sample of the 80 MS/s bus with no filter, so the receive filter
 * edge near +-11 MHz folds slightly. Packing is a PIE byte zip of the two
 * rings (pack_zip), about 8% of core 1.
 *
 * Capture: modem diag[19:0] carries 10-bit I/Q. PARLIO lanes 0-7 take
 * Q[9:2], lanes 8-15 take I[9:2]: each 16-bit word is one CI8 pair, and
 * putting Q first undoes the engine's spectral inversion for free (swapping
 * I and Q mirrors the spectrum). For CI8, PARLIO runs at 40 MHz
 * (every other sample of the 80 MS/s bus) into a DMA ring; the start is
 * calibrated against dump-engine shots.
 *
 * Transport: VRT packets sent as raw Ethernet frames (lwIP is far too slow at
 * ~55k packets/s), to the host at S31_GW (local_net.h), UDP port VRT_PORT.
 *   data:    type 1 (IF data with stream ID), TSF sample count, 720 CI8 or
 *            360 CI16 pairs
 *   context: type 4, CIF0 = bandwidth, RF reference, sample rate, data format
 * Field encodings follow SoapyVITA49 (VRT.cpp appendCIF0/parsePayloadFormat).
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/parlio_rx.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include "esp_ipc_isr.h"
#include "esp_timer.h"
#include "esp_cpu.h"
#include "esp_eth.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/etharp.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "esp_private/regi2c_ctrl.h"
#include "soc/ahb_dma_struct.h"
#include "soc/axi_dma_struct.h"
#include "s31_dump.h"
#include "cam_lsb.h"
#include "dma_pos.h"
#include "net.h"
#include "local_net.h"

extern void adctrig(int count, int mode, int a2, int a3, int a4,
                    int a5, int a6, int a7, int s9, int s10);

/* 1: also build the bench-debugging extras: a DMA callback trace, and the
 * raw register / capture test commands on the text port (see control_task).
 * They are unauthenticated, so they are off by default. */
#ifndef S31_DEBUG_CMDS
#define S31_DEBUG_CMDS 0
#endif

#define VRT_PORT      4991
#define STREAM_ID     0x53310001u           /* "S1" + 1 */
#define DIAG_EN_REG   0x20586028u
/* The driver splits the ring into 4092-byte DMA nodes (largest 4-byte-aligned
 * size). The EOF interrupt must land on node boundaries, or the callbacks
 * report partial nodes and the write position drifts: EOF every NODE bytes,
 * ring = a whole number of nodes. */
#define NODE          4092
#define RING_BYTES    (16 * NODE)
#define PAYLOAD       1440                  /* 720 CI8 or 360 CI16 pairs */
#define VRT_HDR       16                    /* word0, SID, 64-bit sample count */
#define FRAME_LEN     (14 + 20 + 8 + VRT_HDR + PAYLOAD)
/* Calibration copies only the window that must hold the dump shot (just
 * behind the DMA write position), into internal RAM: copying a whole ring
 * into PSRAM was slow enough for the DMA to overwrite the shot first. */
#define WIN           16384                         /* samples */
#define DUMP_N        511
#define CMP_N         250
#define WINDOW_LO     0x2F060000u
#define WINDOW_HI     0x2F080000u

static const int pads[16] = { 0, 1, 2, 3, 21, 23, 25, 35, 36, 37, 38, 39, 40, 42, 43, 44 };
/* 10-bit: the camera input runs 16 bits wide and holds the I/Q low bits
 * where the CI16 items need them: each sample's half-word is Q[1:0] << 6,
 * then I[1:0] << 6, the second bytes of the two big-endian items. Pads 45-48
 * carry I0, I1, Q0, Q1 (diag 0, 1, 10, 11) into lanes 14, 15, 6, 7. A payload
 * is then the two rings byte-interleaved (pack_zip). */
#define PCLK_PAD      49
#define C_NODE        2048                          /* bytes */
#define C_NODES       16
#define C_RING        (C_NODE * C_NODES / 2)        /* samples */
#define P_SAMPLES     (RING_BYTES / 2)
static const int c_pads[4] = { 45, 46, 47, 48 };
static const int c_diag[4] = { 0, 1, 10, 11 };
static const int c_lanes[4] = { 14, 15, 6, 7 };
static int bits10;                                  /* 1: 10-bit CI16 at 20 MS/s */
/* capture rate vs the 80 MS/s bus: CI8 captures at 40 MHz (every 2nd bus
 * sample); 10-bit captures at 20 MHz (every 4th), so packing reads
 * contiguous samples and the DMA writes half as much. */
static int step = 2, cmp_n = CMP_N;
static double rate_hz = 40e6;
static uint16_t *c_ring, *c_snap;
static int64_t cam_off;                             /* CAM sample = PARLIO sample + cam_off */
/* lane -> diag line: Q[9:2] = diag 12..19 on lanes 0-7, I[9:2] = diag 2..9 on 8-15 */
static const int lane_diag[16] = { 12, 13, 14, 15, 16, 17, 18, 19, 2, 3, 4, 5, 6, 7, 8, 9 };

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t *const bank = (volatile uint32_t *)S31_DUMP_SRAM;
static uint8_t *ring, *snap;
static uint32_t dump_copy[DUMP_N];

static struct dma_pos ppos;                 /* PARLIO ring write position (dma_pos.h) */
static TaskHandle_t sender;
static esp_eth_handle_t eth;

static uint64_t written_now(void) { return dma_pos_written(&ppos); }

static int diag_sig(int n) { return n < 8 ? 118 + n : n < 16 ? 204 + (n - 8) : 154 + (n - 16); }

#if S31_DEBUG_CMDS
/* first callbacks, for checking the driver's DMA node layout */
#define NDBG 40
static volatile uint32_t dbg_off[NDBG], dbg_len[NDBG], dbg_n;

/* Paired software/hardware trace of the RX DMA: per callback, the driver's
 * node (from e->data), the node of the hardware's last completed
 * descriptor, and that descriptor's word 0 (size bits 0-11, received length
 * bits 12-23). tr_start keeps the first 64 callbacks of the latest start,
 * tr_roll the latest 256. */
struct trace_rec { uint32_t cyc; uint16_t sw, hw; uint32_t dw0; };
static struct trace_rec tr_start[64], tr_roll[256];
static volatile uint32_t tr_start_n, tr_roll_n;

static IRAM_ATTR bool in_sram(uint32_t a) { return a >= 0x2F000000u && a < 0x2F080000u; }
#endif

static bool IRAM_ATTR on_partial(parlio_rx_unit_handle_t u, const parlio_rx_event_data_t *e, void *arg)
{
#if S31_DEBUG_CMDS
    {
        struct trace_rec t = { .cyc = esp_cpu_get_cycle_count(), .hw = 0xFFFF, .dw0 = 0 };
        t.sw = (uint16_t)(((uintptr_t)e->data - (uintptr_t)ring) / NODE);
        if (ppos.eof_reg) {
            uint32_t d = *ppos.eof_reg;
            if (in_sram(d)) {
                t.dw0 = ((volatile uint32_t *)d)[0];
                t.hw = (uint16_t)((((volatile uint32_t *)d)[1] - (uint32_t)(uintptr_t)ring) / NODE);
            }
        }
        if (tr_start_n < 64) tr_start[tr_start_n++] = t;
        tr_roll[tr_roll_n++ & 255] = t;
    }
    if (dbg_n < NDBG) { dbg_off[dbg_n] = (uintptr_t)e->data - (uintptr_t)ring; dbg_len[dbg_n] = e->recv_bytes; dbg_n++; }
#endif
    /* The write position comes from the hardware's last completed
     * descriptor, not from e->data: after a calibration restart one stale
     * interrupt can put the driver's node index a node ahead of the DMA for
     * good (driver node = hardware node + 1 on every callback).
     * dma_pos_update() also skips stale repeats and recovers laps a
     * long interrupt stall hid. */
    if (!dma_pos_update(&ppos)) return false;
    BaseType_t woken = pdFALSE;
    if (sender) vTaskNotifyGiveFromISR(sender, &woken);
    return woken == pdTRUE;
}

static void *alloc_outside_window(size_t bytes)
{
    void *held[8]; int nh = 0;
    for (;;) {
        void *p = heap_caps_aligned_alloc(64, bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!p) break;
        uintptr_t a = (uintptr_t)p;
        if (a + bytes <= WINDOW_LO || a >= WINDOW_HI) {
            while (nh) heap_caps_free(held[--nh]);
            return p;
        }
        if (nh == 8) { heap_caps_free(p); break; }
        held[nh++] = p;
    }
    while (nh) heap_caps_free(held[--nh]);
    return NULL;
}

static void own_dump(int count)
{
    taskENTER_CRITICAL(&mux);
    esp_ipc_isr_stall_other_cpu();
    S31_REG(S31_TCM_DUMP_OWNER) = 0xFF000000u;
    uint32_t c = S31_REG(S31_DUMP_CTRL);
    c &= ~(S31_DUMP_CTRL_COUNT_M | S31_DUMP_CTRL_ABORT | S31_DUMP_CTRL_TRIG | S31_DUMP_CTRL_ENABLE);
    c |= (uint32_t)count;
    S31_REG(S31_DUMP_CTRL) = c | S31_DUMP_CTRL_ENABLE;
    S31_REG(S31_DUMP_CTRL) = c | S31_DUMP_CTRL_ENABLE | S31_DUMP_CTRL_TRIG;
    S31_REG(S31_DUMP_CTRL) = c | S31_DUMP_CTRL_ENABLE;
    for (int i = 0; i < 200000 && !(S31_REG(S31_DUMP_CTRL) & S31_DUMP_CTRL_DONE); i++) {}
    S31_REG(S31_DUMP_CTRL) = c;
    S31_REG(S31_TCM_DUMP_OWNER) = 0;
    esp_ipc_isr_release_other_cpu();
    taskEXIT_CRITICAL(&mux);
}

/* ring word as the lanes are mapped: Q8 low byte, I8 high byte */
static uint16_t dump_word(uint32_t d)
{
    return (uint16_t)(((d >> 12) & 0xFF) | (((d >> 2) & 0xFF) << 8));
}

static int64_t pos_mod(int64_t a, int64_t m) { a %= m; return a < 0 ? a + m : a; }

/* Copy WIN samples of a ring (elem bytes each, ring of rsamp samples) ending
 * `ahead` samples past absolute position `head`; returns the window's
 * absolute start. */
static int64_t copy_window(uint8_t *dst, const uint8_t *src, int elem, int64_t rsamp, int64_t head, int ahead)
{
    int64_t start = head + ahead - WIN;
    int64_t at = pos_mod(start, rsamp);
    int64_t first = rsamp - at < WIN ? rsamp - at : WIN;
    memcpy(dst, src + at * elem, first * elem);
    if (first < WIN) memcpy(dst + first * elem, src, (WIN - first) * elem);
    return start;
}

static int verify(void)
{
    own_dump(DUMP_N);
    esp_rom_delay_us(60);
    copy_window(snap, ring, 2, RING_BYTES / 2, (int64_t)(written_now() / 2), 4096);
    for (int j = 0; j < DUMP_N; j++) dump_copy[j] = bank[j];
    const uint16_t *s = (const uint16_t *)snap;
    int words = WIN, best = -1, bk = 0, bp = 0;
    for (int p = 0; p < step && best < 32; p++)
        for (int k = 0; k < words; k++) {
            int m = 0;
            for (int j = 0; j < 32; j++) m += s[(k + j) % words] == dump_word(dump_copy[p + step * j]);
            if (m > best) { best = m; bk = k; bp = p; }
            if (m == 32) break;
        }
    int exact = 0;
    for (int j = 0; j < cmp_n; j++) exact += s[(bk + j) % words] == dump_word(dump_copy[bp + step * j]);
    return exact;
}

/* ---- VRT packet building (big-endian words) ---- */
static void be32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static void be64(uint8_t *p, uint64_t v) { be32(p, v >> 32); be32(p + 4, (uint32_t)v); }
static uint64_t fix20(double v) { return (uint64_t)(int64_t)(v * (double)(1 << 20)); }

static uint16_t ip_csum(const uint8_t *h)
{
    uint32_t sum = 0;
    for (int i = 0; i < 20; i += 2) sum += (h[i] << 8) | h[i + 1];
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)~sum;
}

/* Ethernet + IPv4 + UDP header for a frame carrying `vrt_len` bytes of VRT. */
static void frame_header(uint8_t *f, const uint8_t dst_mac[6], const uint8_t src_mac[6], int vrt_len)
{
    memcpy(f, dst_mac, 6); memcpy(f + 6, src_mac, 6);
    f[12] = 0x08; f[13] = 0x00;
    uint8_t *ip = f + 14, *udp = f + 34;
    uint32_t src = esp_ip4addr_aton(S31_IP), dst = esp_ip4addr_aton(S31_GW);
    int iplen = 20 + 8 + vrt_len;
    memset(ip, 0, 20);
    ip[0] = 0x45; ip[2] = iplen >> 8; ip[3] = iplen & 0xFF; ip[6] = 0x40; ip[8] = 64; ip[9] = 17;
    memcpy(ip + 12, &src, 4); memcpy(ip + 16, &dst, 4);
    uint16_t c = ip_csum(ip); ip[10] = c >> 8; ip[11] = c & 0xFF;
    udp[0] = VRT_PORT >> 8; udp[1] = VRT_PORT & 0xFF; udp[2] = VRT_PORT >> 8; udp[3] = VRT_PORT & 0xFF;
    udp[4] = (8 + vrt_len) >> 8; udp[5] = (8 + vrt_len) & 0xFF; udp[6] = udp[7] = 0;
}

/* The LO frequency: set by the control tasks and the monitor on core 0,
 * read by the sender on core 1. A double is two words on this CPU, so every
 * access goes through lo_get / lo_put and never sees half an update. */
static double lo_hz;
static portMUX_TYPE lo_mux = portMUX_INITIALIZER_UNLOCKED;
static double lo_get(void) { taskENTER_CRITICAL(&lo_mux); double f = lo_hz; taskEXIT_CRITICAL(&lo_mux); return f; }
static void lo_put(double f) { taskENTER_CRITICAL(&lo_mux); lo_hz = f; taskEXIT_CRITICAL(&lo_mux); }
static volatile int ctx_now;           /* set to send a context packet immediately */

/* ---- Receive gain ----
 * The Wi-Fi AGC's gain index (the dump word's bits 20-27) can be forced:
 * AGC register 0x2010702C holds it in bits 24-31 and bit 23 forces it
 * (librftest's force_rx_gain, Wi-Fi path: write the index with bit 23
 * clear, wait, then set bit 23). gain_idx < 0 leaves the AGC in charge.
 * The index is not calibrated in dB: about 0.35 dB per step on the noise
 * floor from 20 to 50, steeper above; above ~80 the output clips. */
#define AGC_FORCE_REG 0x2010702Cu
static int gain_idx = -1;
static int manual_idx = 50;            /* index restored when the AGC is switched off */

static void apply_gain(void)
{
    uint32_t r = REG_READ(AGC_FORCE_REG) & ~(1u << 23);
    if (gain_idx < 0) { REG_WRITE(AGC_FORCE_REG, r); return; }
    r = (r & 0x007FFFFFu) | ((uint32_t)gain_idx << 24);
    REG_WRITE(AGC_FORCE_REG, r);
    esp_rom_delay_us(5);
    REG_WRITE(AGC_FORCE_REG, r | (1u << 23));
}

static void set_gain(int idx)
{
    gain_idx = idx < 0 ? -1 : idx > 127 ? 127 : idx;
    if (gain_idx >= 0) manual_idx = gain_idx;
    apply_gain();
    ctx_now = 1;
}

/* ---- RF synthesizer ----
 * SDM block 0x63 registers 3-6 hold the LO as a 27-bit word W:
 * LO = 30 MHz * (32 + W / 2^19); r3 = W[26:19], r4 = W[18:11], r5 = W[10:3],
 * r6 = W[2:0]. The PHY writes it on every channel change. */
#define I2C_SDM 0x63
static uint32_t sdm_word(double f) { return (uint32_t)((f / 30e6 - 32.0) * 524288.0 + 0.5); }
static double sdm_hz(uint32_t w) { return 30e6 * (32.0 + w / 524288.0); }
static uint32_t sdm_read(void)
{
    return ((uint32_t)regi2c_ctrl_read_reg(I2C_SDM, 0, 3) << 19) | ((uint32_t)regi2c_ctrl_read_reg(I2C_SDM, 0, 4) << 11) |
           ((uint32_t)regi2c_ctrl_read_reg(I2C_SDM, 0, 5) << 3) | (regi2c_ctrl_read_reg(I2C_SDM, 0, 6) & 7);
}

static void sdm_write(uint32_t w)
{
    regi2c_ctrl_write_reg(I2C_SDM, 0, 3, (uint8_t)(w >> 19));
    regi2c_ctrl_write_reg(I2C_SDM, 0, 4, (uint8_t)(w >> 11));
    regi2c_ctrl_write_reg(I2C_SDM, 0, 5, (uint8_t)(w >> 3));
    regi2c_ctrl_write_reg(I2C_SDM, 0, 6, (uint8_t)(w & 7));
}

/* RFPLL block 0x62: the VCO capacitor code is 9 bits, r1 = cap[7:0] and
 * r2 bit 6 = cap[8]; the PHY sets it directly (r11 bit 6, manual mode, is
 * already on). r12 bits 2 and 3 flag the loop out of lock: bit 2 = cap too
 * small for the frequency, bit 3 = too large. */
#define I2C_RFPLL 0x62
static int pll_cap, pll_first, pll_len;

static void cap_write(int cap)
{
    regi2c_ctrl_write_reg(I2C_RFPLL, 0, 1, (uint8_t)cap);
    uint8_t r2 = regi2c_ctrl_read_reg(I2C_RFPLL, 0, 2);
    regi2c_ctrl_write_reg(I2C_RFPLL, 0, 2, (uint8_t)((r2 & ~0x40) | ((cap >> 8) & 1) << 6));
}

static bool pll_locked(void) { return ((regi2c_ctrl_read_reg(I2C_RFPLL, 0, 12) >> 2) & 3) == 0; }

/* Scan all 512 codes and keep the middle of the longest locked run. */
static bool cap_scan(void)
{
    int start = 0, len = 0, best_start = 0, best_len = 0;
    for (int cap = 0; cap < 512; cap++) {
        cap_write(cap);
        esp_rom_delay_us(20);
        if (!pll_locked()) { len = 0; continue; }
        if (len++ == 0) start = cap;
        if (len > best_len) { best_start = start; best_len = len; }
    }
    if (!best_len) return false;
    pll_first = best_start; pll_len = best_len;
    pll_cap = best_start + (best_len - 1) / 2;
    cap_write(pll_cap);
    esp_rom_delay_us(20);
    return pll_locked();
}

/* Channel width the PHY is set up for, which also sets the analog receive
 * filter: 20 (default) or 40 MHz (HT40, secondary channel above for
 * channels 1-7, below for 8-13). */
static int chan_mhz = 20;

static esp_err_t wifi_chan(int ch)
{
    wifi_second_chan_t sec = chan_mhz == 40 ? (ch <= 7 ? WIFI_SECOND_CHAN_ABOVE : WIFI_SECOND_CHAN_BELOW)
                                            : WIFI_SECOND_CHAN_NONE;
    return esp_wifi_set_channel(ch, sec);
}

static int tune_hz(double f, double *actual);

/* Receive filter width: the PHY's channel width, 20 or 40 MHz. Measured at
 * 40 MS/s (gain index 60, median spectra): 20 MHz mode is flat to about
 * +-10 MHz and 18 dB down at +-15; 40 MHz mode stays within about 3 dB out
 * to +-20 MHz. The driver refuses 40 MHz while 802.11ax is enabled, and
 * nothing here uses ax, so 40 MHz runs with 11b/g/n. Returns 0, or -1 with
 * the previous width kept. */
static int set_width(int mhz)
{
    static uint8_t proto0;
    if (!proto0) esp_wifi_get_protocol(WIFI_IF_STA, &proto0);
    int prev = chan_mhz;
    esp_wifi_set_protocol(WIFI_IF_STA, mhz == 40 ? (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N)
                                                 : proto0);
    if (esp_wifi_set_bandwidth(WIFI_IF_STA, mhz == 40 ? WIFI_BW40 : WIFI_BW20) != ESP_OK) {
        esp_wifi_set_protocol(WIFI_IF_STA, prev == 40 ? (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N)
                                                      : proto0);
        return -1;
    }
    chan_mhz = mhz;
    double act;
    tune_hz(lo_get(), &act);                 /* re-set the channel with the new width */
    ctx_now = 1;
    return 0;
}

/* Set the LO to f: the nearest Wi-Fi channel (1-13) sets up the rest of the
 * receiver, then the synthesizer word and VCO capacitor are set for f. */
static bool set_lo(double f)
{
    int ch = (int)floor((f - 2407e6) / 5e6 + 0.5);
    if (ch < 1) ch = 1;
    if (ch > 13) ch = 13;
    if (wifi_chan(ch) != ESP_OK) return false;
    uint32_t w = sdm_word(f);
    sdm_write(w);
    if (!cap_scan()) return false;
    lo_put(sdm_hz(w));
    apply_gain();
    ctx_now = 1;
    return true;
}

/* Tune to f (Hz). Requests outside TUNE_MIN_HZ..TUNE_MAX_HZ are refused
 * without touching the receiver; if the VCO does not lock, the previous
 * frequency is restored. Returns 0, or -1 with the receiver unchanged. */
#define TUNE_MIN_HZ 2150e6
#define TUNE_MAX_HZ 2850e6
static int tune_hz(double f, double *actual)
{
    int ret = -1;
    if (f >= TUNE_MIN_HZ && f <= TUNE_MAX_HZ) {
        double prev = lo_get();
        if (set_lo(f)) ret = 0;
        else set_lo(prev);
    }
    *actual = lo_get();
    return ret;
}

/* Control: UDP port VRT_PORT - 1, text commands:
 *   f <Hz>          tune exactly, 2150-2850 MHz; "ok <LO Hz> cap ..." or
 *                   "refused, still <LO Hz>" (receiver unchanged)
 *   ch <1..13>      tune to a Wi-Fi channel centre; "ok <LO MHz>"
 *   gain <0..127>   force the AGC gain index; "gain auto" hands it back
 *   bw <20|40>      receive filter width
 *   status          LO, lock, gain, monitor counters
 *   agc             gain index range over one 511-sample dump shot
 * Anything else replies "err". Built with S31_DEBUG_CMDS 1, also:
 *   rd <blk> <reg>  read an analog (regi2c) register, hex
 *   wr <blk> <reg> <val>  write one (can detune the receiver)
 *   rm <addr> / wm <addr> <val>  read / write a 32-bit register
 *   ring / trace    dump the raw capture ring / DMA callback trace
 *   stall <us>      hold the capture core, to test ring-lap recovery
 *   check           the calibration's bit-exact test on the running stream
 */
/* One owner of the radio settings at a time: the text and VITA 49.2
 * controls and the monitor. */
static SemaphoreHandle_t radio_mutex;

/* Radio monitor: the Wi-Fi PHY's own tracking could rewrite the synthesizer
 * or drop the forced gain, and the VCO could drift out of lock. Every
 * 500 ms: restore our synthesizer word, rescan the capacitor if unlocked,
 * re-force the gain; count each. */
static volatile uint32_t mon_sdm, mon_relock, mon_gain, mon_checks;

static void monitor_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        xSemaphoreTake(radio_mutex, portMAX_DELAY);
        mon_checks++;
        double f = lo_get();
        if (sdm_read() != sdm_word(f)) {
            mon_sdm++;
            set_lo(f);
        } else if (!pll_locked()) {
            mon_relock++;
            if (!cap_scan()) set_lo(f);
        }
        if (gain_idx >= 0) {
            uint32_t r = REG_READ(AGC_FORCE_REG);
            if (!(r & (1u << 23)) || (int)(r >> 24) != gain_idx) { mon_gain++; apply_gain(); }
        }
        xSemaphoreGive(radio_mutex);
    }
}

static void control_task(void *arg)
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in me = { .sin_family = AF_INET, .sin_port = htons(VRT_PORT - 1),
                              .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (s < 0 || bind(s, (struct sockaddr *)&me, sizeof(me)) != 0) {
        printf("stream: text control port %d unavailable\n", VRT_PORT - 1);
        vTaskDelete(NULL);
    }
    char buf[64];
    for (;;) {
        struct sockaddr_in from; socklen_t fl = sizeof(from);
        int n = recvfrom(s, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &fl);
        if (n <= 0) continue;
        buf[n] = 0;
        xSemaphoreTake(radio_mutex, portMAX_DELAY);
        int ch = 0;
#if S31_DEBUG_CMDS
        unsigned blk, reg, val;
        unsigned addr;
#endif
        double f;
        char reply[256] = "err";
        if (sscanf(buf, "f %lf", &f) == 1) {
            double act;
            int64_t t0 = esp_timer_get_time();
            int c = tune_hz(f, &act);
            int ms = (int)((esp_timer_get_time() - t0) / 1000);
            if (c == 0) snprintf(reply, sizeof(reply), "ok %.3f cap %d run %d-%d %dms", act, pll_cap, pll_first, pll_first + pll_len - 1, ms);
            else snprintf(reply, sizeof(reply), "refused, still %.3f", act);
            printf("stream: tune %.6f MHz -> %s\n", f / 1e6, reply);
        } else if (sscanf(buf, "bw %d", &ch) == 1 && (ch == 20 || ch == 40)) {
            int t = set_width(ch);
            snprintf(reply, sizeof(reply), "%s %d MHz, LO %.3f", t == 0 ? "ok" : "failed, now", chan_mhz, lo_get() / 1e6);
        } else if (strncmp(buf, "status", 6) == 0) {
            snprintf(reply, sizeof(reply), "LO %.3f %s, cap %d, gain %d, monitor: %" PRIu32 " checks, synth restored %"
                     PRIu32 ", relocked %" PRIu32 ", gain re-forced %" PRIu32 ", DMA laps %" PRIu32 "/%" PRIu32,
                     lo_get(), pll_locked() ? "locked" : "UNLOCKED", pll_cap, gain_idx, mon_checks, mon_sdm, mon_relock,
                     mon_gain, ppos.laps, bits10 ? cam_lsb_laps() : 0);
        } else if (strncmp(buf, "gain auto", 9) == 0) {
            set_gain(-1);
            snprintf(reply, sizeof(reply), "ok auto");
        } else if (sscanf(buf, "gain %d", &ch) == 1) {
            set_gain(ch);
            snprintf(reply, sizeof(reply), "ok %d", gain_idx);
        } else if (strncmp(buf, "agc", 3) == 0) {
            /* one dump shot: the per-sample AGC gain index (word bits 20-27) */
            own_dump(DUMP_N);
            int lo = 255, hi = 0, hist[256] = {0}, mode = 0;
            for (int j = 0; j < DUMP_N; j++) {
                int g = (bank[j] >> 20) & 0xFF;
                if (g < lo) lo = g;
                if (g > hi) hi = g;
                if (++hist[g] > hist[mode]) mode = g;
            }
            snprintf(reply, sizeof(reply), "gain %d-%d, mostly %d (%d/%d)", lo, hi, mode, hist[mode], DUMP_N);
        } else if (sscanf(buf, "ch %d", &ch) == 1 && ch >= 1 && ch <= 13) {
            /* exact tune to the channel centre: in 40 MHz mode the PHY's own
             * LO would sit 10 MHz off, at the 40 MHz channel's centre */
            double act;
            if (tune_hz((2407.0 + 5.0 * ch) * 1e6, &act) == 0)
                snprintf(reply, sizeof(reply), "ok %.0f", act / 1e6);
#if S31_DEBUG_CMDS
        } else if (strncmp(buf, "ring", 4) == 0) {
            /* raw capture ring (before packetizing): WIN samples ending at the
             * DMA position, sent back as 32 datagrams of 1024 bytes, each
             * prefixed with its index; then the usual reply */
            int64_t start = copy_window(snap, ring, 2, P_SAMPLES, (int64_t)(written_now() / 2), 0);
            static uint8_t pkt[4 + 1024];
            for (int k = 0; k < 32; k++) {
                pkt[0] = (uint8_t)k; pkt[1] = pkt[2] = pkt[3] = 0;
                memcpy(pkt + 4, snap + k * 1024, 1024);
                sendto(s, pkt, sizeof(pkt), 0, (struct sockaddr *)&from, fl);
            }
            snprintf(reply, sizeof(reply), "ring %lld", (long long)start);
        } else if (strncmp(buf, "trace", 5) == 0) {
            /* tr_start (first 64 callbacks of the last start), then the
             * latest 256: 12-byte records, little-endian, 4 datagrams */
            static struct trace_rec cp[256];
            uint32_t n0 = tr_start_n, nr = tr_roll_n;
            for (int j = 0; j < 256; j++) cp[j] = tr_roll[(nr + j) & 255];      /* oldest first */
            static uint8_t pkt[4 + 1024];
            pkt[0] = 0xF0; pkt[1] = (uint8_t)n0; pkt[2] = pkt[3] = 0;
            memcpy(pkt + 4, tr_start, sizeof(tr_start));
            sendto(s, pkt, 4 + sizeof(tr_start), 0, (struct sockaddr *)&from, fl);
            for (int k = 0; k < 4; k++) {                  /* 64 records per datagram */
                pkt[0] = (uint8_t)k; pkt[1] = pkt[2] = pkt[3] = 0;
                memcpy(pkt + 4, (uint8_t *)cp + k * 768, 768);
                sendto(s, pkt, 4 + 768, 0, (struct sockaddr *)&from, fl);
            }
            snprintf(reply, sizeof(reply), "trace %" PRIu32 " start records, %" PRIu32 " total, dma reg %p",
                     n0, nr, (void *)ppos.eof_reg);
        } else if (sscanf(buf, "stall %d", &ch) == 1 && ch > 0 && ch <= 5000) {
            /* test: hold the capture core (its DMA interrupts) for ch us,
             * long enough to hide whole ring laps */
            uint32_t p0 = ppos.laps, c0 = cam_lsb_laps();
            taskENTER_CRITICAL(&mux);
            esp_ipc_isr_stall_other_cpu();
            esp_rom_delay_us(ch);
            esp_ipc_isr_release_other_cpu();
            taskEXIT_CRITICAL(&mux);
            vTaskDelay(pdMS_TO_TICKS(5));
            snprintf(reply, sizeof(reply), "stalled %d us, laps recovered: PARLIO %" PRIu32 ", camera %" PRIu32,
                     ch, ppos.laps - p0, cam_lsb_laps() - c0);
        } else if (strncmp(buf, "check", 5) == 0) {
            /* the calibration's bit-exact test, on the running stream:
             * PARLIO ring against one dump shot */
            int n = 0, worst = cmp_n;
            for (int k = 0; k < 8; k++) { int e = verify(); n += e == cmp_n; if (e < worst) worst = e; }
            snprintf(reply, sizeof(reply), "%d/8 shots bit-exact, worst %d/%d", n, worst, cmp_n);
        } else if (sscanf(buf, "rm %x", &addr) == 1 && addr >= 0x20000000u && addr < 0x30000000u && !(addr & 3)) {
            snprintf(reply, sizeof(reply), "%08" PRIx32, REG_READ(addr));
        } else if (sscanf(buf, "wm %x %x", &addr, &val) == 2 && addr >= 0x20000000u && addr < 0x30000000u && !(addr & 3)) {
            REG_WRITE(addr, val);
            snprintf(reply, sizeof(reply), "%08" PRIx32, REG_READ(addr));
        } else if (sscanf(buf, "rd %x %x", &blk, &reg) == 2) {
            snprintf(reply, sizeof(reply), "%02x", regi2c_ctrl_read_reg(blk, 0, reg));
        } else if (sscanf(buf, "wr %x %x %x", &blk, &reg, &val) == 3) {
            regi2c_ctrl_write_reg(blk, 0, reg, val);
            snprintf(reply, sizeof(reply), "%02x", regi2c_ctrl_read_reg(blk, 0, reg));
#endif
        }
        xSemaphoreGive(radio_mutex);
        sendto(s, reply, strlen(reply), 0, (struct sockaddr *)&from, fl);
    }
}

/* gain field: stage 1 in the low half, signed, 1/128 units; -1 = AGC */
static uint32_t gain_word(void) { return (uint16_t)(int16_t)(gain_idx * 128); }

/* State/Event field: only the AGC/MGC indicator is reported */
static uint32_t se_word(void) { return (1u << 28) | (gain_idx < 0 ? (1u << 16) : 0); }

/* data payload format word: CI8 (8/8) or CI16 (16/16), complex, signed fixed */
static uint32_t fmt_word(void)
{
    return bits10 ? (1u << 29) | (15u << 6) | 15u : (1u << 29) | (7u << 6) | 7u;
}

static int build_context(uint8_t *vrt, int count, uint64_t sample)
{
    int w = 0;
    uint8_t *p = vrt;
    /* type 4 (IF context), TSI none, TSF sample count (1) */
    be32(p + 4, STREAM_ID); be64(p + 8, sample); w = 4;
    be32(p + 4 * w, (1u << 29) | (1u << 27) | (1u << 23) | (1u << 21) | (1u << 16) | (1u << 15)); w++;  /* CIF0 */
    be64(p + 4 * w, fix20(chan_mhz * 1e6)); w += 2;    /* bandwidth: receive filter */
    be64(p + 4 * w, fix20(lo_get())); w += 2;          /* RF reference frequency */
    be32(p + 4 * w, gain_word()); w++;                 /* gain: stage 1 = index, -1 = AGC */
    be64(p + 4 * w, fix20(rate_hz)); w += 2;           /* sample rate */
    be32(p + 4 * w, se_word()); w++;                   /* State/Event: AGC or manual */
    be32(p + 4 * w, fmt_word()); w++;                  /* payload format */
    be32(p + 4 * w, 0); w++;
    be32(p, (0x4u << 28) | (1u << 20) | ((uint32_t)(count & 0xF) << 16) | (uint32_t)w);
    return 4 * w;
}

/* ---- 10-bit: align the camera ring to the PARLIO ring ---- */
static int last_ppar, last_cpar;
static uint16_t c_expect(uint32_t d) { return (uint16_t)((((d >> 10) & 3) << 6) | ((d & 3) << 14)); }

static int find_p(const uint16_t *r, int *pos, int *par)
{
    int best = -1;
    for (int p = 0; p < step && best < 64; p++)
        for (int k = 0; k + cmp_n < WIN && best < 64; k++) {
            int m = 0;
            for (int j = 0; j < 64; j++) m += r[k + j] == dump_word(dump_copy[p + step * j]);
            if (m > best) { best = m; *pos = k; *par = p; }
        }
    return best;
}

static int find_c(const uint16_t *r, int *pos, int *par)
{
    int best = -1;
    for (int p = 0; p < step && best < 64; p++)
        for (int k = 0; k + cmp_n < WIN && best < 64; k++) {
            int m = 0;
            for (int j = 0; j < 64; j++) m += r[k + j] == c_expect(dump_copy[p + step * j]);
            if (m > best) { best = m; *pos = k; *par = p; }
        }
    return best;
}

/* One dump shot recorded by both rings: measure cam_off from the rings'
 * absolute counters and check the merge. Returns merged exact count. */
static int cam_align(void)
{
    own_dump(DUMP_N);
    esp_rom_delay_us(60);
    int64_t pstart = copy_window(snap, ring, 2, P_SAMPLES, (int64_t)(written_now() / 2), 4096);
    int64_t cstart = copy_window((uint8_t *)c_snap, (const uint8_t *)c_ring, 2, C_RING, (int64_t)cam_lsb_written() / 2, 4096);
    for (int j = 0; j < DUMP_N; j++) dump_copy[j] = bank[j];
    int pp = 0, ppar = 0, cp = 0, cpar = 0;
    if (find_p((const uint16_t *)snap, &pp, &ppar) < 64 || find_c(c_snap, &cp, &cpar) < 64) return -1;
    last_ppar = ppar; last_cpar = cpar;
    if (ppar != cpar) return -2;                    /* rings saw different bus samples */
    cam_off = (cstart + cp) - (pstart + pp);
    const uint16_t *pr = (const uint16_t *)snap;
    int merged = 0;
    for (int j = 0; j < cmp_n; j++) {
        uint32_t d = dump_copy[ppar + step * j];
        uint16_t w = pr[pp + j];
        uint16_t lo = c_snap[cp + j];
        int q10 = ((w & 0xFF) << 2) | ((lo >> 6) & 3), i10 = ((w >> 8) << 2) | (lo >> 14);
        merged += i10 == (int)(d & 0x3FF) && q10 == (int)((d >> 10) & 0x3FF);
    }
    return merged;
}

/* 10-bit payload: n8 x 8 samples. PIE: one 128-bit load from each ring
 * (8 PARLIO half-words Q[9:2],I[9:2]; 8 camera half-words Q[1:0]<<6,I[1:0]<<6),
 * byte zip, two stores: Q10<<6 then I10<<6 as big-endian 16-bit items.
 * All three pointers 16-byte aligned. */
static void pack_zip(uint32_t *o, const uint16_t *p, const uint16_t *c, int n8)
{
    /* PIE address operands must be in x8-x15 (a2-a4 = x12-x14) */
    register uint32_t *ro __asm__("a2") = o;
    register const uint16_t *rp __asm__("a3") = p, *rc __asm__("a4") = c;
    for (int k = 0; k < n8; k++)
        __asm__ volatile(
            "esp.vld.128.ip q0, %0, 16\n"
            "esp.vld.128.ip q1, %1, 16\n"
            "esp.vzip.8 q0, q1\n"
            "esp.vst.128.ip q0, %2, 16\n"
            "esp.vst.128.ip q1, %2, 16\n"
            : "+r"(rp), "+r"(rc), "+r"(ro) : : "memory");
}

static void pack_ref(uint32_t *o, const uint16_t *p, const uint16_t *c, int n)
{
    for (int k = 0; k < n; k++)
        o[k] = (p[k] & 0xFFu) | ((c[k] & 0xFFu) << 8) | ((uint32_t)(p[k] >> 8) << 16) | ((uint32_t)(c[k] >> 8) << 24);
}

static bool pack_selftest(void)
{
    static uint16_t p[64] __attribute__((aligned(16))), c[64] __attribute__((aligned(16)));
    static uint32_t a[64] __attribute__((aligned(16))), b[64];
    uint32_t x = 0x12345678;
    for (int k = 0; k < 64; k++) {
        x = x * 1664525u + 1013904223u; p[k] = x >> 16;
        x = x * 1664525u + 1013904223u; c[k] = x >> 16;
    }
    pack_zip(a, p, c, 8);
    pack_ref(b, p, c, 64);
    return memcmp(a, b, sizeof(a)) == 0;
}

static bool learn_host_mac(uint8_t mac[6])
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) return false;
    struct sockaddr_in to = { .sin_family = AF_INET, .sin_port = htons(VRT_PORT + 1) };
    to.sin_addr.s_addr = esp_ip4addr_aton(S31_GW);
    struct netif *nif = esp_netif_get_netif_impl(esp_netif_get_handle_from_ifkey("ETH_DEF"));
    ip4_addr_t host; host.addr = to.sin_addr.s_addr;
    for (int t = 0; t < 50; t++) {
        sendto(s, "s31", 3, 0, (struct sockaddr *)&to, sizeof(to));   /* triggers ARP */
        vTaskDelay(pdMS_TO_TICKS(100));
        struct eth_addr *ea; const ip4_addr_t *ip_ret;
        if (nif && etharp_find_addr(nif, &host, &ea, &ip_ret) >= 0) {
            memcpy(mac, ea->addr, 6);
            close(s);
            return true;
        }
    }
    close(s);
    return false;
}

struct stats {
    uint64_t sent, lost, lost_total, retries, tx_fail, ctx_fail, cyc_pack, cyc_tx;
    int64_t max_gap, max_lag;
    int skips;
    esp_err_t last_err;
};
static QueueHandle_t stats_q;

static void sender_task(void *arg)
{
    /* payload at frame + 58: offset the buffer by 6 so it is 16-byte aligned
     * (the 10-bit packer stores 128 bits at a time) */
    static uint32_t frame_buf[(1514 + 6 + 3) / 4] __attribute__((aligned(16)));
    uint8_t *frame = (uint8_t *)frame_buf + 6;
    static uint8_t cframe[256];
    uint8_t my_mac[6], host_mac[6];
    esp_eth_ioctl(eth, ETH_CMD_G_MAC_ADDR, my_mac);
    if (!learn_host_mac(host_mac)) { printf("stream: no ARP reply from the host\n"); vTaskDelete(NULL); }
    printf("stream: host MAC learned, sending VRT to port %d\n", VRT_PORT);
    frame_header(frame, host_mac, my_mac, VRT_HDR + PAYLOAD);

    uint64_t rd = written_now(), sample = 0, lost = 0, sent = 0, retries = 0, tx_fail = 0, ctx_fail = 0;
    int64_t n = 0;                          /* 10-bit: next PARLIO sample index */
    uint64_t cyc_pack = 0, cyc_tx = 0, cyc_pack0 = 0, cyc_tx0 = 0;
    esp_err_t last_err = ESP_OK;
#if S31_DEBUG_CMDS
    printf("stream: first DMA nodes (offset/len):");
    for (int i = 0; i < NDBG && i < (int)dbg_n; i++) printf(" %" PRIu32 "/%" PRIu32, dbg_off[i], dbg_len[i]);
    printf("\n");
#endif
    rd = written_now();                 /* start from "now", not from before ARP */
    if (bits10) {
        int64_t pw = (int64_t)(written_now() / 2), cw = (int64_t)cam_lsb_written() / 2 - cam_off;
        n = ((pw < cw ? pw : cw) - 4096) & ~(int64_t)7;   /* 8-sample (16-byte) steps */
    }
    int dcount = 0, ccount = 0;
    int64_t next_ctx = 0, next_stat = esp_timer_get_time() + 1000000;
    uint64_t sent0 = 0, lost0 = 0;
    int64_t prev = esp_timer_get_time(), max_gap = 0, max_lag = 0; int skips = 0;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        int64_t now = esp_timer_get_time();
        if (now - prev > max_gap) max_gap = now - prev;
        prev = now;
        if (now >= next_ctx || ctx_now) {
            ctx_now = 0;
            int clen = build_context(cframe + 42, ccount++, sample);
            frame_header(cframe, host_mac, my_mac, clen);
            /* esp_eth_transmit fails while every transmit descriptor is in
             * use; retry until one frees (here and for data frames) */
            int ct = 0;
            while (esp_eth_transmit(eth, cframe, 42 + clen) != ESP_OK && ++ct < 20000) {}
            if (ct >= 20000) ctx_fail++;
            next_ctx = now + 100000;
        }
        if (bits10) {
            int64_t pw = (int64_t)(written_now() / 2), cw = (int64_t)cam_lsb_written() / 2 - cam_off;
            int64_t head = pw < cw ? pw : cw;           /* both rings have data up to here */
            if (head - n > max_lag) max_lag = head - n;
            if (pw - n > P_SAMPLES - 4096 || cw - n > C_RING - 4096) {   /* fell behind */
                int64_t nn = (head - 8192) & ~(int64_t)7;
                skips++;
                lost += nn - n; sample += nn - n; n = nn;
            }
            while (head - n >= PAYLOAD / 4) {
                if ((int64_t)(written_now() / 2) - n > P_SAMPLES - 4096 ||
                    (int64_t)cam_lsb_written() / 2 - cam_off - n > C_RING - 4096)
                    break;                              /* lapped: the skip above handles it */
                uint8_t *v = frame + 42;
                be32(v, (0x1u << 28) | (1u << 20) | ((uint32_t)(dcount++ & 0xF) << 16) |
                        (uint32_t)((VRT_HDR + PAYLOAD) / 4));
                be32(v + 4, STREAM_ID);
                be64(v + 8, sample);
                /* 360 CI16 pairs: item 1 = Q, item 2 = I (Q first undoes the
                 * spectral inversion), big-endian 10-bit << 6 */
                const uint16_t *pr = (const uint16_t *)ring;
                uint32_t pi = (uint32_t)pos_mod(n, P_SAMPLES), ci = (uint32_t)pos_mod(n + cam_off, C_RING);
                uint32_t *d32 = (uint32_t *)(v + VRT_HDR);
                uint32_t c0 = esp_cpu_get_cycle_count();
                for (int k = 0, total = PAYLOAD / 4; k < total; ) {     /* runs end at ring wraps */
                    int run = total - k;
                    int pl = (int)(P_SAMPLES - pi), cl = (int)(C_RING - ci);
                    if (run > pl) run = pl;
                    if (run > cl) run = cl;
                    pack_zip(d32 + k, pr + pi, c_ring + ci, run / 8);
                    k += run;
                    pi += run; if (pi >= P_SAMPLES) pi -= P_SAMPLES;
                    ci = (ci + run) & (C_RING - 1);
                }
                uint32_t c1 = esp_cpu_get_cycle_count();
                cyc_pack += c1 - c0;
                int tries = 0;
                esp_err_t te;
                while ((te = esp_eth_transmit(eth, frame, FRAME_LEN)) != ESP_OK && ++tries < 20000) {}
                if (te != ESP_OK) { tx_fail++; last_err = te; }
                retries += tries;
                cyc_tx += esp_cpu_get_cycle_count() - c1;
                n += PAYLOAD / 4; sample += PAYLOAD / 4; sent++;
            }
            goto stats;
        }
        for (;;) {
            /* checked before every packet: a transmit stall inside this
             * loop must not let the DMA lap the reader unnoticed */
            uint64_t wr = written_now();
            if (wr - rd > RING_BYTES - 8192) {        /* fell behind: skip ahead */
                uint64_t skip = (wr - rd) - (RING_BYTES / 2);
                skip -= skip % 2;
                rd += skip; lost += skip / 2; sample += skip / 2;
            }
            if (wr - rd < PAYLOAD) break;
            uint8_t *v = frame + 42;
            be32(v, (0x1u << 28) | (1u << 20) | ((uint32_t)(dcount++ & 0xF) << 16) |
                    (uint32_t)((VRT_HDR + PAYLOAD) / 4));
            be32(v + 4, STREAM_ID);
            be64(v + 8, sample);
            uint32_t off = rd % RING_BYTES, first = RING_BYTES - off;
            if (first >= PAYLOAD) memcpy(v + VRT_HDR, ring + off, PAYLOAD);
            else { memcpy(v + VRT_HDR, ring + off, first); memcpy(v + VRT_HDR + first, ring, PAYLOAD - first); }
            int tries = 0;
            esp_err_t te;
            while ((te = esp_eth_transmit(eth, frame, FRAME_LEN)) != ESP_OK && ++tries < 20000) {}
            if (te != ESP_OK) { tx_fail++; last_err = te; }
            retries += tries;
            rd += PAYLOAD; sample += PAYLOAD / 2; sent++;
        }
    stats:
        if (now >= next_stat) {
            /* hand the numbers to stats_task: printing here blocks on the
             * UART for milliseconds, longer than the camera ring lasts */
            struct stats st = {
                .sent = sent - sent0, .lost = lost - lost0, .lost_total = lost, .retries = retries,
                .tx_fail = tx_fail, .ctx_fail = ctx_fail, .last_err = last_err,
                .cyc_pack = cyc_pack - cyc_pack0, .cyc_tx = cyc_tx - cyc_tx0,
                .max_gap = max_gap, .max_lag = max_lag, .skips = skips,
            };
            xQueueOverwrite(stats_q, &st);
            max_gap = max_lag = 0; skips = 0;
            cyc_pack0 = cyc_pack; cyc_tx0 = cyc_tx;
            sent0 = sent; lost0 = lost; next_stat = now + 1000000;
        }
    }
}

static void stats_task(void *arg)
{
    struct stats st;
    for (;;) {
        if (xQueueReceive(stats_q, &st, portMAX_DELAY) != pdTRUE) continue;
        printf("@@STREAM %.1f kpkt/s  %.2f MS/s  lost %" PRIu64 " (total %" PRIu64 ")"
               "  tx retries %" PRIu64 " fails %" PRIu64 " ctx fails %" PRIu64 " last %s\n",
               st.sent / 1000.0, st.sent * (bits10 ? PAYLOAD / 4 : PAYLOAD / 2) / 1e6,
               st.lost, st.lost_total, st.retries, st.tx_fail, st.ctx_fail, esp_err_to_name(st.last_err));
        if (mon_sdm || mon_relock || mon_gain)
            printf("@@RADIO synth restored %" PRIu32 ", relocked %" PRIu32 ", gain re-forced %" PRIu32 " (%" PRIu32 " checks)\n",
                   mon_sdm, mon_relock, mon_gain, mon_checks);
        if (ppos.laps || cam_lsb_laps())
            printf("@@DMA laps recovered after interrupt stalls: PARLIO %" PRIu32 ", camera %" PRIu32 "\n",
                   ppos.laps, cam_lsb_laps());
        if (bits10)
            printf("@@CPU core1: packing %.0f%%, transmit %.0f%% of 320 MHz; longest wait %lld us, "
                   "deepest backlog %lld samples, skips %d\n",
                   st.cyc_pack / 3.2e6, st.cyc_tx / 3.2e6, (long long)st.max_gap, (long long)st.max_lag, st.skips);
    }
}

/* ---- VITA 49.2 controllee (SoapyVITA49 Control.cpp is the reference) ----
 * Command packets (type 6/7) arrive on VRT_PORT; replies go out from the same
 * port, because the controller's socket is connected to it. Layout after the
 * header and stream ID: CAM, message ID, controllee ID, [controller ID], then
 * CIF0 and fields encoded as in a context packet. An acknowledgement is the
 * same packet type with header bit 26 set. */
#define CAM_CONTROLLEE_EN  (1u << 31)
#define CAM_CONTROLLEE_UID (1u << 30)
#define CAM_CONTROLLER_EN  (1u << 29)
#define CAM_CONTROLLER_UID (1u << 28)
#define CAM_NACK_ONLY      (1u << 22)
#define CAM_REQ_VALIDATION (1u << 20)
#define CAM_REQ_EXECUTION  (1u << 19)
#define CAM_REQ_QUERY      (1u << 18)
#define CAM_ACK_PARTIAL    (1u << 11)
#define CAM_ACK_EXECUTED   (1u << 10)
#define CIF0_BW   (1u << 29)
#define CIF0_IF   (1u << 28)
#define CIF0_RF   (1u << 27)
#define CIF0_GAIN (1u << 23)
#define CIF0_RATE (1u << 21)
#define CIF0_SE   (1u << 16)          /* State/Event: AGC/MGC enable 28, indicator 16 */
#define CIF0_FMT  (1u << 15)

static uint32_t rd_be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }
static double rd_fix20(const uint8_t *p)
{
    return (double)(int64_t)((uint64_t)rd_be32(p) << 32 | rd_be32(p + 4)) / (double)(1 << 20);
}

/* Field sizes in words for CIF0 bits 31..0 (VITA 49.0 table 9.1-1). */
static int cif0_words(int bit)
{
    switch (bit) {
    case 29: case 28: case 27: case 26: case 25: return 2;   /* bw, IF, RF, RF off, IF off */
    case 24: case 23: case 22: return 1;                     /* ref level, gain, over-range */
    case 21: case 20: return 2;                               /* sample rate, ts adjust */
    case 19: case 18: return 1;                               /* ts cal time, temperature */
    case 17: return 2;                                        /* device ID */
    case 16: return 1;                                        /* state/event */
    case 15: return 2;                                        /* data format */
    default: return -1;                                       /* not handled */
    }
}


static void vrt_control_task(void *arg)
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in me = { .sin_family = AF_INET, .sin_port = htons(VRT_PORT),
                              .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (s < 0 || bind(s, (struct sockaddr *)&me, sizeof(me)) != 0) {
        printf("stream: VITA 49.2 control port %d unavailable\n", VRT_PORT);
        vTaskDelete(NULL);
    }
    static uint8_t in[512], out[256];
    uint32_t acount = 0;
    for (;;) {
        struct sockaddr_in from; socklen_t fl = sizeof(from);
        int n = recvfrom(s, in, sizeof(in), 0, (struct sockaddr *)&from, &fl);
        if (n < 16) continue;
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
        double rf = 0, rate = 0, bw = 0;
        bool have_rf = false, have_rate = false, have_bw = false, have_gain = false;
        double gain = 0;
        bool have_agc = false, agc_on = false;
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
            xSemaphoreTake(radio_mutex, portMAX_DELAY);
            if (have_rf) {
                double act;
                if (tune_hz(rf, &act) < 0) bad = true;
                else if (fabs(act - rf) > 1e3) partial = true;
                printf("vrt49.2: tune %.6f MHz -> %s, LO %.6f MHz\n", rf / 1e6, bad ? "refused" : "ok", act / 1e6);
            }
            if (have_rate && fabs(rate - rate_hz) > 1) partial = true;   /* fixed per mode */
            if (have_bw) {                                               /* filter: 20 or 40 MHz */
                if (set_width(bw > 20e6 ? 40 : 20) < 0) bad = true;
                else if (fabs(bw - chan_mhz * 1e6) > 1) partial = true;
            }
            if (have_agc) set_gain(agc_on ? -1 : manual_idx);            /* AGC/MGC */
            if (have_gain) {                                             /* gain index; < 0 = AGC */
                set_gain(gain < 0 ? -1 : (int)floor(gain + 0.5));
                if (gain >= 0 && fabs(gain - gain_idx) > 0.01) partial = true;
            }
            xSemaphoreGive(radio_mutex);
        }

        bool want_ack = query || (cam & (CAM_REQ_EXECUTION | CAM_REQ_VALIDATION));
        if ((cam & CAM_NACK_ONLY) && !bad && !partial) want_ack = false;
        if (!want_ack) continue;

        /* acknowledgement */
        int o = 0;
        uint32_t acam = (cam & (CAM_CONTROLLEE_EN | CAM_CONTROLLEE_UID | CAM_CONTROLLER_EN | CAM_CONTROLLER_UID));
        if (query) acam |= CAM_REQ_QUERY;
        else acam |= (cam & (CAM_REQ_EXECUTION | CAM_REQ_VALIDATION)) | CAM_ACK_EXECUTED;
        if (partial) acam |= CAM_ACK_PARTIAL;
        if (bad) acam |= (1u << 16);                   /* errors */
        be32(out + 4, rd_be32(in + 4));                /* stream ID */
        o = 8;
        be32(out + o, acam); o += 4;
        be32(out + o, msg); o += 4;
        memcpy(out + o, in + 4 * id0, 4 * ids); o += 4 * ids;
        if (query && !bad) {
            uint32_t want = cif0 & (CIF0_BW | CIF0_RF | CIF0_GAIN | CIF0_RATE | CIF0_SE | CIF0_FMT);
            be32(out + o, want); o += 4;
            if (want & CIF0_BW)   { be64(out + o, fix20(chan_mhz * 1e6)); o += 8; }
            if (want & CIF0_RF)   { be64(out + o, fix20(lo_get())); o += 8; }
            if (want & CIF0_GAIN) { be32(out + o, gain_word()); o += 4; }
            if (want & CIF0_RATE) { be64(out + o, fix20(rate_hz)); o += 8; }
            if (want & CIF0_SE)   { be32(out + o, se_word()); o += 4; }
            if (want & CIF0_FMT)  { be32(out + o, fmt_word()); be32(out + o + 4, 0); o += 8; }
        }
        be32(out, (type << 28) | (1u << 26) | ((acount++ & 0xF) << 16) | (uint32_t)(o / 4));
        sendto(s, out, o, 0, (struct sockaddr *)&from, fl);
        printf("vrt49.2: %s msg %" PRIu32 " -> ack%s%s\n", query ? "query" : "set", msg,
               partial ? " partial" : "", bad ? " ERROR" : "");
    }
}

void s31_stream_main(int channel, int bits)
{
    bits10 = bits == 10;
    rate_hz = bits10 ? 20e6 : 40e6;
    step = bits10 ? 4 : 2;
    cmp_n = bits10 ? 120 : CMP_N;               /* (DUMP_N - 3) / 4 = 127 at stride 4 */
    printf("\n== s31-sdr: VITA-49 %s stream, %.0f MS/s ==\n", bits10 ? "CI16 (10-bit)" : "CI8",
           rate_hz / 1e6);
    lo_put((2407.0 + 5.0 * channel) * 1e6);
    taskENTER_CRITICAL(&mux);
    esp_ipc_isr_stall_other_cpu();
    adctrig(1023, 0, 0, 1, 0, 0, 0, 0, 0, 0);       /* lane select: diag = dump word */
    esp_ipc_isr_release_other_cpu();
    taskEXIT_CRITICAL(&mux);

    /* CI8 at 40 MS/s gets the 40 MHz filter; 10-bit at 20 MS/s keeps 20 MHz,
     * the only band limit ahead of its 4:1 subsampling. Set before the
     * calibration, so the bit-exact check runs in the final mode. */
    if (!bits10) printf("stream: 40 MHz filter %s\n", set_width(40) == 0 ? "on" : "REFUSED");

    ring = alloc_outside_window(RING_BYTES);
    if (ring) dma_pos_init(&ppos, ring, RING_BYTES, NODE, bits10 ? 40 : 80);   /* bytes per us */
    snap = heap_caps_malloc(WIN * 2, MALLOC_CAP_INTERNAL);       /* calibration windows */
    if (bits10) {
        c_ring = alloc_outside_window(C_RING * 2);
        c_snap = heap_caps_malloc(WIN * 2, MALLOC_CAP_INTERNAL);
    }
    if (!ring || !snap || (bits10 && (!c_ring || !c_snap))) { printf("stream: buffer alloc failed\n"); return; }

    /* 10-bit: the camera unit makes the 20 MHz sample clock on PCLK_PAD and
     * both units sample on that pad, so they latch the same bus sample.
     * (PARLIO's own clock looped back to the camera arrives one 80 MHz bus
     * sample late, which an inverted PCLK cancels only at 40 MHz.) */
    if (bits10) {
        bool ok = pack_selftest();
        printf("stream: vector packer self-test %s\n", ok ? "ok" : "FAILED");
        if (!ok) return;
        ESP_ERROR_CHECK(cam_lsb_init(PCLK_PAD, c_pads, c_lanes, 16, (uint8_t *)c_ring, C_NODE, C_NODES, 8, 40));
    }
    parlio_rx_unit_config_t ucfg = {
        .trans_queue_depth = 1, .max_recv_size = RING_BYTES, .dma_burst_size = 16,
        .data_width = 16, .clk_src = bits10 ? PARLIO_CLK_SRC_EXTERNAL : PARLIO_CLK_SRC_PLL_F160M,
        .ext_clk_freq_hz = bits10 ? (uint32_t)rate_hz : 0,
        .exp_clk_freq_hz = (uint32_t)rate_hz,
        .clk_in_gpio_num = bits10 ? PCLK_PAD : -1, .clk_out_gpio_num = -1, .valid_gpio_num = -1,
        .flags.free_clk = 1,
    };
    for (int k = 0; k < 16; k++) ucfg.data_gpio_nums[k] = pads[k];
    parlio_rx_unit_handle_t rx;
    ESP_ERROR_CHECK(parlio_new_rx_unit(&ucfg, &rx));
    parlio_rx_event_callbacks_t cbs = { .on_partial_receive = on_partial };
    ESP_ERROR_CHECK(parlio_rx_unit_register_event_callbacks(rx, &cbs, NULL));
    parlio_rx_soft_delimiter_config_t dcfg = {
        .sample_edge = PARLIO_SAMPLE_EDGE_POS, .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = NODE, .timeout_ticks = 0,
    };
    parlio_rx_delimiter_handle_t deli;
    ESP_ERROR_CHECK(parlio_new_rx_soft_delimiter(&dcfg, &deli));

    REG_WRITE(DIAG_EN_REG, bits10 ? 0x000FFFFFu : 0x000FFFFCu);
    int good = 0, starts = 0;
    while (!good && starts < 40) {
        starts++;
        /* The counter must equal the DMA position on every start: the DMA
         * restarts at the ring's beginning, so count from there, including
         * the first node. (Carrying the count across restarts made the
         * sender read the wrong ring offset after a multi-start calibration.) */
        dma_pos_reset(&ppos);
#if S31_DEBUG_CMDS
        tr_start_n = 0;
#endif
        if (bits10 && starts > 1) cam_lsb_rephase();
        ESP_ERROR_CHECK(parlio_rx_unit_enable(rx, true));
        for (int k = 0; k < 16; k++) {
            esp_rom_gpio_connect_out_signal(pads[k], diag_sig(lane_diag[k]), false, false);
            gpio_set_drive_capability(pads[k], GPIO_DRIVE_CAP_3);
        }
        if (bits10) for (int k = 0; k < 4; k++) {
            esp_rom_gpio_connect_out_signal(c_pads[k], diag_sig(c_diag[k]), false, false);
            gpio_set_drive_capability(c_pads[k], GPIO_DRIVE_CAP_3);
        }
        parlio_receive_config_t rcfg = { .delimiter = deli, .flags.partial_rx_en = 1 };
        ESP_ERROR_CHECK(parlio_rx_unit_receive(rx, ring, RING_BYTES, &rcfg));
        ESP_ERROR_CHECK(parlio_rx_soft_delimiter_start_stop(rx, deli, true));
        vTaskDelay(pdMS_TO_TICKS(5));
        int ex = verify();
        good = ex == cmp_n;
        printf("stream: start %d: %d/%d bit-exact\n", starts, ex, cmp_n);
        if (good && bits10) {
            /* Restart the camera until its ring offset is a multiple of 8
             * samples, so pack_zip's loads from both rings are aligned. */
            good = 0;
            for (int t = 1; t <= 64 && !good; t++) {
                cam_lsb_start(false);
                vTaskDelay(pdMS_TO_TICKS(2));
                int m = cam_align();
                if (m == cmp_n && (cam_off & 7) == 0) {
                    int64_t off1 = cam_off;
                    vTaskDelay(pdMS_TO_TICKS(50));
                    int m2 = cam_align();
                    printf("stream: camera try %d, bus phase %d: merged 10-bit %d/%d, again %d/%d, offset %lld then %lld\n",
                           t, last_cpar, m, cmp_n, m2, cmp_n, (long long)off1, (long long)cam_off);
                    good = m2 == cmp_n && off1 == cam_off;
                } else if (m != cmp_n) {
                    printf("stream: camera try %d: merged %d/%d (bus phase %d vs %d)\n", t, m, cmp_n, last_cpar, last_ppar);
                }
                if (!good) cam_lsb_stop();
            }
        }
        if (!good) { parlio_rx_soft_delimiter_start_stop(rx, deli, false); parlio_rx_unit_disable(rx); }
    }
    if (!good) { printf("stream: calibration failed\n"); return; }
    /* snap stays allocated: the debug "check" command reuses it */

    eth = s31_net_eth_handle();
    if (bits10) { double act; tune_hz(lo_get(), &act); }   /* exact word, as the monitor expects */
    radio_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(monitor_task, "radiomon", 3072, NULL, 4, NULL, 0);
    stats_q = xQueueCreate(1, sizeof(struct stats));
    xTaskCreatePinnedToCore(stats_task, "vrtstats", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(sender_task, "vrt", 6144, NULL, 20, &sender, 1);
    xTaskCreatePinnedToCore(control_task, "vrtctl", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(vrt_control_task, "vrt49.2", 4096, NULL, 5, NULL, 0);
}
