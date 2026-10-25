/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * RTL2832U + R820T/R820T2 driver, ported from webrtlsdr's rtlcom.ts,
 * rtl2832u.ts and r8xx.ts (Copyright 2013 Google Inc., 2024 Jacobo Tarrio
 * Barreiro, Apache-2.0; see NOTICE). Register meanings are as documented
 * there.
 *
 * Every register access is a vendor control transfer: wValue = register
 * address, wIndex = block (0x100 USB, 0x200 system, 0x600 I2C, demod page),
 * OR 0x10 for writes.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rtl_drv.h"

#define RTL_XTAL      28800000u
#define R8XX_IF_HZ    3570000u
#define R820T_I2C     0x34
#define WRITE_FLAG    0x10
#define BLK_USB       0x100
#define BLK_SYS       0x200
#define BLK_I2C       0x600

struct rtl_dev {
    usb_host_client_handle_t client;
    usb_device_handle_t dev;
    usb_transfer_t *xfer;               /* control transfers, one at a time */
    SemaphoreHandle_t done;
    bool broken;                        /* a transfer never completed */
    uint8_t i2c;
    int vco_power_ref;
    uint32_t xtal;
    bool pll_lock;
    uint8_t shadow[0x20 - 5];           /* tuner registers 0x05-0x1f */
    const char *tuner;
};

/* ---- transport ---- */

static void ctrl_done(usb_transfer_t *t) { xSemaphoreGive((SemaphoreHandle_t)t->context); }

/* One vendor control transfer; stalls are retried as webrtlsdr does. Reads
 * always ask for at least 8 bytes. */
static int ctrl(rtl_dev_t *r, bool in, uint16_t value, uint16_t index, uint8_t *data, uint16_t len)
{
    if (r->broken) return -1;
    uint16_t wlen = in && len < 8 ? 8 : len;
    for (int attempt = 0; attempt <= 5; attempt++) {
        usb_setup_packet_t *s = (usb_setup_packet_t *)r->xfer->data_buffer;
        s->bmRequestType = (in ? USB_BM_REQUEST_TYPE_DIR_IN : USB_BM_REQUEST_TYPE_DIR_OUT) |
                           USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_RECIP_DEVICE;
        s->bRequest = 0;
        s->wValue = value;
        s->wIndex = index;
        s->wLength = wlen;
        if (!in && len) memcpy(r->xfer->data_buffer + USB_SETUP_PACKET_SIZE, data, len);
        r->xfer->num_bytes = USB_SETUP_PACKET_SIZE + wlen;
        r->xfer->device_handle = r->dev;
        r->xfer->bEndpointAddress = 0;
        r->xfer->callback = ctrl_done;
        r->xfer->context = r->done;
        if (usb_host_transfer_submit_control(r->client, r->xfer) != ESP_OK) return -1;
        /* the library has no transfer timeout yet: give up rather than hang */
        if (xSemaphoreTake(r->done, pdMS_TO_TICKS(500)) != pdTRUE) { r->broken = true; return -1; }
        if (r->xfer->status == USB_TRANSFER_STATUS_COMPLETED) {
            if (in && len) memcpy(data, r->xfer->data_buffer + USB_SETUP_PACKET_SIZE, len);
            return 0;
        }
        if (r->xfer->status != USB_TRANSFER_STATUS_STALL) return -1;
        vTaskDelay(pdMS_TO_TICKS(5 * (attempt + 1)));
    }
    return -1;
}

/* Little-endian register write (USB and system blocks). */
static int write_reg(rtl_dev_t *r, uint16_t block, uint16_t addr, uint16_t val, int len)
{
    uint8_t b[2] = { val & 0xff, val >> 8 };
    return ctrl(r, false, addr, block | WRITE_FLAG, b, len);
}

static int read_reg(rtl_dev_t *r, uint16_t block, uint16_t addr, uint8_t *val)
{
    return ctrl(r, true, addr, block, val, 1);
}

/* Demodulator registers are written big-endian, then a dummy read
 * completes the write. */
static int demod_write(rtl_dev_t *r, int page, uint8_t addr, uint16_t val, int len)
{
    uint8_t b[2];
    if (len == 1) b[0] = val & 0xff;
    else { b[0] = val >> 8; b[1] = val & 0xff; }
    if (ctrl(r, false, (addr << 8) | 0x20, page | WRITE_FLAG, b, len)) return -1;
    uint8_t dummy;
    return read_reg(r, 0x0a, 0x0120, &dummy);
}

static int i2c_write(rtl_dev_t *r, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return ctrl(r, false, r->i2c, BLK_I2C | WRITE_FLAG, b, 2);
}

static int i2c_read(rtl_dev_t *r, uint8_t reg, uint8_t *buf, int len)
{
    if (ctrl(r, false, r->i2c, BLK_I2C | WRITE_FLAG, &reg, 1)) return -1;
    return ctrl(r, true, r->i2c, BLK_I2C, buf, len);
}

/* Consecutive registers, at most 7 per transfer. */
static int i2c_write_buf(rtl_dev_t *r, uint8_t reg, const uint8_t *vals, int n)
{
    uint8_t b[8];
    for (int i = 0; i < n; i += 7) {
        int k = n - i < 7 ? n - i : 7;
        b[0] = reg + i;
        memcpy(b + 1, vals + i, k);
        if (ctrl(r, false, r->i2c, BLK_I2C | WRITE_FLAG, b, k + 1)) return -1;
    }
    return 0;
}

/* The tuner sits behind the demodulator's I2C repeater. */
static int i2c_open(rtl_dev_t *r) { return demod_write(r, 1, 0x01, 0x18, 1); }
static int i2c_close(rtl_dev_t *r) { return demod_write(r, 1, 0x01, 0x10, 1); }

/* ---- R8xx tuner ---- */

static const uint8_t r8xx_init_regs[0x20 - 5] = {
    0x83, 0x32, 0x75, 0xc0, 0x40, 0xd6, 0x6c, 0xf5, 0x63, 0x75, 0x68, 0x64, 0x83, 0x80,
    0x00, 0x0f, 0x00, 0xc0, 0x30, 0x48, 0xcc, 0x60, 0x00, 0x54, 0xae, 0x4a, 0xc0,
};

/* Multiplexer settings per band: from MHz, open drain, RF/tracking filter,
 * LPNF/LPF. */
static const struct { uint16_t mhz; uint8_t od, rf, lpf; } r8xx_mux[] = {
    { 0, 0x08, 0x02, 0xdf },   { 50, 0x08, 0x02, 0xbe },  { 55, 0x08, 0x02, 0x8b },
    { 60, 0x08, 0x02, 0x7b },  { 65, 0x08, 0x02, 0x69 },  { 70, 0x08, 0x02, 0x58 },
    { 75, 0x00, 0x02, 0x44 },  { 90, 0x00, 0x02, 0x34 },  { 110, 0x00, 0x02, 0x24 },
    { 140, 0x00, 0x02, 0x14 }, { 180, 0x00, 0x02, 0x13 }, { 250, 0x00, 0x02, 0x11 },
    { 280, 0x00, 0x02, 0x00 }, { 310, 0x00, 0x41, 0x00 }, { 588, 0x00, 0x40, 0x00 },
};

static uint8_t bitrev8(uint8_t b)
{
    static const uint8_t rev4[16] = { 0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };
    return (rev4[b & 0xf] << 4) | rev4[b >> 4];
}

/* The tuner returns its registers bit-reversed. */
static int tuner_read(rtl_dev_t *r, uint8_t *buf, int len)
{
    if (i2c_read(r, 0x00, buf, len)) return -1;
    for (int i = 0; i < len; i++) buf[i] = bitrev8(buf[i]);
    return 0;
}

static void set_mask(rtl_dev_t *r, uint8_t addr, uint8_t val, uint8_t mask)
{
    uint8_t *s = &r->shadow[addr - 5];
    *s = (*s & ~mask) | (val & mask);
}

static int write_mask(rtl_dev_t *r, uint8_t addr, uint8_t val, uint8_t mask)
{
    set_mask(r, addr, val, mask);
    return i2c_write(r, addr, r->shadow[addr - 5]);
}

static void pll_lock_wait(rtl_dev_t *r)
{
    uint8_t b[3];
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 1; i <= 4; i++) {
            if (tuner_read(r, b, 3) == 0 && (b[2] & 0x40)) { r->pll_lock = true; return; }
            vTaskDelay(pdMS_TO_TICKS(i * 5));
        }
        write_mask(r, 0x12, 0x60, 0xe0);           /* raise the VCO current and try again */
    }
    r->pll_lock = false;
}

/* Set the PLL to `freq` Hz; returns the frequency set, or 0. */
static double set_pll(rtl_dev_t *r, uint32_t freq)
{
    uint64_t ref = r->xtal;
    write_mask(r, 0x10, 0x00, 0x10);
    write_mask(r, 0x1a, 0x00, 0x0c);
    write_mask(r, 0x12, 0x80, 0xe0);
    int div_num = (int)floor(log2(1770000000.0 / freq));
    if (div_num < 0) div_num = 0;
    if (div_num > 6) div_num = 6;
    uint32_t mix_div = 1u << (div_num + 1);
    uint8_t b[5];
    if (tuner_read(r, b, 5)) return 0;
    int vco_fine = (b[4] & 0x30) >> 4;
    if (vco_fine > r->vco_power_ref && div_num > 0) div_num--;
    else if (vco_fine < r->vco_power_ref && div_num < 6) div_num++;
    uint64_t vco = (uint64_t)freq * mix_div;
    uint64_t nint = vco / (2 * ref), fra = vco % (2 * ref);
    if (nint > 63 || nint < 13) { r->pll_lock = false; return 0; }
    uint32_t ni = (nint - 13) / 4, si = (nint - 13) % 4;
    uint64_t sdm = (32768 * fra) / ref;
    if (sdm > 65535) sdm = 65535;
    set_mask(r, 0x10, div_num << 5, 0xe0);
    set_mask(r, 0x12, fra == 0 ? 0x08 : 0x00, 0x08);
    set_mask(r, 0x14, ni + (si << 6), 0xff);
    set_mask(r, 0x15, sdm & 0xff, 0xff);
    set_mask(r, 0x16, sdm >> 8, 0xff);
    if (i2c_write_buf(r, 0x10, &r->shadow[0x10 - 5], 7)) return 0;
    pll_lock_wait(r);
    write_mask(r, 0x1a, 0x08, 0x08);
    return r->pll_lock ? 2.0 * ref * (nint + sdm / 65536.0) / mix_div : 0;
}

static void set_mux(rtl_dev_t *r, uint32_t freq)
{
    uint32_t mhz = freq / 1000000;
    int i = 0;
    while (i + 1 < (int)(sizeof(r8xx_mux) / sizeof(r8xx_mux[0])) && mhz >= r8xx_mux[i + 1].mhz) i++;
    write_mask(r, 0x17, r8xx_mux[i].od, 0x08);
    write_mask(r, 0x1a, r8xx_mux[i].rf, 0xc3);
    write_mask(r, 0x1b, r8xx_mux[i].lpf, 0xff);
    write_mask(r, 0x10, 0x00, 0x0b);
    write_mask(r, 0x08, 0x00, 0x3f);
    write_mask(r, 0x09, 0x00, 0x3f);
}

static int calibrate_filter(rtl_dev_t *r)
{
    for (int first = 1;; first = 0) {
        write_mask(r, 0x0b, 0x60, 0x60);
        write_mask(r, 0x0f, 0x04, 0x04);
        write_mask(r, 0x10, 0x00, 0x03);
        set_pll(r, 56000000);
        if (!r->pll_lock) return -1;
        write_mask(r, 0x0b, 0x10, 0x10);
        write_mask(r, 0x0b, 0x00, 0x10);
        write_mask(r, 0x0f, 0x00, 0x04);
        uint8_t b[5];
        if (tuner_read(r, b, 5)) return -1;
        int cap = b[4] & 0x0f;
        if (cap == 0x0f) cap = 0;
        if (cap == 0 || !first) return cap;
    }
}

static int tuner_open(rtl_dev_t *r)
{
    demod_write(r, 1, 0xb1, 0x1a, 1);              /* low-IF input */
    demod_write(r, 0, 0x08, 0x4d, 1);              /* ADC input select for the tuner's IF */
    demod_write(r, 1, 0x15, 0x01, 1);              /* inverted spectrum */
    i2c_open(r);
    memcpy(r->shadow, r8xx_init_regs, sizeof(r->shadow));
    i2c_write_buf(r, 0x05, r->shadow, sizeof(r->shadow));
    vTaskDelay(pdMS_TO_TICKS(5));
    /* electronics */
    write_mask(r, 0x0c, 0x00, 0x0f);
    write_mask(r, 0x13, 0x31, 0x3f);
    write_mask(r, 0x1d, 0x00, 0x38);
    int cap = calibrate_filter(r);
    if (cap < 0) { i2c_close(r); return -1; }
    write_mask(r, 0x0a, 0x10 | cap, 0x1f);
    write_mask(r, 0x0b, 0x6b, 0xef);
    write_mask(r, 0x07, 0x00, 0x80);
    write_mask(r, 0x06, 0x10, 0x30);
    write_mask(r, 0x1e, 0x40, 0x60);
    write_mask(r, 0x05, 0x00, 0x80);
    write_mask(r, 0x1f, 0x00, 0x80);
    write_mask(r, 0x0f, 0x00, 0x80);
    write_mask(r, 0x19, 0x60, 0x60);
    write_mask(r, 0x1d, 0xe5, 0xc7);
    write_mask(r, 0x1c, 0x24, 0xf8);
    write_mask(r, 0x0d, 0x53, 0xff);
    write_mask(r, 0x0e, 0x75, 0xff);
    write_mask(r, 0x05, 0x00, 0x60);
    write_mask(r, 0x06, 0x00, 0x08);
    write_mask(r, 0x11, 0x38, 0x08);
    write_mask(r, 0x17, 0x30, 0x30);
    write_mask(r, 0x0a, 0x40, 0x60);
    write_mask(r, 0x1d, 0x00, 0x38);
    write_mask(r, 0x1c, 0x00, 0x04);
    write_mask(r, 0x06, 0x00, 0x40);
    write_mask(r, 0x1a, 0x30, 0x30);
    write_mask(r, 0x1d, 0x18, 0x38);
    write_mask(r, 0x1c, 0x24, 0x04);
    write_mask(r, 0x1e, 0x0d, 0x1f);
    write_mask(r, 0x1a, 0x20, 0x30);
    return i2c_close(r);
}

/* ---- RTL2832U ---- */

static const uint8_t fir_coeffs[20] = {
    0xca, 0xdc, 0xd7, 0xd8, 0xe0, 0xf2, 0x0e, 0x35, 0x06, 0x50,
    0x9c, 0x0d, 0x71, 0x11, 0x14, 0x71, 0x74, 0x19, 0x41, 0xa5,
};

static int demod_init(rtl_dev_t *r)
{
    int e = 0;
    e |= write_reg(r, BLK_USB, 0x2000, 0x09, 1);     /* DMA enable, full packet mode */
    e |= write_reg(r, BLK_USB, 0x2158, 0x0200, 2);   /* 512-byte bulk packets */
    e |= write_reg(r, BLK_USB, 0x2148, 0x0210, 2);   /* stall + FIFO reset */
    e |= write_reg(r, BLK_SYS, 0x300b, 0x22, 1);     /* demod analog power + clock */
    e |= write_reg(r, BLK_SYS, 0x3000, 0xe8, 1);     /* PLL, ADC I/Q, release reset */
    e |= demod_write(r, 1, 0x01, 0x14, 1);           /* soft reset */
    e |= demod_write(r, 1, 0x01, 0x10, 1);
    e |= demod_write(r, 1, 0x15, 0x00, 1);
    for (int a = 0x16; a <= 0x1b; a++) e |= demod_write(r, 1, a, 0x00, 1);
    for (int i = 0; i < 20; i++) e |= demod_write(r, 1, 0x1c + i, fir_coeffs[i], 1);
    e |= demod_write(r, 0, 0x19, 0x05, 1);           /* SDR mode, no digital AGC */
    e |= demod_write(r, 1, 0x93, 0xf0, 1);
    e |= demod_write(r, 1, 0x94, 0x0f, 1);
    e |= demod_write(r, 1, 0x11, 0x00, 1);
    e |= demod_write(r, 1, 0x04, 0x00, 1);
    e |= demod_write(r, 0, 0x61, 0x60, 1);
    e |= demod_write(r, 0, 0x06, 0x80, 1);
    e |= demod_write(r, 1, 0xb1, 0x1b, 1);
    e |= demod_write(r, 0, 0x0d, 0x83, 1);           /* no 4.096 MHz test clock output */
    return e ? -1 : 0;
}

static int set_if_freq(rtl_dev_t *r, uint32_t if_hz)
{
    int32_t m = -(int32_t)(((uint64_t)if_hz << 22) / r->xtal);
    int e = demod_write(r, 1, 0x19, (m >> 16) & 0x3f, 1);
    e |= demod_write(r, 1, 0x1a, (m >> 8) & 0xff, 1);
    e |= demod_write(r, 1, 0x1b, m & 0xff, 1);
    return e ? -1 : 0;
}

rtl_dev_t *rtl_open(usb_host_client_handle_t client, usb_device_handle_t dev)
{
    rtl_dev_t *r = calloc(1, sizeof(*r));
    if (!r) return NULL;
    r->client = client;
    r->dev = dev;
    r->xtal = RTL_XTAL;
    r->done = xSemaphoreCreateBinary();
    if (!r->done || usb_host_transfer_alloc(64 + USB_SETUP_PACKET_SIZE, 0, &r->xfer) != ESP_OK) goto fail;
    if (demod_init(r)) goto fail;

    /* tuner: R820T / R820T2 answer with chip ID 0x69 at I2C 0x34 */
    r->i2c = R820T_I2C;
    r->vco_power_ref = 2;
    uint8_t id = 0;
    i2c_open(r);
    int e = i2c_read(r, 0x00, &id, 1);
    i2c_close(r);
    if (e || id != 0x69) { printf("@@RTL no supported tuner (I2C 0x34 id 0x%02x)\n", id); goto fail; }
    r->tuner = "R820T";
    if (tuner_open(r)) { printf("@@RTL tuner init failed\n"); goto fail; }

    rtl_set_gain(r, -1, NULL);
    demod_write(r, 1, 0x3e, 0x00, 1);                /* no ppm correction */
    demod_write(r, 1, 0x3f, 0x00, 1);
    if (set_if_freq(r, R8XX_IF_HZ)) goto fail;
    return r;
fail:
    rtl_close(r);
    return NULL;
}

void rtl_close(rtl_dev_t *r)
{
    if (!r) return;
    /* After a control transfer that never completed, the USB stack may
     * still own it and later signal r->done: leave both (and r) alone. */
    if (r->broken) return;
    if (r->xfer) usb_host_transfer_free(r->xfer);
    if (r->done) vSemaphoreDelete(r->done);
    free(r);
}

const char *rtl_tuner_name(const rtl_dev_t *r) { return r->tuner; }

int rtl_set_sample_rate(rtl_dev_t *r, uint32_t rate, uint32_t *actual)
{
    uint32_t ratio = (uint32_t)((((uint64_t)r->xtal) << 22) / rate) & 0x0ffffffc;
    if (actual) *actual = (uint32_t)((((uint64_t)r->xtal) << 22) / ratio);
    int e = demod_write(r, 1, 0x9f, (ratio >> 16) & 0xffff, 2);
    e |= demod_write(r, 1, 0xa1, ratio & 0xffff, 2);
    e |= demod_write(r, 1, 0x01, 0x14, 1);           /* reset the demodulator */
    e |= demod_write(r, 1, 0x01, 0x10, 1);
    return e ? -1 : 0;
}

int rtl_set_freq(rtl_dev_t *r, uint32_t hz, uint32_t *actual)
{
    i2c_open(r);
    set_mux(r, hz + R8XX_IF_HZ);
    double f = set_pll(r, hz + R8XX_IF_HZ);
    i2c_close(r);
    if (f <= 0) return -1;
    if (actual) *actual = (uint32_t)(f - R8XX_IF_HZ + 0.5);
    return 0;
}

int rtl_set_gain(rtl_dev_t *r, double db, double *actual)
{
    int e = i2c_open(r);
    if (actual) *actual = -1;
    if (db < 0) {
        e |= write_mask(r, 0x05, 0x00, 0x10);        /* LNA auto */
        e |= write_mask(r, 0x07, 0x10, 0x10);        /* mixer auto */
        e |= write_mask(r, 0x0c, 0x0b, 0x9f);        /* VGA 26.5 dB */
    } else {
        /* LNA steps ~2.3 dB, mixer ~1.2 dB: step both together, up to
         * 15 x 3.5 = 52.5 dB */
        int full = (int)floor(db / 3.5), half = db - 3.5 * full >= 2.3;
        if (full < 0) full = 0;
        if (full > 15) full = 15;
        if (full == 15) half = 0;
        e |= write_mask(r, 0x05, 0x10, 0x10);
        e |= write_mask(r, 0x07, 0x00, 0x10);
        e |= write_mask(r, 0x0c, 0x08, 0x9f);
        e |= write_mask(r, 0x05, full + half, 0x0f);
        e |= write_mask(r, 0x07, full, 0x0f);
        if (actual) *actual = full * 3.5 + half * 2.3;
    }
    e |= i2c_close(r);
    return e ? -1 : 0;
}

int rtl_reset_buffer(rtl_dev_t *r)
{
    int e = write_reg(r, BLK_USB, 0x2148, 0x0210, 2);
    e |= write_reg(r, BLK_USB, 0x2148, 0x0000, 2);
    return e ? -1 : 0;
}
