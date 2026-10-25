/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * RTL2832U + R820T/R820T2 USB dongle driver, ported to C from webrtlsdr
 * (Copyright 2013 Google Inc., 2024 Jacobo Tarrio Barreiro, Apache-2.0;
 * see NOTICE). Blocking calls: run them from a task other than the one that
 * pumps usb_host_client_handle_events(), which delivers their completions.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "usb/usb_host.h"

typedef struct rtl_dev rtl_dev_t;

/* Initialise the demodulator and find the tuner; NULL if either fails. The
 * caller has opened the device and claimed interface 0. */
rtl_dev_t *rtl_open(usb_host_client_handle_t client, usb_device_handle_t dev);
/* Free the driver's resources (not the USB device itself). */
void rtl_close(rtl_dev_t *r);
const char *rtl_tuner_name(const rtl_dev_t *r);
/* Returns 0 and the rate / frequency actually set, or -1. */
int rtl_set_sample_rate(rtl_dev_t *r, uint32_t rate, uint32_t *actual);
int rtl_set_freq(rtl_dev_t *r, uint32_t hz, uint32_t *actual);
/* Tuner gain in dB, 0-52.5 in steps of about 1.2-2.3 dB; negative = the
 * tuner's own AGC. `actual` (if not NULL) gets the gain set, or -1 for AGC. */
int rtl_set_gain(rtl_dev_t *r, double db, double *actual);
/* Flush the sample FIFO; call before starting bulk reads from endpoint 0x81. */
int rtl_reset_buffer(rtl_dev_t *r);
