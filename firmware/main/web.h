/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdint.h>

/* Start the radio's Wi-Fi hotspot and its web page (web.c). */
void s31_web_start(void);

/* One block of stereo audio (interleaved, 48 kHz) from radio_task, for the
 * phones listening; returns at once when none are. */
void s31_web_audio(const int16_t *stereo, int frames);
