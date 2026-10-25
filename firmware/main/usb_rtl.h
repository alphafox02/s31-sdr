/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>
#include <stddef.h>

/* Start USB host on the board's USB-A port; an RTL-SDR plugged in there is
 * streamed as a second VITA-49 stream (usb_rtl.c). Runs on core 0. */
void s31_usb_start(void);

/* "rtl ..." text commands, without the "rtl " prefix: "f <Hz>",
 * "gain <dB>" or "gain auto", "rate <S/s>", "status". Fills `reply` and
 * returns true if the command was recognised. */
bool s31_rtl_command(const char *cmd, char *reply, size_t n);
