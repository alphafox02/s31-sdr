/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Set up the codec and start the radio; call before s31_usb_start. */
void s31_radio_start(void);

/* RTL-SDR samples (unsigned 8-bit I, Q), from the USB callback. */
void s31_radio_feed(const uint8_t *iq, int len);

/* Text commands: "on", "off" (mute), "f <MHz>", "seek up", "seek down",
 * "scan", "vol <0-100>", "vol up", "vol down", "tone" (a 1 kHz test
 * tone), "status". Fills `reply` and returns true if the command was
 * recognised. */
bool s31_radio_command(const char *cmd, char *reply, size_t n);

/* For voice control: the codec's input (48 kHz stereo, 16-bit: the
 * microphone left, a copy of the playback right; 0 on success), ducking
 * the radio while a command is spoken, and a chime. */
int s31_radio_read_mic(void *buf, int bytes);
void s31_radio_duck(bool on);
enum { CHIME_LISTEN = 1, CHIME_DONE, CHIME_MISSED };
void s31_radio_chime(int kind);

/* The radio's state as JSON for the web page: live, muted, mhz, vol,
 * stations. Returns the length. */
int s31_radio_status_json(char *buf, size_t n);
