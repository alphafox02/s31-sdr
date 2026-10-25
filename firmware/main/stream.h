/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Take the capture buffers; called first thing at boot, before other
 * allocations break up internal RAM. */
void s31_stream_reserve(int bits);
void s31_stream_main(int channel, int bits);

/* xTaskCreatePinnedToCore that reports a failure (usually no internal RAM
 * for the stack) on the console instead of failing silently. */
bool s31_spawn(TaskFunction_t fn, const char *name, uint32_t stack, void *arg, UBaseType_t prio,
               TaskHandle_t *handle, BaseType_t core);

/* Send packets of another stream through the S31 sender, which transmits
 * them between its own. s31_vrt_claim returns room for one VRT packet (up
 * to 1088 bytes), or NULL if the queue is full (counted by
 * s31_vrt_queue_drops); s31_vrt_commit queues it, from and to UDP `port`.
 * One producer task only. */
uint8_t *s31_vrt_claim(void);
void s31_vrt_commit(uint16_t port, int vrt_len);
uint32_t s31_vrt_queue_drops(void);
