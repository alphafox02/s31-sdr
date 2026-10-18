/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/* Free-running capture on the LCD_CAM camera input into a DMA ring of
 * `nodes` x `node` bytes, `width` (8 or 16) bits per PCLK edge; pad k feeds
 * data lane data_lanes[k], the other lanes read 0. clk_div != 0: also drive
 * PLL_F160M / clk_div out on pclk_pad and sample on it. bytes_per_us: the
 * ring's fill rate, for recovering laps hidden by interrupt stalls. */
esp_err_t cam_lsb_init(int pclk_pad, const int data_pads[4], const int data_lanes[4], int width,
                       uint8_t *ring, size_t node, int nodes, int clk_div, uint32_t bytes_per_us);
void cam_lsb_rephase(void);         /* restart the clock divider at a new phase */
void cam_lsb_start(bool invert_pclk);
void cam_lsb_stop(void);
uint64_t cam_lsb_written(void);     /* bytes written since start (hardware descriptor) */
uint32_t cam_lsb_laps(void);        /* whole ring laps recovered after interrupt stalls */
