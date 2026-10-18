/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * Write position of a DMA receive ring, from the hardware's last completed
 * descriptor. Driver callbacks are not a reliable count: after a calibration
 * restart a stale interrupt put the PARLIO driver's node index one ahead of
 * the DMA, and two completions can share one interrupt.
 *
 * The descriptor gives the position modulo the ring, which cannot tell no
 * progress from a whole lap. The time since the previous completion gives
 * the advance to within one node plus interrupt latency (tens of us), far
 * less than half a ring period (409 us for CI8, 819 us otherwise), so it
 * selects the number of whole laps a long stall hid. The count stays exact,
 * and the sender's overrun check turns the jump into an honest sample gap.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Bytes advanced, given the advance modulo the ring and the elapsed time. */
static inline uint64_t dma_advance(uint32_t delta_mod, int64_t elapsed_us,
                                   uint32_t bytes_per_us, uint32_t ring_bytes)
{
    int64_t extra = elapsed_us * (int64_t)bytes_per_us - (int64_t)delta_mod;
    if (elapsed_us <= 0 || extra < (int64_t)(ring_bytes / 2)) return delta_mod;
    return delta_mod + (uint64_t)((extra + ring_bytes / 2) / ring_bytes) * ring_bytes;
}

struct dma_pos {
    uintptr_t ring;
    uint32_t ring_bytes, node_max, bytes_per_us;
    volatile uint32_t *eof_reg;     /* the channel's in_suc_eof_des_addr */
    uint32_t last_desc;             /* its value at the last counted completion */
    uintptr_t last_end;
    int64_t last_us;                /* 0: nothing counted since the reset */
    volatile uint64_t written;      /* bytes since the reset */
    volatile uint32_t laps;         /* whole laps recovered from timing */
};

void dma_pos_init(struct dma_pos *p, void *ring, uint32_t ring_bytes, uint32_t node_max,
                  uint32_t bytes_per_us);
void dma_pos_reset(struct dma_pos *p);     /* at each start, before the DMA runs */
bool dma_pos_update(struct dma_pos *p);    /* from the completion ISR; true if it advanced */
uint64_t dma_pos_written(const struct dma_pos *p);
