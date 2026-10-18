/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0 */
#include "dma_pos.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "soc/ahb_dma_struct.h"
#include "soc/axi_dma_struct.h"

static IRAM_ATTR bool in_sram(uint32_t a) { return a >= 0x2F000000u && a < 0x2F080000u; }

/* The channel serving this ring: the one whose last completed descriptor
 * points into it (5 AHB, 3 AXI receive channels). */
static IRAM_ATTR void find_channel(struct dma_pos *p)
{
    for (int k = 0; k < 8 && !p->eof_reg; k++) {
        volatile uint32_t *r = k < 5 ? &AHB_DMA.channel[k].in.in_suc_eof_des_addr.val
                                     : &AXI_DMA.in[k - 5].conf.in_suc_eof_des_addr.val;
        uint32_t d = *r;
        if (!in_sram(d)) continue;
        uint32_t buf = ((volatile uint32_t *)d)[1];
        if (buf >= p->ring && buf < p->ring + p->ring_bytes) p->eof_reg = r;
    }
}

void dma_pos_init(struct dma_pos *p, void *ring, uint32_t ring_bytes, uint32_t node_max,
                  uint32_t bytes_per_us)
{
    *p = (struct dma_pos){ .ring = (uintptr_t)ring, .ring_bytes = ring_bytes,
                           .node_max = node_max, .bytes_per_us = bytes_per_us };
    dma_pos_reset(p);
}

void dma_pos_reset(struct dma_pos *p)
{
    p->written = 0;
    p->last_end = p->ring;
    p->last_us = 0;
    p->last_desc = p->eof_reg ? *p->eof_reg : 0;    /* left over from the previous start */
}

bool IRAM_ATTR dma_pos_update(struct dma_pos *p)
{
    if (!p->eof_reg) find_channel(p);
    if (!p->eof_reg) return false;
    uint32_t d = *p->eof_reg;
    if (!in_sram(d)) return false;
    uintptr_t buf = ((volatile uint32_t *)d)[1];
    uint32_t len = (((volatile uint32_t *)d)[0] >> 12) & 0xFFF;
    if (buf < p->ring || len == 0 || len > p->node_max || buf + len > p->ring + p->ring_bytes) return false;
    int64_t now = esp_timer_get_time();
    uintptr_t end = buf + len;
    uint32_t mod = (uint32_t)((end - p->last_end + p->ring_bytes) % p->ring_bytes);
    uint64_t adv = p->last_us ? dma_advance(mod, now - p->last_us, p->bytes_per_us, p->ring_bytes) : mod;
    /* The same descriptor again counts only as whole laps after a real
     * completion; otherwise it is a stale or repeated interrupt. */
    if (d == p->last_desc && (!p->last_us || adv < p->ring_bytes)) return false;
    if (!adv) return false;
    if (adv >= p->ring_bytes) p->laps += (uint32_t)(adv / p->ring_bytes);
    p->written += adv;
    p->last_end = end;
    p->last_desc = d;
    p->last_us = now;
    return true;
}

uint64_t dma_pos_written(const struct dma_pos *p)
{
    uint64_t a, b;
    do { a = p->written; b = p->written; } while (a != b);   /* updated by the ISR */
    return a;
}
