/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host test for dma_advance() (firmware/main/dma_pos.h):
 *   cc -std=c11 -Wall -Wextra -Werror -Ifirmware/main tests/test_dma_advance.c -o test_dma_advance
 */
#include <stdio.h>
#include "dma_pos.h"

static int fails;
#define EXPECT(got, want) do { unsigned long long g = (got), w = (want); \
    if (g != w) { printf("FAIL %s:%d: %llu != %llu\n", __FILE__, __LINE__, g, w); fails++; } } while (0)

int main(void)
{
    const uint32_t ring = 16 * 4092, node = 4092, bpu = 80;   /* CI8: 80 bytes/us, ring 818 us */
    /* normal: one node in ~51 us, with interrupt latency either way */
    EXPECT(dma_advance(node, 51, bpu, ring), node);
    EXPECT(dma_advance(node, 5, bpu, ring), node);
    EXPECT(dma_advance(node, 300, bpu, ring), node);
    /* two completions behind one interrupt */
    EXPECT(dma_advance(2 * node, 102, bpu, ring), 2 * node);
    /* nothing new, shortly after */
    EXPECT(dma_advance(0, 20, bpu, ring), 0);
    /* a stall hid exactly one lap: same position, ~818 us later */
    EXPECT(dma_advance(0, 818, bpu, ring), ring);
    /* a stall of one lap plus a node, latency +-100 us */
    EXPECT(dma_advance(node, 870, bpu, ring), ring + node);
    EXPECT(dma_advance(node, 770, bpu, ring), ring + node);
    EXPECT(dma_advance(node, 970, bpu, ring), ring + node);
    /* three laps and three nodes */
    EXPECT(dma_advance(3 * node, 3 * 818 + 153, bpu, ring), 3 * ring + 3 * node);
    /* no time information */
    EXPECT(dma_advance(node, 0, bpu, ring), node);
    /* 10-bit PARLIO: 40 bytes/us, ring 1637 us */
    EXPECT(dma_advance(node, 102, 40, ring), node);
    EXPECT(dma_advance(node, 1637 + 102, 40, ring), ring + node);
    if (!fails) printf("dma_advance: all tests passed\n");
    return fails != 0;
}
