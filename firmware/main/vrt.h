/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * VITA 49 helpers shared by the streams: big-endian words, the 20-bit
 * fixed point used for frequencies and rates, and the CIF0 and command
 * (CAM) bits this firmware uses.
 */
#pragma once
#include <stdint.h>

static inline void be32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static inline void be64(uint8_t *p, uint64_t v) { be32(p, v >> 32); be32(p + 4, (uint32_t)v); }
static inline uint64_t fix20(double v) { return (uint64_t)(int64_t)(v * (double)(1 << 20)); }

static inline uint32_t rd_be32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

static inline double rd_fix20(const uint8_t *p)
{
    return (double)(int64_t)((uint64_t)rd_be32(p) << 32 | rd_be32(p + 4)) / (double)(1 << 20);
}

#define CAM_CONTROLLEE_EN  (1u << 31)
#define CAM_CONTROLLEE_UID (1u << 30)
#define CAM_CONTROLLER_EN  (1u << 29)
#define CAM_CONTROLLER_UID (1u << 28)
#define CAM_NACK_ONLY      (1u << 22)
#define CAM_REQ_VALIDATION (1u << 20)
#define CAM_REQ_EXECUTION  (1u << 19)
#define CAM_REQ_QUERY      (1u << 18)
#define CAM_ACK_PARTIAL    (1u << 11)
#define CAM_ACK_EXECUTED   (1u << 10)
#define CIF0_BW   (1u << 29)
#define CIF0_IF   (1u << 28)
#define CIF0_RF   (1u << 27)
#define CIF0_GAIN (1u << 23)
#define CIF0_RATE (1u << 21)
#define CIF0_SE   (1u << 16)          /* State/Event: AGC/MGC enable 28, indicator 16 */
#define CIF0_FMT  (1u << 15)

/* Field sizes in words for CIF0 bits 31..0 (VITA 49.0 table 9.1-1). */
static inline int cif0_words(int bit)
{
    switch (bit) {
    case 29: case 28: case 27: case 26: case 25: return 2;   /* bw, IF, RF, RF off, IF off */
    case 24: case 23: case 22: return 1;                     /* ref level, gain, over-range */
    case 21: case 20: return 2;                               /* sample rate, ts adjust */
    case 19: case 18: return 1;                               /* ts cal time, temperature */
    case 17: return 2;                                        /* device ID */
    case 16: return 1;                                        /* state/event */
    case 15: return 2;                                        /* data format */
    default: return -1;                                       /* not handled */
    }
}
