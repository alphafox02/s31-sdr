/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-S31 RX sample-dump engine: the registers this firmware uses.
 *
 * Block names come from Espressif's public ESP-IDF headers
 * (components/soc/esp32s31/include/modem/reg_base.h,
 * register/soc/hp_system_reg.h). Field layouts inside the data-dump block
 * were recovered by disassembling adctrig() and print_dump_data() in the
 * ESP-IDF RF test library (librftest.a).
 *
 * The engine captures 80 MS/s, 10-bit I/Q with the AGC gain index per
 * sample, one 32-bit word per sample: I[9:0] | Q[19:10] | gain[27:20]. The
 * raw I/Q is spectrally inverted (absolute frequency = LO - offset); the
 * streamer undoes that by sending Q first.
 */
#pragma once
#include <stdint.h>

#define S31_REG(a)              (*(volatile uint32_t *)(a))

/* MODEM_DATADUMP (MODEM_BASE + 0x9000): control word. adctrig writes
 * count + 1 into the length field: a capture of N samples writes N + 1
 * words. */
#define S31_DATADUMP_BASE       0x20109000u
#define S31_DUMP_CTRL           (S31_DATADUMP_BASE + 0x04)
#define S31_DUMP_CTRL_COUNT_M   0x0001FFFFu   /* bits 0-16: length           */
#define S31_DUMP_CTRL_ABORT     (1u << 17)    /* set: ends the capture at once */
#define S31_DUMP_CTRL_DONE      (1u << 18)    /* set by hardware             */
#define S31_DUMP_CTRL_TRIG      (1u << 19)    /* write 1 then 0 to start     */
#define S31_DUMP_CTRL_ENABLE    (1u << 31)

/* HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG (DR_REG_HP_SYS_BASE + 0x114): each bit
 * hands one 16 KB block of the 512 KB internal SRAM (0x2F000000..0x2F080000)
 * to the dump engine; 0xFF000000 is the top 128 KB, 0x2F060000..0x2F07FFFF.
 * While a block is handed over the CPU cannot use it, and that window holds
 * the Wi-Fi task and the ROM heap table, so a capture must run with
 * interrupts off and the other core stalled. */
#define S31_TCM_DUMP_OWNER      0x20586114u

/* Destination: one 32-bit word per sample from here. */
#define S31_DUMP_SRAM           0x2F060000u
