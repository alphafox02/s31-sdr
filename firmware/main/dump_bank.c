/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * Reserve the RX dump-engine's fixed destination bank (0x2F060000) so FreeRTOS
 * and the heap do not use it. In the factory cert-test firmware this region is
 * a dedicated capture bank; in an ordinary app it falls inside the RAM heap, so
 * without this the engine overwrites live allocations.
 *
 * 64 KB: the engine may take ownership of more than the words it writes (on the
 * C3 the CPU loses the whole block while the dump runs).
 *
 * This file exports no symbols, so the component is linked WHOLE_ARCHIVE
 * (main/CMakeLists.txt); otherwise the linker drops it and the reservation
 * silently disappears.
 */
#include "heap_memory_layout.h"

SOC_RESERVE_MEMORY_REGION(0x2F060000, 0x2F070000, s31_dump_bank);  /* 64 KB */
