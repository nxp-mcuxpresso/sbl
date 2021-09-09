/*
 * Copyright 2018-2021 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(__MEMORY_MAP_H__)
#define __MEMORY_MAP_H__

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

/* ======================== Size definitions ================================= */
#define SIZE_64B 0x00000040u
#define SIZE_512B 0x0000200u
#define SIZE_1KB 0x00000400u
#define SIZE_8KB (SIZE_1KB * 8u)
#define SIZE_32KB (SIZE_1KB * 32u)
#define SIZE_40KB (SIZE_1KB * 40u)
#define SIZE_48KB (SIZE_1KB * 48u)
#define SIZE_64KB (SIZE_1KB * 64u)
#define SIZE_96KB (SIZE_1KB * 96u)
#define SIZE_128KB (SIZE_1KB * 128u)
#define SIZE_256KB (SIZE_1KB * 256u)
#define SIZE_448KB (SIZE_1KB * 448u)
#define SIZE_512KB (SIZE_1KB * 512u)
#define SIZE_640KB (SIZE_1KB * 640u)
#define SIZE_1MB (SIZE_1KB * 1024u)
#define SIZE_2MB (SIZE_1MB * 2u)
#define SIZE_4MB (SIZE_1MB * 4u)
#define SIZE_8MB (SIZE_1MB * 8u)
#define SIZE_16MB (SIZE_1MB * 16u)
#define SIZE_64MB (SIZE_1MB * 64u)
#define SIZE_128MB (SIZE_1MB * 128u)
#define SIZE_256MB (SIZE_1MB * 256u)
#define SIZE_504MB (SIZE_1MB * 504u)
#define SIZE_512MB (SIZE_1MB * 512u)
#define SIZE_1GB (SIZE_1MB * 1024u)

/* ======================== ROM region definitions ================================= */
#define ROM_START_ADDRESS (0x00200000)
#define ROM_SIZE (SIZE_256KB)
#define ROM_END_ADDRES (ROM_START_ADDRESS + ROM_SIZE - 1)

#define M4_ROM_ALIAS_START_ADDRESS (0x00000000u)
#define M4_ROM_ALIAS_SIZE (SIZE_256KB)
#define M4_ROM_ALIAS_END_ADDRES (M4_ROM_ALIAS_START_ADDRESS + M4_ROM_ALIAS_SIZE - 1)

/* ======================== RAM region definitions ================================= */
#define M7_ITCM_SRAM_START_ADDRESS (0x00000000u)
#define M7_ITCM_SRAM_SIZE (SIZE_512KB)
#define M7_ITCM_SRAM_END_ADDRESS (M7_ITCM_SRAM_START_ADDRESS + M7_ITCM_SRAM_SIZE - 1)

#define M7_DTCM_SRAM_START_ADDRESS (0x20000000u)
#define M7_DTCM_SRAM_SIZE (SIZE_512KB)
#define M7_DTCM_SRAM_END_ADDRESS (M7_DTCM_SRAM_START_ADDRESS + M7_DTCM_SRAM_SIZE - 1)

/* M4 ITCM and DTCM */
#define M4_TCM_SRAM_START_ADDRESS (0x1FFE0000)
#define M4_TCM_SRAM_SIZE (SIZE_256KB)
#define M4_TCM_SRAM_END_ADDRESS (M4_TCM_SRAM_START_ADDRESS + M4_TCM_SRAM_SIZE - 1)

#define M4_ITCM_SRAM_START_ADDRESS (0x1FFE0000)
#define M4_ITCM_SRAM_SIZE (SIZE_128KB)
#define M4_ITCM_SRAM_END_ADDRESS (M4_ITCM_SRAM_START_ADDRESS + M4_ITCM_SRAM_SIZE - 1)

#define M4_DTCM_SRAM_START_ADDRESS (0x20000000)
#define M4_DTCM_SRAM_SIZE (SIZE_128KB)
#define M4_DTCM_SRAM_END_ADDRESS (M4_DTCM_SRAM_START_ADDRESS + M4_DTCM_SRAM_SIZE - 1)

/* OCRAM(M4), OCRAM1&2, and OCRAM(M7) */
#define OCRAM_START_ADDRESS (0x20200000u)
#define OCRAM_SIZE (SIZE_2MB)
#define OCRAM_END_ADDRESS (OCRAM_START_ADDRESS + OCRAM_SIZE - 1)

#define M4_OCRAM_START_ADDRESS (0x20200000u)
#define M4_OCRAM_SIZE (SIZE_256KB)
#define M4_OCRAM_END_ADDRESS (M4_OCRAM_START_ADDRESS + M4_OCRAM_SIZE - 1)

/* OCRAM1 */
#define OCRAM1_START_ADDRESS (0x20240000u)
#define OCRAM1_SIZE (SIZE_512KB);
#define OCRAM1_END_ADDRESS (OCRAM1_START_ADDRESS + OCRAM1_SIZE - 1)

/* OCRAM(M7) */
#define OCRAM2_START_ADDRESS (0x202C0000u)
#define OCRAM2_SIZE (SIZE_512KB);
#define OCRAM2_END_ADDRESS (OCRAM2_START_ADDRESS + OCRAM2_SIZE - 1)

/* OCRAM(M7) */
#define M7_OCRAM_START_ADDRESS (0x20360000u)
#define M7_OCRAM_SIZE (SIZE_640KB);
#define M7_OCRAM_END_ADDRESS (M7_OCRAM_START_ADDRESS + M7_OCRAM_SIZE - 1)

/* ======================== FLEXSPI region definitions ================================= */
#define FLEXSPI1_AMBA_START_ADDRESS (0x30000000u)
#define FLEXSPI1_AMBA_SIZE (SIZE_256MB)
#define FLEXSPI1_AMBA_END_ADDRESS (FLEXSPI1_AMBA_START_ADDRESS + FLEXSPI1_AMBA_SIZE - 1)

// Note: This region is only valid on CM4 core
#define M4_FLEXSPI1_ALIAS_START_ADDRESS (0x08000000u)
#define M4_FLEXSPI1_ALIAS_SIZE (SIZE_256MB)
#define M4_FLEXSPI1_ALIAS_END_ADDRESS (M4_FLEXSPI1_ALIAS_START_ADDRESS + M4_FLEXSPI1_ALIAS_SIZE - 1)

#define FLEXSPI2_AMBA_START_ADDRESS (0x60000000u)
#define FLEXSPI2_AMBA_SIZE (SIZE_504MB)
#define FLEXSPI2_AMBA_END_ADDRESS (FLEXSPI2_AMBA_START_ADDRESS + FLEXSPI2_AMBA_SIZE - 1)

/* ======================== CAAM region definitions ================================= */
#define CAAM_SECURE_RAM_START_ADDRESS (0x00280000u)
#define CAAM_SECURE_RAM_SIZE (SIZE_64KB)
#define CAAM_SECURE_RAM_END_ADDRESS (CAAM_SECURE_RAM_START_ADDRESS + CAAM_SECURE_RAM_SIZE - 1)

#endif // __MEMORY_MAP_H__

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
