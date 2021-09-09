/*
 * Copyright 2018 - 2021 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bootloader.h"
#if BL_FEATURE_FLEXSPI_NOR_MODULE
//#include "flexspi/fsl_flexspi.h"
#endif
#include "fusemap.h"
#include "memory_config.h"
#if BL_FEATURE_OTFAD_MODULE
#include "ocotp/fsl_ocotp.h"
#endif
#include "peripherals_pinmux.h"

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FREQ_396MHz (396000000U)
#define FREQ_480MHz (480000000U)
#define FREQ_528MHz (528000000U)
#define FREQ_24MHz (24000000U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void update_memory_map(void);

/*******************************************************************************
 * Codes
 ******************************************************************************/
bool is_boot_pin_asserted(void)
{
    // Boot pin for Flash only target
    return false;
}

//!@brief Get Primary boot device type
uint32_t get_primary_boot_device(void)
{
    uint8_t flash_device = 0xFF;

    switch (FUSE_BOOT_DEVICE_VALUE)
    {
        /* 0000 */
        case 0x0:
            flash_device = kBootDevice_FlexSpiNOR; // FlexSPI NOR
            break;
        /* 001x */
        case 0x2:
        case 0x3:
            flash_device = kBootDevice_SemcNAND; // Semc NAND
        /* 01xx */
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
            flash_device = kBootDevice_SD; // SD
            break;
        /* 10xx */
        case 0x8:
        case 0x9:
        case 0xa:
        case 0xb:
            flash_device = kBootDevice_MMC; // MMC/eMMC
            break;
        /* 11xx */
        case 0xc:
        case 0xd:
        case 0xe:
        case 0xf:
            flash_device = kBootDevice_FlexSpiNAND; // MMC/eMMC
            break;
    }

    return flash_device;
}

#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    return size;
}

#endif // __ICCARM__

void update_available_peripherals()
{
    // Flashloader always uses NXP Kboot PID and VID.
}

void update_specific_properties(void)
{
    property_store_t *store = g_bootloaderContext.propertyInterface->store;

    store->UniqueDeviceId.uid[0] = FUSE_DEVICE_UUID_WORD0_VAL;
    store->UniqueDeviceId.uid[1] = FUSE_DEVICE_UUID_WORD1_VAL;
}

void init_system(void)
{
    SCB_DisableDCache();
    __ISB();
    __DSB();
}

void deinit_system(void)
{
    SCB_EnableDCache();
    __ISB();
    __DSB();
}

void init_hardware(void)
{
    init_system();

    // Configure Clocks
    configure_clocks(kClockOption_EnterBootloader);

    // Update memory map according to actual Fuse definitions
    update_memory_map();

    CLOCK_EnableClock(kCLOCK_Usb);
}

void deinit_hardware(void)
{
    CLOCK_DisableClock(kCLOCK_Usb);

    deinit_system();
}

//!@brief Get the hab status.
habstatus_option_t get_hab_status(void)
{
    habstatus_option_t habStatus = kHabStatus_Close;

    if (FUSE_FIELD_RETURN_VALUE)
    {
        habStatus = kHabStatus_FA;
    }
    else
    {
        if (FUSE_SEC_CONFIG_VALUE == 1)
        {
            habStatus = kHabStatus_Open;
        }
        else if (FUSE_SEC_CONFIG_VALUE < 1)
        {
            habStatus = kHabStatus_Fab;
        }
        else
        {
            habStatus = kHabStatus_Close;
        }
    }

    return habStatus;
}

bool is_cm4_boot(void)
{
    enum
    {
        kPartNumber_CM4 = 0xC24,
        kPartNumber_CM7 = 0xC27,
    };

    uint32_t partNumber = (SCB->CPUID & SCB_CPUID_PARTNO_Msk) >> SCB_CPUID_PARTNO_Pos;
    if (partNumber == kPartNumber_CM4)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void update_memory_map(void)
{
    typedef struct
    {
        uint16_t dtcmSize;
        uint16_t itcmSize;
        uint16_t ocramSize;
    } flexram_cfg_t;

    const flexram_cfg_t k_flexramCfgList[] = {
        { 256, 256, 0 },   { 320, 192, 0 },  { 384, 128, 0 },  { 448, 64, 0 },    { 512, 0, 0 },    { 192, 320, 0 },
        { 128, 384, 0 },   { 64, 448, 0 },   { 0, 512, 0 },    { 256, 192, 64 },  { 320, 128, 64 }, { 384, 64, 64 },
        { 448, 0, 64 },    { 192, 256, 64 }, { 128, 320, 64 }, { 64, 384, 64 },   { 0, 448, 64 },   { 192, 192, 128 },
        { 256, 128, 128 }, { 328, 64, 128 }, { 384, 0, 128 },  { 128, 256, 128 }, { 64, 320, 128 }, { 0, 384, 128 },
        { 192, 128, 192 }, { 256, 64, 192 }, { 320, 0, 192 },  { 128, 192, 192 }, { 64, 256, 192 }, { 0, 320, 192 },
        { 128, 128, 256 }, { 192, 64, 256 }, { 256, 0, 256 },  { 64, 192, 256 },  { 0, 256, 256 },  { 128, 64, 320 },
        { 192, 0, 320 },   { 64, 128, 320 }, { 0, 192, 320 },  { 64, 64, 384 },   { 128, 0, 384 },  { 0, 128, 384 },
        { 64, 0, 448 },    { 0, 64, 448 },   { 0, 0, 512 },
    };

    uint32_t flexramCfgIndex = FUSE_FLEXRAM_CFG_VALUE;
    if (flexramCfgIndex >= ARRAY_SIZE(k_flexramCfgList))
    {
        // Reset the device and try to reboot again in case this issue is caused by abnormals
        NVIC_SystemReset();
    }
    debug_printf("FLEXRAM config, index = %x, dtcmSize = %x, itcmSize = %x, ocramSize = %x\n", flexramCfgIndex,
                 k_flexramCfgList[flexramCfgIndex].dtcmSize, k_flexramCfgList[flexramCfgIndex].itcmSize,
                 k_flexramCfgList[flexramCfgIndex].ocramSize);

    if (is_cm4_boot())
    {
        // ITCM + DTCM
        /*
         * Note: DTCM is next to ITCM, merge DTCM into ITCM to make boundary-crossing access pass.
         */
        g_memoryMap[kIndexITCM].startAddress = M4_TCM_SRAM_START_ADDRESS;
        g_memoryMap[kIndexITCM].endAddress = M4_TCM_SRAM_END_ADDRESS;

        // Duplicated index, the same with kIndexITCM.
        g_memoryMap[kIndexDTCM].startAddress = M4_TCM_SRAM_START_ADDRESS;
        g_memoryMap[kIndexDTCM].endAddress = M4_TCM_SRAM_END_ADDRESS;

        // OCRAM, in CM7, RAM address 0x2020_0000 - 0x2023_ffff is used as ITCM and DTCM,
        // So SW should avoid using above address
        uint32_t flexramOcramSize = 1024u * k_flexramCfgList[flexramCfgIndex].ocramSize;
        uint32_t fixedOcramSize = 1536u * 1024u;
        g_memoryMap[kIndexOCRAM].startAddress = OCRAM_START_ADDRESS;
        g_memoryMap[kIndexOCRAM].endAddress =
            g_memoryMap[kIndexOCRAM].startAddress + fixedOcramSize + flexramOcramSize - 1;

        debug_printf("CM4 boot, RAM region list:\n");
        debug_printf("ITCM start=%x, end=%x:\n", g_memoryMap[kIndexITCM].startAddress,
                     g_memoryMap[kIndexITCM].endAddress);
        debug_printf("DTCM start=%x, end=%x:\n", g_memoryMap[kIndexDTCM].startAddress,
                     g_memoryMap[kIndexDTCM].endAddress);
        debug_printf("OCRAM start=%x, end=%x:\n", g_memoryMap[kIndexOCRAM].startAddress,
                     g_memoryMap[kIndexOCRAM].endAddress);
    }
    else
    {
//#warning Need to ajust the memory map according to the FLEXRAM CFG settings in FUSEMAP

        uint32_t itcmSize = 1024u * k_flexramCfgList[flexramCfgIndex].itcmSize;
        uint32_t dtcmSize = 1024u * k_flexramCfgList[flexramCfgIndex].dtcmSize;
        uint32_t flexramOcramSize = 1024u * k_flexramCfgList[flexramCfgIndex].ocramSize;

        // ITCM
        g_memoryMap[kIndexITCM].startAddress = M7_ITCM_SRAM_START_ADDRESS;
        g_memoryMap[kIndexITCM].endAddress = g_memoryMap[kIndexITCM].startAddress + itcmSize - 1;

        // DTCM
        g_memoryMap[kIndexDTCM].startAddress = M7_DTCM_SRAM_START_ADDRESS;
        g_memoryMap[kIndexDTCM].endAddress = g_memoryMap[kIndexDTCM].startAddress + dtcmSize - 1;

// OCRAM
//#warning Need to update the fixedOcramSize if the ECC is enabled
        uint32_t fixedOcramSize = 1536u * 1024u;
        g_memoryMap[kIndexOCRAM].startAddress = OCRAM_START_ADDRESS;
        g_memoryMap[kIndexOCRAM].endAddress =
            g_memoryMap[kIndexOCRAM].startAddress + fixedOcramSize + flexramOcramSize - 1;

        debug_printf("CM7 boot, RAM region list:\n");
        debug_printf("ITCM start=%x, end=%x:\n", g_memoryMap[kIndexITCM].startAddress,
                     g_memoryMap[kIndexITCM].endAddress);
        debug_printf("DTCM start=%x, end=%x:\n", g_memoryMap[kIndexDTCM].startAddress,
                     g_memoryMap[kIndexDTCM].endAddress);
        debug_printf("OCRAM start=%x, end=%x:\n", g_memoryMap[kIndexOCRAM].startAddress,
                     g_memoryMap[kIndexOCRAM].endAddress);
    }
}

int memset_s(void *s, size_t smax, int c, size_t n)
{
    if (n > smax)
    {
        return 1;
    }
    memset(s, c, n);
    return 0;
}

bool isp_cleanup_exit(bool *isInfiniteIsp)
{
    // GPR0-31 can not be wrote by default, so we switch to GPR32
    uint32_t flag = (IOMUXC_SNVS_GPR->GPR32 & (~IOMUXC_SNVS_GPR_GPR32_LOCK_MASK))>> 1;
    switch (flag)
    {
        case CLEANUP_SBL_TO_ISP:
            *isInfiniteIsp = true;
            flag = 0x0;
            break;
        case CLEANUP_ISP_TO_SBL:
        default:
            break;
    }
    IOMUXC_SNVS_GPR->GPR32 &= IOMUXC_SNVS_GPR_GPR32_LOCK_MASK;
    return flag;
}

void isp_cleanup_enter(uint32_t flag)
{
    IOMUXC_SNVS_GPR->GPR32 = (uint32_t)(((uint16_t)flag) << 1);
    NVIC_SystemReset();
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
