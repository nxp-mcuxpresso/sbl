/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "fsl_sd.h"
#include "fsl_debug_console.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "board.h"
#include "sdmmc_config.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_common.h"

/* MCUboot */
#include "flexspi_flash.h"
#include "sysflash.h"
#include "bootutil/image.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
* @brief wait card insert function.
*/
static status_t sdcardWaitCardInsert(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if(MCUBOOT_APP == 1)
#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__)
    __attribute__((section(".head.conf")))
#elif defined(__ICCARM__)
#pragma location=".head.conf"
#endif
const uint8_t head_data[1024] = {0x00};
#endif

extern flash_ops_s mcuboot_flash;

static FATFS g_fileSystem; /* File system object */
//static FIL g_fileObject;   /* File object */

struct image_header *pri_hdr = (struct image_header *)(FLASH_DEVICE_BASE_ADDR + FLASH_AREA_IMAGE_1_OFFSET);
/*******************************************************************************
 * Code
 ******************************************************************************/
void local_version(void)
{
    if (pri_hdr->ih_magic == IMAGE_MAGIC)
        PRINTF("Local image version: %d.%d.%d\r\n", pri_hdr->ih_ver.iv_major,
	                                                pri_hdr->ih_ver.iv_minor,
                                                    pri_hdr->ih_ver.iv_revision);
    else
        PRINTF("Failed to get version number\r\n");
}

//extern void parser(char ch);
extern void os_heap_init(void);
/*!
 * @brief Main function
 */
int fatfs_sdcard_main(void)
{
    FRESULT error;
    const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};
    volatile bool isForce = false;

    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    sbl_flash_init();
    os_heap_init();
    local_version();

    PRINTF("\r\nThis example to demonstrate how to use SD card to implement ota.\r\n");

    PRINTF("\r\nPlease insert a card into board.\r\n");

    if (sdcardWaitCardInsert() != kStatus_Success)
    {
        return -1;
    }

    if (f_mount(&g_fileSystem, driverNumberBuffer, 0U))
    {
        PRINTF("Mount volume failed.\r\n");
        return -1;
    }

#if (FF_FS_RPATH >= 2U)
    error = f_chdrive((char const *)&driverNumberBuffer[0U]);
    if (error)
    {
        PRINTF("Change drive failed.\r\n");
        return -1;
    }
#endif


    PRINTF("\r\nPlease input a command......\r\n");
    PRINTF("command >");
    while (true) {
        //parser(GETCHAR());
//        *ch = GETCHAR();
//        PUTCHAR(ch);
    }
}

static status_t sdcardWaitCardInsert(void)
{
    BOARD_SD_Config(&g_sd, NULL, BOARD_SDMMC_SD_HOST_IRQ_PRIORITY, NULL);

    /* SD host init function */
    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        PRINTF("\r\nSD host init fail\r\n");
        return kStatus_Fail;
    }
    /* power off card */
    SD_SetCardPower(&g_sd, false);

    /* wait card insert */
    if (SD_PollingCardInsert(&g_sd, kSD_Inserted) == kStatus_Success)
    {
        PRINTF("\r\nCard inserted.\r\n");
        /* power on the card */
        SD_SetCardPower(&g_sd, true);
    }
    else
    {
        PRINTF("\r\nCard detect fail.\r\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
}
