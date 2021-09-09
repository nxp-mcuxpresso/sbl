/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_flashiap.h"

#define HZ_TO_KHZ_DIV 1000

/*******************************************************************************
 * Code
 ******************************************************************************/

static status_t translate_iap_status(uint32_t status)
{
    /* Translate IAP return code to sdk status code */
    if (status == kStatus_Success)
    {
        return status;
    }
    else
    {
        return MAKE_STATUS(kStatusGroup_FLASHIAP, status);
    }
}

status_t FLASHIAP_PrepareSectorForWrite(uint32_t startSector, uint32_t endSector)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_PrepareSectorforWrite;
    command[1] = startSector;
    command[2] = endSector;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_CopyRamToFlash(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes, uint32_t systemCoreClock)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_CopyRamToFlash;
    command[1] = dstAddr;
    command[2] = (uint32_t)srcAddr;
    command[3] = numOfBytes;
    command[4] = systemCoreClock / HZ_TO_KHZ_DIV;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_EraseSector(uint32_t startSector, uint32_t endSector, uint32_t systemCoreClock)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_EraseSector;
    command[1] = startSector;
    command[2] = endSector;
    command[3] = systemCoreClock / HZ_TO_KHZ_DIV;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_ErasePage(uint32_t startPage, uint32_t endPage, uint32_t systemCoreClock)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_ErasePage;
    command[1] = startPage;
    command[2] = endPage;
    command[3] = systemCoreClock / HZ_TO_KHZ_DIV;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_BlankCheckSector(uint32_t startSector, uint32_t endSector)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_BlankCheckSector;
    command[1] = startSector;
    command[2] = endSector;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_Compare(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_Compare;
    command[1] = dstAddr;
    command[2] = (uint32_t)srcAddr;
    command[3] = numOfBytes;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}
