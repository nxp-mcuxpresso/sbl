/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "fsl_flashiap.h"
#include "usb_flash.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief flash configuration */
uint32_t startSector;
uint32_t endSector;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief USB flash erasing function.
 *
 * This function erases the flash area from start address to the end.
 *
 * @param address  The start address.
 * @param address  The erase size.
 *
 * @return A FLASH error or kStatus_FLASH_Success.
 */
usb_flash_status_t USB_FlashCalcuateSectorNum(uint32_t address, uint32_t size)
{
    usb_flash_status_t flashstatus = kStatus_USB_FlashSuccess;
    if (!size)
    {
        return kStatus_USB_FlashErrorUnknown;
    }
    startSector = address / (FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES);
    if ((address + size) / (FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES) >= (startSector))
    {
        endSector = (address + size) / (FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES);
    }
    else
    {
        endSector = startSector;
    }

    return flashstatus;
}
/*!
 * @brief USB flash initialization function.
 *
 * This function initializes the flash driver structure and variables.
 *
 * @return A FLASH error or kStatus_FLASH_Success.
 */
usb_flash_status_t USB_FlashInit(void)
{
    usb_flash_status_t status = kStatus_USB_FlashSuccess;
    return status;
}
/*!
 * @brief USB flash erasing function.
 *
 * This function erases the flash area from start address to the end.
 *
 * @param address  The start address.
 * @param address  The erase size.
 *
 * @return A FLASH error or kStatus_FLASH_Success.
 */
usb_flash_status_t USB_FlashErase(uint32_t address, uint32_t size)
{
    usb_flash_status_t flashstatus = kStatus_USB_FlashSuccess;
    uint32_t status;
    USB_FlashCalcuateSectorNum(address, size);
    FLASHIAP_PrepareSectorForWrite(startSector, endSector);
    status = FLASHIAP_EraseSector(startSector, endSector, SystemCoreClock);

    if (kStatus_FLASHIAP_Success != status)
    {
        flashstatus = kStatus_USB_FlashErrorErase;
        return flashstatus;
    }
    return flashstatus;
}

/*!
 * @brief USB flash programming function.
 *
 * This function program flash with data at locations passed in through parameters.
 *
 * @param address The start address to be programmed.
 * @param buffer  Pointer to buffer data.
 * @param length  The length of data in byte.
 *
 * @return A FLASH error or kStatus_FLASH_Success.
 */
usb_flash_status_t USB_FlashProgram(uint32_t address, uint8_t *buffer, uint32_t length)
{
    usb_flash_status_t flashstatus = kStatus_USB_FlashSuccess;
    uint32_t status;
    uint32_t dataLen = length;
    if (length % (FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES))
    {
        length = (length / FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES + 1) * (FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES);
    }

    FLASHIAP_PrepareSectorForWrite(startSector, endSector);
    {
        FLASHIAP_CopyRamToFlash(address, (uint32_t *)&buffer[0], length, SystemCoreClock);
    }
    status = FLASHIAP_Compare(address, (uint32_t *)&buffer[0], dataLen);
    if (status != kStatus_FLASHIAP_Success)
    {
        flashstatus = kStatus_USB_FlashErrorProgramVerify;
    }
    return flashstatus;
}
