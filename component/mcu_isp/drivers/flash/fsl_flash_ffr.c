/*
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_flash_ffr.h"
#include "bootloader/bl_context.h"
//#include "mbedtls/sha256.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

// Get cfpa version from specified page
static void ffr_get_cfpa_version(flash_config_t *config, uint32_t ffrPageOffset, bool needNewer);
// Check hash of specified memory range
static bool ffr_is_range_hash_check_pass(flash_config_t *config,
                                         uint8_t *start,
                                         uint32_t lenInBytes,
                                         uint8_t *hashDigestStart);
// Check hash of specified page
static bool ffr_is_page_hash_check_pass(flash_config_t *config, uint8_t *page_data);
// Configure flash firewall
static void ffr_configure_firewall(ffr_bank_type_t bankType, bool setEnable);
// Check to see whether bank is locked
static bool ffr_is_firewall_locked(ffr_bank_type_t bankType);
// Copy memory by ReadSingleWord command
static status_t ffr_ipcmd_memcpy(flash_config_t *config, uint8_t *dest, uint8_t *src, uint32_t size);
// Compare memory by ReadSingleWord command
static uint32_t ffr_ipcmd_memcmp(flash_config_t *config, uint8_t *dest, uint8_t *src, uint32_t size);

static status_t ffr_check_cfpa_revision(flash_config_t *config,
                                        const cfpa_cfg_info_t *formerCfpa,
                                        const cfpa_cfg_info_t *newCfpa);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/* @brief Flash FFR size in bytes */
#define FSL_FEATURE_SYSCON_FLASH_FFR_SIZE_BYTES (8704)

status_t FFR_Init(flash_config_t *config)
{
    config->ffrConfig.ffrBlockBase = FSL_FEATURE_SYSCON_FLASH_SIZE_BYTES - FSL_FEATURE_SYSCON_FLASH_FFR_SIZE_BYTES;
    config->ffrConfig.ffrTotalSize = FSL_FEATURE_SYSCON_FLASH_FFR_SIZE_BYTES;
    config->ffrConfig.ffrPageSize = FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES;
    // Assume that CFPA is blank and latest CFPA page is the first one, version is 0.
    config->ffrConfig.cfpaPageVersion = 0;
    config->ffrConfig.cfpaPageOffset = kFfrPageOffset_CFPA_Cfg;

    return kStatus_FLASH_Success;
}

/*
status_t FFR_Deinit(flash_config_t *config)
{
    // Enable firewall for all flash banks
    ffr_configure_firewall(kFFR_BankTypeBank2_CFPA, true);
    ffr_configure_firewall(kFFR_BankTypeBank1_CMPA, true);
    ffr_configure_firewall(kFFR_BankTypeBank0_NMPA, true);

    return kStatus_FLASH_Success;
}

status_t FFR_CustomerPagesInit(flash_config_t *config)
{
    status_t status = kStatus_Fail;
    cfpa_cfg_info_t newCfpaCfgInfo;
    cfpa_cfg_info_t formerCfpaInfo;

    // There will be a page version field used to locate active page. The page
    // with highest numerical version value is deemed as active page.
    uint32_t scratchPageIndex =
        config->ffrConfig.ffrBlockBase / config->ffrConfig.ffrPageSize + kFfrPageOffset_CFPA_Scratch;
    uint32_t scratchPageAddress = scratchPageIndex * config->ffrConfig.ffrPageSize;
    // Search all three CFPA pages to find out the latest one
    ffr_get_cfpa_version(config, kFfrPageOffset_CFPA_Cfg, true);
    ffr_get_cfpa_version(config, kFfrPageOffset_CFPA_CfgPong, true);
    ffr_get_cfpa_version(config, kFfrPageOffset_CFPA_Scratch, true);
    // If the latest one is Scratch page, that means we need to do update work (ping-pong)
    if (config->ffrConfig.cfpaPageOffset == kFfrPageOffset_CFPA_Scratch)
    {
        // Get the newer version in the ping-pong pages
        config->ffrConfig.cfpaPageVersion = 0;
        config->ffrConfig.cfpaPageOffset = kFfrPageOffset_CFPA_Cfg;
        ffr_get_cfpa_version(config, kFfrPageOffset_CFPA_Cfg, true);
        ffr_get_cfpa_version(config, kFfrPageOffset_CFPA_CfgPong, true);
        uint32_t newerCfpaPingpongPageAddress =
            config->ffrConfig.ffrBlockBase + config->ffrConfig.cfpaPageOffset * config->ffrConfig.ffrPageSize;
        if (ffr_is_firewall_locked(kFFR_BankTypeBank2_CFPA))
        {
            return kStatus_FLASH_FfrBankIsLocked;
        }
        else
        {
            // Disable flash firewall
            ffr_configure_firewall(kFFR_BankTypeBank2_CFPA, false);
        }

        // Cleanup temp page buffer
        memset(&newCfpaCfgInfo, 0x0, sizeof(cfpa_cfg_info_t));
        // Get latest CFPA page data from Scratch page
        status = FLASH_Read(config, scratchPageAddress, (uint8_t *)&newCfpaCfgInfo, sizeof(cfpa_cfg_info_t));
        if (status != kStatus_FLASH_Success)
        {
            return status;
        }

        // Intentially set the former CFPA to highest version
        cfpa_cfg_info_t formerCfpaInfo;
        memset(&formerCfpaInfo, 0xff, sizeof(cfpa_cfg_info_t));
        status = FLASH_Read(config, newerCfpaPingpongPageAddress, (uint8_t *)&formerCfpaInfo, sizeof(cfpa_cfg_info_t));
        if (status != kStatus_FLASH_Success)
        {
            return status;
        }

        // Check the validity of CFPA (including digest, rollback protection)
        status = ffr_check_cfpa_revision(config, &formerCfpaInfo, &newCfpaCfgInfo);
        if (status != kStatus_Success)
        {
            return status;
        }

        // Double check here for better resistance protection on glitch-attack
        if (formerCfpaInfo.enableFaMode != 0)
        {
            return kStatus_FLASH_CfpaVersionRollbackDisallowed;
        }

        // Erase the oldest CFPA page and Program new CFPA page in flash
        config->ffrConfig.cfpaPageVersion = (uint32_t)(~0);
        ffr_get_cfpa_version(config, kFfrPageOffset_CFPA_Cfg, false);
        ffr_get_cfpa_version(config, kFfrPageOffset_CFPA_CfgPong, false);
        uint32_t olderCfpaPingPongPageAddress =
            config->ffrConfig.ffrBlockBase + config->ffrConfig.cfpaPageOffset * config->ffrConfig.ffrPageSize;
        status = FLASH_Erase(config, olderCfpaPingPongPageAddress, config->ffrConfig.ffrPageSize, kFLASH_ApiEraseKey);
        if (status != kStatus_FLASH_Success)
        {
            return status;
        }

        // Clear cmpaProgInProgress field as it is just temp flag used for CMPA update
        if (newCfpaCfgInfo.cmpaProgInProgress != kFfrCmpaProgStatus_Idle)
        {
            newCfpaCfgInfo.cmpaProgInProgress = kFfrCmpaProgStatus_Idle;
            // Re-Calculate Hash as the page data is changed
            {
                mbedtls_sha256_context shaCtx;
                // Compute HASH of page data
                mbedtls_sha256_init(&shaCtx);
                mbedtls_sha256_starts(&shaCtx, false);
                mbedtls_sha256_update(&shaCtx, (unsigned char *)&newCfpaCfgInfo,
                                      config->ffrConfig.ffrPageSize - sizeof(newCfpaCfgInfo.sha256));
                mbedtls_sha256_finish(&shaCtx, (uint8_t *)&newCfpaCfgInfo.sha256[0]);
                mbedtls_sha256_free(&shaCtx);
            }
        }
        status = FLASH_Program(config, olderCfpaPingPongPageAddress, (uint8_t *)&newCfpaCfgInfo,
                               config->ffrConfig.ffrPageSize);
        if (status != kStatus_FLASH_Success)
        {
            return status;
        }
        config->ffrConfig.cfpaPageVersion = newCfpaCfgInfo.version;
    }

    return kStatus_FLASH_Success;
}

status_t FFR_InfieldPageWrite(flash_config_t *config, uint8_t *page_data, uint32_t valid_len)
{
    cfpa_cfg_info_t cfpaCfgInfo;

    // Check length to see whether it equals to page size
    if ((valid_len != config->ffrConfig.ffrPageSize) &&
        (valid_len != config->ffrConfig.ffrPageSize - sizeof(cfpaCfgInfo.sha256)))
    {
        return kStatus_FLASH_SizeError;
    }
    // Cleanup temp page buffer
    memset(&cfpaCfgInfo, 0x0, sizeof(cfpa_cfg_info_t));
    // Copy customer CFPA data to temp page buffer
    memcpy(&cfpaCfgInfo, page_data, valid_len);

    // Check version to see whether it is newest CFPA page
    if ((cfpaCfgInfo.version > config->ffrConfig.cfpaPageVersion) ||
        ((cfpaCfgInfo.version == config->ffrConfig.cfpaPageVersion) &&
         (cfpaCfgInfo.cmpaProgInProgress == kFfrCmpaProgStatus_InProgress)))
    {
        uint32_t scratchPageIndex =
            config->ffrConfig.ffrBlockBase / config->ffrConfig.ffrPageSize + kFfrPageOffset_CFPA_Scratch;
        uint32_t scratchPageAddress = scratchPageIndex * config->ffrConfig.ffrPageSize;
        // Check the cmpa update-in-progress flag, if yes, keep it
        if (cfpaCfgInfo.cmpaProgInProgress != kFfrCmpaProgStatus_InProgress)
        {
            uint32_t cmpaProgInProgress;
            uint32_t offset = (uint8_t *)&cfpaCfgInfo.cmpaProgInProgress - (uint8_t *)&cfpaCfgInfo;
            status_t status = FLASH_Read(config, scratchPageAddress + offset, (uint8_t *)&cmpaProgInProgress,
                                         sizeof(cmpaProgInProgress));
            if ((status == kStatus_FLASH_Success) && (cmpaProgInProgress == kFfrCmpaProgStatus_InProgress))
            {
                cfpaCfgInfo.cmpaProgInProgress = kFfrCmpaProgStatus_InProgress;
            }
        }

        // Record latest CFPA page version
        config->ffrConfig.cfpaPageVersion = cfpaCfgInfo.version;
        // Store customer CFPA data into CFPA scratch page
        config->ffrConfig.cfpaPageOffset = kFfrPageOffset_CFPA_Scratch;

        if (ffr_is_firewall_locked(kFFR_BankTypeBank2_CFPA))
        {
            return kStatus_FLASH_FfrBankIsLocked;
        }
        else
        {
            // Disable flash firewall
            ffr_configure_firewall(kFFR_BankTypeBank2_CFPA, false);
        }

        // Erase CFPA scratch page area
        status_t status = FLASH_Erase(config, scratchPageAddress, config->ffrConfig.ffrPageSize, kFLASH_ApiEraseKey);
        if (status != kStatus_FLASH_Success)
        {
            return status;
        }

        // Calculate Hash for CFPA page, so customer doesn't need to care about HASH calculation
        {
            mbedtls_sha256_context shaCtx;
            // Compute HASH of page data
            mbedtls_sha256_init(&shaCtx);
            mbedtls_sha256_starts(&shaCtx, false);
            mbedtls_sha256_update(&shaCtx, (unsigned char *)&cfpaCfgInfo,
                                  config->ffrConfig.ffrPageSize - sizeof(cfpaCfgInfo.sha256));
            mbedtls_sha256_finish(&shaCtx, (uint8_t *)&cfpaCfgInfo.sha256[0]);
            mbedtls_sha256_free(&shaCtx);
        }

        // Program customer CFPA data to CFPA scratch page
        status = FLASH_Program(config, scratchPageAddress, (uint8_t *)&cfpaCfgInfo, config->ffrConfig.ffrPageSize);
        if (status != kStatus_FLASH_Success)
        {
            return status;
        }
    }
    else
    {
        return kStatus_FLASH_OutOfDateCfpaPage;
    }

    return kStatus_FLASH_Success;
}
*/

// Generic read function, used by customer to read data stored in 'Customer In-field Page'.
status_t FFR_GetCustomerInfieldData(flash_config_t *config, uint8_t *pData, uint32_t offset, uint32_t len)
{
    uint32_t cfpaPageSize = config->ffrConfig.ffrPageSize;
    uint32_t cfpaPageAddress =
        config->ffrConfig.ffrBlockBase + config->ffrConfig.cfpaPageOffset * config->ffrConfig.ffrPageSize;

    // Check arguments
    if ((offset + len) > cfpaPageSize)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    return ffr_ipcmd_memcpy(config, pData, (uint8_t *)(cfpaPageAddress + offset), len);
}

/*
// This routine will Check if Cust Factory Page Update is in progress.
bool FFR_IsCmpaCfgPageUpdateInProgress(flash_config_t *config)
{
    status_t status = kStatus_Fail;
    bool isProgressFlagSet = false;
    cfpa_cfg_info_t cfpaCfgInfo;

    // Cleanup temp page buffer
    memset(&cfpaCfgInfo, 0x0, sizeof(cfpa_cfg_info_t));
    uint32_t scratchPageIndex =
        config->ffrConfig.ffrBlockBase / config->ffrConfig.ffrPageSize + kFfrPageOffset_CFPA_Scratch;
    uint32_t scratchPageAddress = scratchPageIndex * config->ffrConfig.ffrPageSize;
    // Get current CFPA page
    status = FLASH_Read(config, scratchPageAddress, (uint8_t *)&cfpaCfgInfo, sizeof(cfpa_cfg_info_t));
    if (status == kStatus_FLASH_Success)
    {
        if (cfpaCfgInfo.cmpaProgInProgress == kFfrCmpaProgStatus_InProgress)
        {
            isProgressFlagSet = true;
        }
    }

    return isProgressFlagSet;
}

status_t FFR_RecoverCmpaCfgPage(flash_config_t *config)
{
    cmpa_cfg_info_t cmpaCfgInfo;
    uint32_t cmpaCfgPageSize = config->ffrConfig.ffrPageSize;
    uint32_t cmpaCfgPageAddress = config->ffrConfig.ffrBlockBase + kFfrPageOffset_CMPA_Cfg * cmpaCfgPageSize;

    memset(&cmpaCfgInfo, 0x0, sizeof(cmpa_cfg_info_t));
    if (ffr_is_firewall_locked(kFFR_BankTypeBank1_CMPA))
    {
        return kStatus_FLASH_FfrBankIsLocked;
    }
    else
    {
        // Disable flash firewall
        ffr_configure_firewall(kFFR_BankTypeBank1_CMPA, false);
    }
    status_t status = FLASH_Erase(config, cmpaCfgPageAddress, cmpaCfgPageSize, kFLASH_ApiEraseKey);
    if (status != kStatus_FLASH_Success)
    {
        return status;
    }
    // Program recovery data (0x00s)
    status = FLASH_Program(config, cmpaCfgPageAddress, (uint8_t *)&cmpaCfgInfo, cmpaCfgPageSize);
    if (status != kStatus_FLASH_Success)
    {
        return status;
    }

    return kStatus_FLASH_Success;
}

// This routine will do preparation work for Cust Factory Page Update.
status_t FFR_ProcessCmpaCfgPageUpdate(flash_config_t *config, cmpa_prog_process_t option)
{
    status_t status = kStatus_Fail;

    if (option == kFfrCmpaProgProcess_Pre)
    {
        cfpa_cfg_info_t cfpaCfgInfo;

        // Cleanup temp page buffer
        memset(&cfpaCfgInfo, 0x0, sizeof(cfpa_cfg_info_t));

        uint32_t cfpaPageSize = config->ffrConfig.ffrPageSize;
        uint32_t cfpaPageAddress =
            config->ffrConfig.ffrBlockBase + config->ffrConfig.cfpaPageOffset * config->ffrConfig.ffrPageSize;
        // Get current CFPA page
        status = FLASH_Read(config, cfpaPageAddress, (uint8_t *)&cfpaCfgInfo, sizeof(cfpa_cfg_info_t));
        if (status != kStatus_FLASH_Success)
        {
            return status;
        }
        // Set this field (cmpaProgInProgress) with magic value (0x5CC55AA5)
        cfpaCfgInfo.cmpaProgInProgress = kFfrCmpaProgStatus_InProgress;
        // Update CFPA page
        status = FFR_InfieldPageWrite(config, (uint8_t *)&cfpaCfgInfo, sizeof(cfpa_cfg_info_t));
    }
    else if (option == kFfrCmpaProgProcess_Post)
    {
        uint32_t scratchPageIndex =
            config->ffrConfig.ffrBlockBase / config->ffrConfig.ffrPageSize + kFfrPageOffset_CFPA_Scratch;
        uint32_t scratchPageAddress = scratchPageIndex * config->ffrConfig.ffrPageSize;
        cfpa_cfg_info_t *cfpaCfgInfo = (cfpa_cfg_info_t *)0;
        uint32_t flagOffset = (uint8_t *)&cfpaCfgInfo->cmpaProgInProgress - (uint8_t *)cfpaCfgInfo;
        uint32_t cmpaProgInProgressFlag = 0;

        // Get cmpaProgInProgressFlag from scratch page
        status = FLASH_Read(config, scratchPageAddress + flagOffset, (uint8_t *)&cmpaProgInProgressFlag,
                            sizeof(cmpaProgInProgressFlag));
        if (status != kStatus_FLASH_Success)
        {
            return status;
        }

        if (cmpaProgInProgressFlag == kFfrCmpaProgStatus_InProgress)
        {
            if (ffr_is_firewall_locked(kFFR_BankTypeBank2_CFPA))
            {
                return kStatus_FLASH_FfrBankIsLocked;
            }
            else
            {
                // Disable flash firewall
                ffr_configure_firewall(kFFR_BankTypeBank2_CFPA, false);
            }
            // Erase CFPA scratch page area to clear the magic value of cmpaProgInProgress flag
            status_t status =
                FLASH_Erase(config, scratchPageAddress, config->ffrConfig.ffrPageSize, kFLASH_ApiEraseKey);
        }
    }

    return status;
}

// This routine will erase "customer factory page" and program the page with passed data.
status_t FFR_CustFactoryPageWrite(flash_config_t *config, uint8_t *page_data, bool seal_part)
{
    cmpa_cfg_info_t cmpaCfgInfo;
    bool isHashNeeded = false;
    uint32_t cmpaCfgPageSize = config->ffrConfig.ffrPageSize;
    uint32_t cmpaCfgPageAddress = config->ffrConfig.ffrBlockBase + kFfrPageOffset_CMPA_Cfg * cmpaCfgPageSize;

    // If 'seal_part' parameter is TRUE then the routine will compute SHA256 hash of
    //   the page contents and then programs the pages.
    // 1.During development customer code uses this API with 'seal_part' set to FALSE.
    // 2.During manufacturing this parameter should be set to TRUE to seal the part
    //   from further modifications.
    // Cleanup temp page buffer
    memset(&cmpaCfgInfo, 0x0, sizeof(cmpa_cfg_info_t));
    // Copy customer CMPA data to temp page buffer
    memcpy(&cmpaCfgInfo, page_data, cmpaCfgPageSize - sizeof(cmpaCfgInfo.sha256));
    if (seal_part)
    {
        isHashNeeded = true;
    }
    else
    {
        page_data += cmpaCfgPageSize - sizeof(cmpaCfgInfo.sha256);
        for (uint32_t i = 0; i < sizeof(cmpaCfgInfo.sha256); i++)
        {
            if (*(page_data + i))
            {
                isHashNeeded = true;
                break;
            }
        }
    }
    if (isHashNeeded)
    {
        mbedtls_sha256_context shaCtx;
        // Compute HASH of page data
        mbedtls_sha256_init(&shaCtx);
        mbedtls_sha256_starts(&shaCtx, false);
        mbedtls_sha256_update(&shaCtx, (unsigned char *)&cmpaCfgInfo, cmpaCfgPageSize - sizeof(cmpaCfgInfo.sha256));
        mbedtls_sha256_finish(&shaCtx, (uint8_t *)&cmpaCfgInfo.sha256[0]);
        mbedtls_sha256_free(&shaCtx);
    }

    if (ffr_is_firewall_locked(kFFR_BankTypeBank1_CMPA))
    {
        return kStatus_FLASH_FfrBankIsLocked;
    }
    else
    {
        // Disable flash firewall
        ffr_configure_firewall(kFFR_BankTypeBank1_CMPA, false);
    }

    // Erase and Program new CMPA page in flash
    status_t status = FLASH_Erase(config, cmpaCfgPageAddress, cmpaCfgPageSize, kFLASH_ApiEraseKey);
    if (status != kStatus_FLASH_Success)
    {
        return status;
    }
    // Program actual data
    status = FLASH_Program(config, cmpaCfgPageAddress, (uint8_t *)&cmpaCfgInfo, cmpaCfgPageSize);
    if (status != kStatus_FLASH_Success)
    {
        return status;
    }

    return kStatus_FLASH_Success;
}
*/

// Generic read function, used by customer to read data stored in 'Customer Factory Page'.
status_t FFR_GetCustomerData(flash_config_t *config, uint8_t *pData, uint32_t offset, uint32_t len)
{
    uint32_t cmpaCfgPageSize = config->ffrConfig.ffrPageSize;
    uint32_t cmpaCfgPageAddress = config->ffrConfig.ffrBlockBase + kFfrPageOffset_CMPA_Cfg * cmpaCfgPageSize;

    // Check arguments
    if ((offset + len) > cmpaCfgPageSize)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    return ffr_ipcmd_memcpy(config, pData, (uint8_t *)(cmpaCfgPageAddress + offset), len);
}

/*
// This routine writes the 3 pages allocated for Key store data.
status_t FFR_KeystoreWrite(flash_config_t *config, ffr_key_store_t *pKeyStore)
{
    status_t status = kStatus_Fail;
    uint8_t *pageData = (uint8_t *)pKeyStore;
    uint32_t keyPageSize = config->ffrConfig.ffrPageSize;

    // Used during manufacturing. Should write pages when 'customer factory page' is not in sealed state.
    {
        bool isCmpaSealed = false;
        cmpa_cfg_info_t *cmpaCfgInfo = (cmpa_cfg_info_t *)0;
        uint32_t cmpaCfgPageSize = config->ffrConfig.ffrPageSize;
        uint32_t cmpaCfgPageStart = config->ffrConfig.ffrBlockBase + kFfrPageOffset_CMPA_Cfg * cmpaCfgPageSize;
        uint8_t *hashDigestStart =
            (uint8_t *)(cmpaCfgPageStart + (uint32_t)((uint8_t *)cmpaCfgInfo->sha256 - (uint8_t *)cmpaCfgInfo));

        uint8_t blankHash[32];
        uint8_t *storedHash = hashDigestStart;

        memset(blankHash, 0, sizeof(blankHash));
        if (ffr_ipcmd_memcmp(config, blankHash, storedHash, sizeof(blankHash)) != 0)
        {
            isCmpaSealed = true;
        }
        if (isCmpaSealed)
        {
            return kStatus_FLASH_SealedFfrRegion;
        }
    }

    if (ffr_is_firewall_locked(kFFR_BankTypeBank1_CMPA))
    {
        return kStatus_FLASH_FfrBankIsLocked;
    }
    else
    {
        // Disable flash firewall
        ffr_configure_firewall(kFFR_BankTypeBank1_CMPA, false);
    }
    for (uint32_t index = 0; index < kFfrPageNum_CMPA_Key; index++)
    {
        uint32_t keyPageAddress = config->ffrConfig.ffrBlockBase + (kFfrPageOffset_CMPA_Key + index) * keyPageSize;

        // Erase and Program new KEY page in flash
        status = FLASH_Erase(config, keyPageAddress, keyPageSize, kFLASH_ApiEraseKey);
        if (status != kStatus_FLASH_Success)
        {
            return status;
        }

        status = FLASH_Program(config, keyPageAddress, pageData, keyPageSize);
        if (status != kStatus_FLASH_Success)
        {
            return status;
        }

        pageData += keyPageSize;
    }

    return kStatus_FLASH_Success;
}

status_t FFR_KeystoreGetAC(flash_config_t *config, uint8_t *pActivationCode)
{
    // Calling code should pass buffer pointer which can hold activation code 1192 bytes
    uint32_t activationCodeLen = kFfrBlockSize_ActivationCode;
    uint32_t activationCodePageSize = config->ffrConfig.ffrPageSize;
    uint32_t activationCodePageAddress =
        config->ffrConfig.ffrBlockBase + kFfrPageOffset_CMPA_Key * activationCodePageSize;
    uint32_t activationCodeStart = activationCodePageAddress + sizeof(cmpa_key_store_header_t);

    while (activationCodeLen)
    {
        uint32_t readoutBytes;
        if ((activationCodeStart % activationCodePageSize + activationCodeLen) > activationCodePageSize)
        {
            readoutBytes = activationCodePageSize - activationCodeStart % activationCodePageSize;
        }
        else
        {
            readoutBytes = activationCodeLen;
        }

        status_t status = ffr_ipcmd_memcpy(config, pActivationCode, (uint8_t *)activationCodeStart, readoutBytes);
        if (status != kStatus_FLASH_Success)
        {
            return status;
        }

        activationCodeLen -= readoutBytes;
        pActivationCode += readoutBytes;
        activationCodeStart += readoutBytes;
        activationCodePageAddress = activationCodeStart;
    }

    return kStatus_FLASH_Success;
}

status_t FFR_KeystoreGetKC(flash_config_t *config, uint8_t *pKeyCode, ffr_key_type_t keyIndex)
{
    // Calling code should pass buffer pointer which can hold key code 52 bytes.
    uint32_t keyStart = config->ffrConfig.ffrBlockBase + kFfrPageOffset_CMPA_Key * config->ffrConfig.ffrPageSize;
    keyStart += sizeof(cmpa_key_store_header_t) + kFfrBlockSize_ActivationCode;
    keyStart += (uint32_t)keyIndex * (kFfrBlockSize_Key + sizeof(uint32_t)) + sizeof(uint32_t);

    return ffr_ipcmd_memcpy(config, pKeyCode, (uint8_t *)keyStart, kFfrBlockSize_Key);
}

status_t FFR_NxpAreaCheckIntegrity(flash_config_t *config)
{
    status_t status = kStatus_Fail;

    // Check NXP FFR page integrity and lock bank0 (NMPA area)
    nmpa_cfg_info_t *nmpaCfgInfo = (nmpa_cfg_info_t *)0;
    uint32_t nmpaRange1Start = config->ffrConfig.ffrBlockBase + kFfrPageOffset_NMPA * config->ffrConfig.ffrPageSize;
    uint32_t nmpaRange1Size = (kFfrPageOffset_NMPA_Repair - kFfrPageOffset_NMPA) * config->ffrConfig.ffrPageSize;
    uint32_t nmpaRange2Start = config->ffrConfig.ffrBlockBase + kFfrPageOffset_NMPA_Cfg * config->ffrConfig.ffrPageSize;
    uint32_t nmpaRange2Size = (uint8_t *)nmpaCfgInfo->calcHashReserved - (uint8_t *)nmpaCfgInfo;
    uint8_t *hashDigestStart =
        (uint8_t *)(nmpaRange2Start + (uint32_t)((uint8_t *)nmpaCfgInfo->sha256 - (uint8_t *)nmpaCfgInfo));

    uint8_t calcHash[32];
    uint8_t *storedHash = hashDigestStart;
#if defined(BL_TARGET_FPGA)
    uint32_t hashIndex = sizeof(calcHash);
    while (hashIndex && (!(*storedHash++)))
    {
        hashIndex--;
    }
    // A memory block is needed to be check hash if the SHA256 value in the hash digest area has non-zero value.
    if (hashIndex)
#endif
    {
        mbedtls_sha256_context shaCtx;
        // Compute HASH of pages data
        mbedtls_sha256_init(&shaCtx);
        mbedtls_sha256_starts(&shaCtx, false);
        mbedtls_sha256_update(&shaCtx, (unsigned char *)nmpaRange1Start, nmpaRange1Size);
        mbedtls_sha256_update(&shaCtx, (unsigned char *)nmpaRange2Start, nmpaRange2Size);
        mbedtls_sha256_finish(&shaCtx, calcHash);
        mbedtls_sha256_free(&shaCtx);

        storedHash = hashDigestStart;
        if (ffr_ipcmd_memcmp(config, calcHash, storedHash, sizeof(calcHash)) != 0)
        {
            status = kStatus_FLASH_HashCheckError;
        }
    }
#if defined(BL_TARGET_FPGA)
    else
    {
        status = kStatus_FLASH_Success;
    }
#endif

    return status;
}

status_t FFR_GetRompatchData(flash_config_t *config, uint8_t *pData, uint32_t offset, uint32_t len)
{
    uint32_t romcpPageSize = config->ffrConfig.ffrPageSize;
    uint32_t romcpPageAddress = config->ffrConfig.ffrBlockBase + kFfrPageOffset_NMPA_Romcp * romcpPageSize;
    uint32_t romcpRegionSize = romcpPageSize * kFfrPageNum_NMPA_Romcp;

    // Check arguments
    if ((offset + len) > romcpRegionSize)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    while (len)
    {
        uint32_t readoutBytes = romcpPageSize;
        if (offset % romcpPageSize)
        {
            if (((offset % romcpPageSize) + len) < romcpPageSize)
            {
                readoutBytes = len;
            }
            else
            {
                readoutBytes -= offset % romcpPageSize;
            }
        }
        else
        {
            if (len < romcpPageSize)
            {
                readoutBytes = len;
            }
        }

        ffr_ipcmd_memcpy(config, pData, (uint8_t *)(romcpPageAddress + offset), readoutBytes);
        pData += readoutBytes;
        offset += readoutBytes;
        len -= readoutBytes;
    }

    return kStatus_FLASH_Success;
}
*/

// Generic read function, used by customer to read data stored in 'Manufacuring Programmed CFG Page'.
status_t FFR_GetManufactureData(flash_config_t *config, uint8_t *pData, uint32_t offset, uint32_t len)
{
    uint32_t nmpaCfgPageSize = config->ffrConfig.ffrPageSize;
    uint32_t nmpaCfgPageAddress = config->ffrConfig.ffrBlockBase + kFfrPageOffset_NMPA_Cfg * nmpaCfgPageSize;

    // Check arguments
    if ((offset + len) > nmpaCfgPageSize)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    return ffr_ipcmd_memcpy(config, pData, (uint8_t *)(nmpaCfgPageAddress + offset), len);
}

status_t FFR_GetUUID(flash_config_t *config, uint8_t *uuid)
{
    uint32_t nmpaCfgStart = config->ffrConfig.ffrBlockBase + kFfrPageOffset_NMPA_Cfg * config->ffrConfig.ffrPageSize;
    status_t status = FLASH_VerifyErase(config, nmpaCfgStart, config->ffrConfig.ffrPageSize);

    nmpa_cfg_info_t *nmpaCfgInfo;
    nmpaCfgInfo = (nmpa_cfg_info_t *)nmpaCfgStart;

    return ffr_ipcmd_memcpy(config, uuid, (uint8_t *)nmpaCfgInfo->uuid, sizeof(nmpaCfgInfo->uuid));
}

/*
static void ffr_get_cfpa_version(flash_config_t *config, uint32_t ffrPageOffset, bool needNewer)
{
    cfpa_cfg_info_t cfpaCfgInfo;
    uint32_t cfpaPageSize = config->ffrConfig.ffrPageSize;
    uint32_t cfpaPageIndex = config->ffrConfig.ffrBlockBase / config->ffrConfig.ffrPageSize + ffrPageOffset;
    uint32_t cfpaPageAddress = cfpaPageIndex * cfpaPageSize;

    // Get CFPA page data by IP read
    memset(&cfpaCfgInfo, 0x0, sizeof(cfpa_cfg_info_t));
    status_t status = FLASH_Read(config, cfpaPageAddress, (uint8_t *)&cfpaCfgInfo, sizeof(cfpa_cfg_info_t));
    if (status != kStatus_FLASH_Success)
    {
        return;
    }

    // Avoid getting version number from invalid CFPA scratch page
    if (kFfrPageOffset_CFPA_Scratch == cfpaPageAddress)
    {
        // Validate ScratchPage first
        mbedtls_sha256_context shaCtx;
        uint8_t calcDigest[32];
        // Compute HASH of page data
        mbedtls_sha256_init(&shaCtx);
        mbedtls_sha256_starts(&shaCtx, false);
        mbedtls_sha256_update(&shaCtx, (unsigned char *)&cfpaCfgInfo,
                              config->ffrConfig.ffrPageSize - sizeof(cfpaCfgInfo.sha256));
        mbedtls_sha256_finish(&shaCtx, (uint8_t *)&calcDigest[0]);
        mbedtls_sha256_free(&shaCtx);

        // Do constant comparasion
        uint32_t diff = 0;
        for (uint32_t i = 0; i < ARRAY_SIZE(calcDigest); i++)
        {
            diff |= (calcDigest[i] ^ cfpaCfgInfo.sha256[i]);
        }
        if (diff != 0)
        {
            return;
        }
    }

    if (needNewer)
    {
        if (cfpaCfgInfo.version > config->ffrConfig.cfpaPageVersion)
        {
            config->ffrConfig.cfpaPageVersion = cfpaCfgInfo.version;
            config->ffrConfig.cfpaPageOffset = ffrPageOffset;
        }
    }
    else
    {
        if (cfpaCfgInfo.version < config->ffrConfig.cfpaPageVersion)
        {
            config->ffrConfig.cfpaPageVersion = cfpaCfgInfo.version;
            config->ffrConfig.cfpaPageOffset = ffrPageOffset;
        }
    }
}

static bool ffr_is_range_hash_check_pass(flash_config_t *config,
                                         uint8_t *start,
                                         uint32_t lenInBytes,
                                         uint8_t *hashDigestStart)
{
    uint8_t calcHash[32];
    uint8_t *storedHash = hashDigestStart;
    uint32_t hashIndex = sizeof(calcHash);
    while (hashIndex && (!(*storedHash++)))
    {
        hashIndex--;
    }
    // A memory block is needed to be check hash if the SHA256 value in the hash digest area has non-zero value.
    if (hashIndex)
    {
        mbedtls_sha256_context shaCtx;
        // Compute HASH of page data
        mbedtls_sha256_init(&shaCtx);
        mbedtls_sha256_starts(&shaCtx, false);
        mbedtls_sha256_update(&shaCtx, (unsigned char *)start, lenInBytes);
        mbedtls_sha256_finish(&shaCtx, calcHash);
        mbedtls_sha256_free(&shaCtx);

        storedHash = hashDigestStart;
        if (ffr_ipcmd_memcmp(config, calcHash, storedHash, sizeof(calcHash)) != 0)
        {
            return false;
        }
    }

    return true;
}

static bool ffr_is_page_hash_check_pass(flash_config_t *config, uint8_t *page_data)
{
    uint8_t *storedHash = page_data + config->ffrConfig.ffrPageSize - FLASH_FFR_HASH_DIGEST_SIZE;
    return ffr_is_range_hash_check_pass(config, page_data, config->ffrConfig.ffrPageSize - FLASH_FFR_HASH_DIGEST_SIZE,
                                        storedHash);
}

static void ffr_configure_firewall(ffr_bank_type_t bankType, bool setEnable)
{
    if (setEnable)
    {
        // 1. Enable Flash Protection for three IFR banks
        // SYSCON->FLASHBANKENABLE = 0x000; // Banks 0 (NXP), 1(key)  and 2 (cust) are protected
        // 2. Disable write access to FLASHBENKENABLE register
        switch (bankType)
        {
            case kFFR_BankTypeBank2_CFPA:
                SYSCON->FLASHBANKENABLE = (SYSCON->FLASHBANKENABLE & (~SYSCON_FLASHBANKENABLE_BANK2_MASK)) |
                                          SYSCON_FLASHBANKENABLE_BANK2(0x0u);
                SYSCON->CONFIGLOCKOUT = (SYSCON->CONFIGLOCKOUT & (~SYSCON_CONFIGLOCKOUT_FLASHBANK2_LOCK_MASK)) |
                                        SYSCON_CONFIGLOCKOUT_FLASHBANK2_LOCK(1);
                break;
            case kFFR_BankTypeBank1_CMPA:
                SYSCON->FLASHBANKENABLE = (SYSCON->FLASHBANKENABLE & (~SYSCON_FLASHBANKENABLE_BANK1_MASK)) |
                                          SYSCON_FLASHBANKENABLE_BANK1(0x0u);
                SYSCON->CONFIGLOCKOUT = (SYSCON->CONFIGLOCKOUT & (~SYSCON_CONFIGLOCKOUT_FLASHBANK1_LOCK_MASK)) |
                                        SYSCON_CONFIGLOCKOUT_FLASHBANK1_LOCK(1);
                break;
            case kFFR_BankTypeBank0_NMPA:
            default:
                SYSCON->FLASHBANKENABLE = (SYSCON->FLASHBANKENABLE & (~SYSCON_FLASHBANKENABLE_BANK0_MASK)) |
                                          SYSCON_FLASHBANKENABLE_BANK0(0x0u);
                SYSCON->CONFIGLOCKOUT = (SYSCON->CONFIGLOCKOUT & (~SYSCON_CONFIGLOCKOUT_FLASHBANK0_LOCK_MASK)) |
                                        SYSCON_CONFIGLOCKOUT_FLASHBANK0_LOCK(1);
                break;
        }
    }
    else
    {
        // 1. Enable write access to FLASHBENKENABLE register
        // 2. Disable Flash Protection for three IFR banks
        // SYSCON->FLASHBANKENABLE = 0xaaa; // Banks 0 (NXP), 1(key)  and 2 (cust) are unprotected
        switch (bankType)
        {
            case kFFR_BankTypeBank2_CFPA:
                SYSCON->CONFIGLOCKOUT = (SYSCON->CONFIGLOCKOUT & (~SYSCON_CONFIGLOCKOUT_FLASHBANK2_LOCK_MASK)) |
                                        SYSCON_CONFIGLOCKOUT_FLASHBANK2_LOCK(0);
                SYSCON->FLASHBANKENABLE = (SYSCON->FLASHBANKENABLE & (~SYSCON_FLASHBANKENABLE_BANK2_MASK)) |
                                          SYSCON_FLASHBANKENABLE_BANK2(0xau);
                break;
            case kFFR_BankTypeBank1_CMPA:
                SYSCON->CONFIGLOCKOUT = (SYSCON->CONFIGLOCKOUT & (~SYSCON_CONFIGLOCKOUT_FLASHBANK1_LOCK_MASK)) |
                                        SYSCON_CONFIGLOCKOUT_FLASHBANK1_LOCK(0);
                SYSCON->FLASHBANKENABLE = (SYSCON->FLASHBANKENABLE & (~SYSCON_FLASHBANKENABLE_BANK1_MASK)) |
                                          SYSCON_FLASHBANKENABLE_BANK1(0xau);
                break;
            case kFFR_BankTypeBank0_NMPA:
            default:
                SYSCON->CONFIGLOCKOUT = (SYSCON->CONFIGLOCKOUT & (~SYSCON_CONFIGLOCKOUT_FLASHBANK0_LOCK_MASK)) |
                                        SYSCON_CONFIGLOCKOUT_FLASHBANK0_LOCK(0);
                SYSCON->FLASHBANKENABLE = (SYSCON->FLASHBANKENABLE & (~SYSCON_FLASHBANKENABLE_BANK0_MASK)) |
                                          SYSCON_FLASHBANKENABLE_BANK0(0xau);
                break;
        }
    }
}

static bool ffr_is_firewall_locked(ffr_bank_type_t bankType)
{
    bool isLocked = false;
    uint32_t bank;
    uint32_t flashbankLock;

    switch (bankType)
    {
        case kFFR_BankTypeBank2_CFPA:
            bank =
                (SYSCON->FLASHBANKENABLE & (~SYSCON_FLASHBANKENABLE_BANK2_MASK)) >> SYSCON_FLASHBANKENABLE_BANK2_SHIFT;
            flashbankLock = (SYSCON->CONFIGLOCKOUT & (~SYSCON_CONFIGLOCKOUT_FLASHBANK2_LOCK_MASK)) >>
                            SYSCON_CONFIGLOCKOUT_FLASHBANK2_LOCK_SHIFT;
            break;
        case kFFR_BankTypeBank1_CMPA:
            bank =
                (SYSCON->FLASHBANKENABLE & (~SYSCON_FLASHBANKENABLE_BANK1_MASK)) >> SYSCON_FLASHBANKENABLE_BANK1_SHIFT;
            flashbankLock = (SYSCON->CONFIGLOCKOUT & (~SYSCON_CONFIGLOCKOUT_FLASHBANK1_LOCK_MASK)) >>
                            SYSCON_CONFIGLOCKOUT_FLASHBANK1_LOCK_SHIFT;
            break;
        case kFFR_BankTypeBank0_NMPA:
        default:
            bank =
                (SYSCON->FLASHBANKENABLE & (~SYSCON_FLASHBANKENABLE_BANK0_MASK)) >> SYSCON_FLASHBANKENABLE_BANK0_SHIFT;
            flashbankLock = (SYSCON->CONFIGLOCKOUT & (~SYSCON_CONFIGLOCKOUT_FLASHBANK0_LOCK_MASK)) >>
                            SYSCON_CONFIGLOCKOUT_FLASHBANK0_LOCK_SHIFT;
            break;
    }

    if ((bank != 0xa) && flashbankLock)
    {
        isLocked = true;
    }

    return isLocked;
}
*/

static status_t ffr_ipcmd_memcpy(flash_config_t *config, uint8_t *dest, uint8_t *src, uint32_t size)
{
    return FLASH_Read(config, (uint32_t)src, dest, size);
}

static uint32_t ffr_ipcmd_memcmp(flash_config_t *config, uint8_t *dest, uint8_t *src, uint32_t size)
{
    status_t status = kStatus_Fail;
    uint32_t failedAddress;
    uint32_t failedData;

    status = FLASH_VerifyProgram(config, (uint32_t)src, size, (const uint8_t *)dest, &failedAddress, &failedData);

    if (status == kStatus_FLASH_Success)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/*
status_t ffr_check_cfpa_revision(flash_config_t *config,
                                 const cfpa_cfg_info_t *formerCfpa,
                                 const cfpa_cfg_info_t *newCfpa)
{
    // Validate ScratchPage first
    mbedtls_sha256_context shaCtx;
    uint8_t calcDigest[32];
    // Compute HASH of page data
    mbedtls_sha256_init(&shaCtx);
    mbedtls_sha256_starts(&shaCtx, false);
    mbedtls_sha256_update(&shaCtx, (unsigned char *)newCfpa, config->ffrConfig.ffrPageSize - sizeof(newCfpa->sha256));
    mbedtls_sha256_finish(&shaCtx, (uint8_t *)&calcDigest[0]);
    mbedtls_sha256_free(&shaCtx);

    // Do constant comparasion
    uint32_t diff = 0;
    for (uint32_t i = 0; i < ARRAY_SIZE(calcDigest); i++)
    {
        diff |= (calcDigest[i] ^ newCfpa->sha256[i]);
    }
    if (diff != 0)
    {
        return kStatus_FLASH_CfpaScratchPageInvalid;
    }

    // Do rollback check
    if (formerCfpa->secureFwVersion > newCfpa->secureFwVersion)
    {
        // Ensure the secureFwVersion in new CFPA must be >= existing secureFwVersion
        return kStatus_FLASH_CfpaVersionRollbackDisallowed;
    }
    if (formerCfpa->nsFwVersion > newCfpa->nsFwVersion)
    {
        // Ensure the nsFwVersion in new CFPA must be >= existing nsFwVersion
        return kStatus_FLASH_CfpaVersionRollbackDisallowed;
    }
    if (formerCfpa->vendorUsage > newCfpa->vendorUsage)
    {
        // Ensure the vendorUsage in new CFPA must be >= existing vendorUsage
        return kStatus_FLASH_CfpaVersionRollbackDisallowed;
    }

    if ((formerCfpa->imageKeyRevoke & newCfpa->imageKeyRevoke) != formerCfpa->imageKeyRevoke)
    {
        // Ensure that the imageKeyRevoke must be >= existing imageKeyRevoke
        // Note the imagekeyRevoke are treated like OTP, the version increasing flow, (0, 1, 3, 7, etc)
        return kStatus_FLASH_CfpaVersionRollbackDisallowed;
    }

    if ((formerCfpa->rotkhRevoke & newCfpa->rotkhRevoke) != formerCfpa->rotkhRevoke)
    {
        // Ensure that the rotkhRevoke must be >= existing rotkhRevoke
        // Note the rotkhRevoke are treated like OTP, the version increasing flow, (0, 1, 3, 7, etc)
        return kStatus_FLASH_CfpaVersionRollbackDisallowed;
    }

    if (formerCfpa->enableFaMode != 0)
    {
        // Disallow to update the CFPA is the FA mode is enabled
        return kStatus_FLASH_CfpaVersionRollbackDisallowed;
    }

    return kStatus_Success;
}
*/
