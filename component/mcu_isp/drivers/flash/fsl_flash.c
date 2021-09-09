/*
 * Copyright 2017 - 2018 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_flash.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct
{
    uint32_t flashSizeCode;
    uint32_t flashSize;
} flash_size_code_t;

enum
{
    kSysToFlashFreq_defaultInMHz = 12u
};

enum
{
    kFmcCacheOption_NoBuffer = 0x0u,
    kFmcCacheOption_OneBuffer = 0x1u,
    kFmcCacheOption_AllBuffer = 0x2u,
};

enum
{
    kFmcAccelOption_Slow = 0x0u,
    kFmcAccelOption_Fast = 0x1u,
};

/*!
 * @name Flash controller timing constances
 * @{
 */
#define FLASH_TIMING_MIN_PRECHARGE_DURATION_Tp_NS 15
#define FLASH_TIMING_MIN_EVALUATION_DURATION_Tdpd_NS 25
#define FLASH_TIMING_MIN_SYNTHESIS_COST_NS 34

/*!
 * @name Flash controller command numbers
 * @{
 */
/* Below CMDs apply to both C040HDATFC and C040HDFC flash */
#define FLASH_CMD_INIT 0
#define FLASH_CMD_POWERDOWN 1
#define FLASH_CMD_SET_READ_MODE 2
#define FLASH_CMD_READ_SINGLE_WORD 3
#define FLASH_CMD_ERASE_RANGE 4
#define FLASH_CMD_BLANK_CHECK 5
#define FLASH_CMD_MARGIN_CHECK 6
#define FLASH_CMD_CHECKSUM 7
#define FLASH_CMD_WRITE 8
#define FLASH_CMD_WRITE_PROG 10
#define FLASH_CMD_PROGRAM 12
#define FLASH_CMD_REPORT_ECC 13
/* Below CMDs apply to C040HDATFC flash only */
#define FLASH_CMD_SET_WRITE_MODE 14
#define FLASH_CMD_PATTERN_CHECK 16                /* Test mode only */
#define FLASH_CMD_MASS_ERASE_PROG 17              /* Test mode only */
#define FLASH_CMD_MASS_WRITE 18                   /* Test mode only */
#define FLASH_CMD_READ_TRIM_DATA 19               /* Test mode only */
#define FLASH_CMD_PAGEREGISTER_UPLOAD 20          /* Test mode only */
#define FLASH_CMD_SET_INDIVIDUAL_TRIM_REGISTER 21 /* Test mode only */
#define FLASH_CMD_TRIM_READ_PHASE 22              /* Test mode only */
                                                  /*@}*/

/*!
 * @name Flash property defines
 * @{
 */
/* Mask the number of bits required to select the 32-bit data word (DATAW) from the flash line */
#define FLASH_DATAW_IDX_MAX 3    /* Max DATAW index, 3 for a 128-bit flash line, 7 for 256-bit. */
#define FLASH_DATAW_IDX_MASK 0x3 /* For a 128-bit flash line */
#define FLASH_DATAW_IDX_SHIFT 2  /* For a 128-bit flash line */

/*!
 * @name Flash command magic key
 * @{
 */
enum _flash_command_magic_key
{
    kFLASH_CommandMagicKeyStartAddress = 0xBB, /* The start address of identification number for command */
    kFLASH_CommandMagicKeyEndAddress = 0xAA    /* The end address of identification number for command */
};

/*!
 * @name Flash register info defines
 * @{
 */
#define FLASH_READPARAM_REG (FLASH->DATAW[0])
#define FLASH_READPARAM_WAIT_STATE_MASK (0xFU)
#define FLASH_READPARAM_WAIT_STATE_SHIFT (0U)
#define FLASH_READPARAM_WAIT_STATE(x) \
    (((uint32_t)(((uint32_t)(x)) << FLASH_READPARAM_WAIT_STATE_SHIFT)) & FLASH_READPARAM_WAIT_STATE_MASK)
#define FLASH_READPARAM_INTERFACE_TRIM_MASK (0xFFF0U)
#define FLASH_READPARAM_INTERFACE_TRIM_SHIFT (4U)
#define FLASH_READPARAM_INTERFACE_TRIM(x) \
    (((uint32_t)(((uint32_t)(x)) << FLASH_READPARAM_INTERFACE_TRIM_SHIFT)) & FLASH_READPARAM_INTERFACE_TRIM_MASK)
#define FLASH_READPARAM_CONTROLLER_TRIM_MASK (0xFFF0000U)
#define FLASH_READPARAM_CONTROLLER_TRIM_SHIFT (16U)
#define FLASH_READPARAM_CONTROLLER_TRIM(x) \
    (((uint32_t)(((uint32_t)(x)) << FLASH_READPARAM_CONTROLLER_TRIM_SHIFT)) & FLASH_READPARAM_CONTROLLER_TRIM_MASK)

#define FLASH_WRITEPARAM_REG (FLASH->DATAW[0])
#define FLASH_WRITEPARAM_ERASE_RAMP_CTRL_MASK (0x3U)
#define FLASH_WRITEPARAM_ERASE_RAMP_CTRL_SHIFT (0U)
#define FLASH_WRITEPARAM_ERASE_RAMP_CTRL(x) \
    (((uint32_t)(((uint32_t)(x)) << FLASH_WRITEPARAM_ERASE_RAMP_CTRL_SHIFT)) & FLASH_WRITEPARAM_ERASE_RAMP_CTRL_MASK)
#define FLASH_WRITEPARAM_PROGRAM_RAMP_CTRL_MASK (0xCU)
#define FLASH_WRITEPARAM_PROGRAM_RAMP_CTRL_SHIFT (2U)
#define FLASH_WRITEPARAM_PROGRAM_RAMP_CTRL(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << FLASH_WRITEPARAM_PROGRAM_RAMP_CTRL_SHIFT)) & \
     FLASH_WRITEPARAM_PROGRAM_RAMP_CTRL_MASK)

#define FLASH_READMODE_REG (FLASH->DATAW[0])
#define FLASH_READMODE_ECC_MASK (0x4U)
#define FLASH_READMODE_ECC_SHIFT (2U)
#define FLASH_READMODE_ECC(x) (((uint32_t)(((uint32_t)(x)) << FLASH_READMODE_ECC_SHIFT)) & FLASH_READMODE_ECC_MASK)
#define FLASH_READMODE_MARGIN_MASK (0xC00U)
#define FLASH_READMODE_MARGIN_SHIFT (10U)
#define FLASH_READMODE_MARGIN(x) \
    (((uint32_t)(((uint32_t)(x)) << FLASH_READMODE_MARGIN_SHIFT)) & FLASH_READMODE_MARGIN_MASK)
#define FLASH_READMODE_DMACC_MASK (0x8000U)
#define FLASH_READMODE_DMACC_SHIFT (15U)
#define FLASH_READMODE_DMACC(x) \
    (((uint32_t)(((uint32_t)(x)) << FLASH_READMODE_DMACC_SHIFT)) & FLASH_READMODE_DMACC_MASK)
/*@}*/

/*!
 * @name Flash register access type defines
 * @{
 */
#define FLASH_REG8_ACCESS_TYPE volatile uint8_t *
#define FLASH_REG32_ACCESS_TYPE volatile uint32_t *
/*@}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*! @brief Gets flash size from SYSCON.*/
void flash_get_size(uint32_t *size);
/*! @brief Resets flash controller.*/
static void flash_reset(void);
/*! @brief Configures timing parameters of flash controller.*/
static void flash_timing_configuration(flash_config_t *config);
/*! @brief Internal function Flash command sequence. Called by driver APIs only*/
static status_t flash_command_sequence(flash_config_t *config);
/*! @brief Perform the cache clear to the flash*/
void flash_cache_clear(flash_config_t *config);
/*! @brief Validates the range and alignment of the given address range.*/
static status_t flash_check_range(flash_config_t *config,
                                  uint32_t startAddress,
                                  uint32_t lengthInBytes,
                                  uint32_t alignmentBaseline);
/*! @brief Validates the given user key for flash erase APIs.*/
static status_t flash_check_user_key(uint32_t key);
/*! @brief Reads word from byte address.*/
static uint32_t flash_read_word_from_byte_address(uint8_t *src);
/*! @brief Writes word to byte address.*/
static void flash_write_word_to_byte_address(uint8_t *dst, uint32_t word);

status_t FLASH_Initialization(flash_config_t *config);
status_t FLASH_SetReadModes(flash_config_t *config);
status_t FLASH_SetWriteMode(flash_config_t *config);
status_t FLASH_ErasePageRange(flash_config_t *config, uint32_t firstPageIndex, uint32_t lastPageIndex);
status_t FLASH_BlankCheckPageRange(flash_config_t *config, uint32_t firstPageIndex, uint32_t lastPageIndex);
status_t FLASH_MarginCheckPageRange(flash_config_t *config, uint32_t firstPageIndex, uint32_t lastPageIndex);
status_t FLASH_ReadSingleWord(flash_config_t *config, uint32_t start, uint32_t *readbackData);
status_t FLASH_ReportEccLog(flash_config_t *config, flash_ecc_log_t *eccLog);

/*******************************************************************************
 * Variables
 ******************************************************************************/

#define FSL_FEATURE_SOC_FLASH_SIZE_CODE_ARRAY                                              \
    {                                                                                      \
        {                                                                                  \
            0, 128 * 1024u                                                                 \
        }                                                                                  \
        , { 1, 256 * 1024u }, { 2, 512 * 1024u }, { 5, 0u }, { 0xff, 631 * 1024u + 512u }, \
    \
}

static const flash_size_code_t k_flashSizeCodeArray[] = FSL_FEATURE_SOC_FLASH_SIZE_CODE_ARRAY;

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t FLASH_Init(flash_config_t *config)
{
    status_t status = kStatus_Fail;

    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    /* Fill out a few of the structure members */
    config->PFlashBlockBase = 0;
    config->PFlashBlockCount = 1;
    flash_get_size(&config->PFlashTotalSize);
    config->PFlashPageSize = FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES;
    config->PFlashSectorSize = FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES;

    /* Enable AHB bus clocks for FLASH and FMC */
    if (SYSCON->AHBCLKCTRLX[0] & SYSCON_AHBCLKCTRL0_FLASH_MASK)
    {
        SYSCON->AHBCLKCTRLSET[0] |= SYSCON_AHBCLKCTRL0_FLASH_MASK;
    }
    if (SYSCON->AHBCLKCTRLX[0] & SYSCON_AHBCLKCTRL0_FMC_MASK)
    {
        SYSCON->AHBCLKCTRLSET[0] |= SYSCON_AHBCLKCTRL0_FMC_MASK;
    }

/* Entering Bus fault exception when :
   1. Erasing Protected Page
   2. Blank checking Protected Page
   3. Programming Protected Page
   4. Reading erased Page
*/

/* Niobe4 Flash memory data layout:
     There are 1280 pages, each page has 512-bytes data
     Each 16-bytes data has one ECC byte, ECC byte is not accessible
     Each page has one DMACC word (16 byte), it could be accessed by ReadSingleWord CMD
     The flash topmost 8 pages are not to be modified by the final user; depending on the
      security requirements of the device, reading may also be not allowed. It is the integrator
      responsibility to ensure that these pages are not made accessible.
*/
#if 0
    /* Entering reset mode to reset flash controller*/
    flash_reset();

    /* Immediately after leaving reset mode, an initialization phase takes place,
       where some memory locations are read, and corresponding volatile locations
       are initialized depending on the value just read. */
    status = FLASH_Initialization(config);
    if ((kStatus_FLASH_Success != status) && (status != kStatus_FLASH_EccError))
    {
        return status;
    }
#endif

/* Configuration is normally performed by system software shortly after initialization,
   although default configuration values are normally chosen to allow safe operation
   with no further software intervention. */
#if !defined(BL_TARGET_FPGA)
//    status = FLASH_ConfigTiming(config);
//    if (kStatus_FLASH_Success != status)
//    {
//        return status;
//    }
#endif

    return kStatus_FLASH_Success;
}

status_t FLASH_ConfigTiming(flash_config_t *config)
{
    status_t status = kStatus_Fail;

    if (!config->modeConfig.sysFreqInMHz)
    {
        config->modeConfig.sysFreqInMHz = kSysToFlashFreq_defaultInMHz;
    }

    /* Fill out default parameters for ReadSingleWord command */
    config->modeConfig.readSingleWord.readWithEccOff = kFLASH_ReadWithEccOn;
    config->modeConfig.readSingleWord.readMarginLevel = kFLASH_ReadMarginNormal;
    config->modeConfig.readSingleWord.readDmaccWord = kFLASH_ReadDmaccDisabled;

    /* Flash Controller & FMC internal number of Wait States*/
    uint32_t numWaitStates;
    /* Flash Controller & FMC internal number of Wait States Parameter (to be written in registers) */
    uint32_t numWaitStatesParam;
    numWaitStates = ((90 * config->modeConfig.sysFreqInMHz) / 1000) + 1;

    //if (SYSCON->FLASHRCLKSEL & SYSCON_FLASHRCLKSEL_SEL_MASK)
    if ((*(volatile uint32_t *)(SYSCON_BASE + 0x2EC)) & 0x1U)
    {
        /* Flash is operating with RCLK */
        numWaitStatesParam = numWaitStates;
    }
    else
    {
        /* Flash is operating with Delay Line */
        numWaitStatesParam = numWaitStates - 1;
    }
    config->modeConfig.setReadMode.readWaitStates = numWaitStatesParam;

    /* Adjust Flash Controller internal read waiting time (numWaitStatesParam) */
    status = FLASH_SetReadModes(config);

    /* Adjust FMC waiting time cycles (numWaitStatesParam) */
    // SYSCON->FMCCR = (SYSCON->FMCCR & 0xFFFF0000) | 0x001A | ((numWaitStatesParam & 0xF) << 12);
    //SYSCON->FMCCR = (SYSCON->FMCCR & (~SYSCON_FMCCR_VALID_MASK)) | SYSCON_FMCCR_FETCHCTL(kFmcCacheOption_AllBuffer) |
    //                SYSCON_FMCCR_DATACTL(kFmcCacheOption_AllBuffer) | SYSCON_FMCCR_ACCEL(kFmcAccelOption_Fast) |
    //                SYSCON_FMCCR_MAMTIM(config->modeConfig.setReadMode.readWaitStates);

    SYSCON->FMCCR = (SYSCON->FMCCR & (~0x0000FFFFU)) | 
                    (((uint32_t)(((uint32_t)(kFmcCacheOption_AllBuffer)) << 0U)) & 0x00000003U) |
                    (((uint32_t)(((uint32_t)(kFmcCacheOption_AllBuffer)) << 2U)) & 0x0000000CU) | 
                    (((uint32_t)(((uint32_t)(kFmcAccelOption_Fast)) << 4U)) & 0x00000010U) |
                    (((uint32_t)(((uint32_t)(config->modeConfig.setReadMode.readWaitStates)) << 12U)) & 0x0000F000U);

    return status;
}

status_t FLASH_Erase(flash_config_t *config, uint32_t start, uint32_t lengthInBytes, uint32_t key)
{
    status_t status = kStatus_Fail;
    uint32_t endAddress = start + lengthInBytes - 1;

    /* Check the supplied address range. */
    status = flash_check_range(config, start, lengthInBytes, kFLASH_AlignementUnitVerifyErase);
    if (kStatus_FLASH_Success != status)
    {
        return status;
    }
    /* Validate the user key */
    status = flash_check_user_key(key);
    if (kStatus_FLASH_Success != status)
    {
        return status;
    }

    start = ALIGN_DOWN(start, config->PFlashPageSize);
    endAddress = ALIGN_DOWN(endAddress, config->PFlashPageSize);

    status = FLASH_ErasePageRange(config, start / config->PFlashPageSize, endAddress / config->PFlashPageSize);

    return status;
}

status_t FLASH_VerifyErase(flash_config_t *config, uint32_t start, uint32_t lengthInBytes)
{
    status_t status = kStatus_Fail;
    uint32_t endAddress = start + lengthInBytes - 1;

    /* Check the supplied address range. */
    status = flash_check_range(config, start, lengthInBytes, kFLASH_AlignementUnitVerifyErase);
    if (kStatus_FLASH_Success != status)
    {
        return status;
    }

    start = ALIGN_DOWN(start, config->PFlashPageSize);
    endAddress = ALIGN_DOWN(endAddress, config->PFlashPageSize);

    /* the c040hdatfc will return an AHB error on reading erased locations, so
       the blank check needs to be done using the BLANK_CHECK controller command */
    status = FLASH_BlankCheckPageRange(config, start / config->PFlashPageSize, endAddress / config->PFlashPageSize);

    return status;
}

status_t FLASH_Program(flash_config_t *config, uint32_t start, uint8_t *src, uint32_t lengthInBytes)
{
    status_t status = kStatus_Fail;
    bool isFirstPage = true;
    // Got from Raff: a flash page cannot be programmed twice without an erase in between,
    //  So the start and length must be page aligned.
    status = flash_check_range(config, start, lengthInBytes, kFLASH_AlignementUnitProgram);
    if (kStatus_FLASH_Success != status)
    {
        return status;
    }

    while (lengthInBytes > 0)
    {
        uint32_t payloadSize = (lengthInBytes > config->PFlashPageSize) ? config->PFlashPageSize : lengthInBytes;
        uint32_t datawIndex;
        uint32_t datawAddr;

        FLASH->INT_CLR_STATUS = FLASH_INT_CLR_STATUS_FAIL_MASK | FLASH_INT_CLR_STATUS_ERR_MASK |
                                FLASH_INT_CLR_STATUS_DONE_MASK | FLASH_INT_CLR_STATUS_ECC_ERR_MASK;

        if (isFirstPage)
        {
            uint32_t pageColumnAddress = start & (config->PFlashPageSize - 1);
            /* Check if start address is on the page boundary */
            if (pageColumnAddress)
            {
                payloadSize = config->PFlashPageSize - pageColumnAddress;
                payloadSize = (payloadSize > lengthInBytes) ? lengthInBytes : payloadSize;
            }
            isFirstPage = false;
        }

        for (datawAddr = start; datawAddr < start + payloadSize; datawAddr += 4)
        {
            /* Get the 32-bit data word index from the address */
            datawIndex = (datawAddr >> FLASH_DATAW_IDX_SHIFT) & FLASH_DATAW_IDX_MASK;
            if (datawIndex == 0)
            {
                /* This is the first DATAW, STARTA expects the 128-bit flash word
                    address, so shift right by 4 */
                FLASH->STARTA = datawAddr >> 4;
            }
            FLASH->DATAW[datawIndex] = flash_read_word_from_byte_address(src);
            src += 4;
            if (datawIndex == FLASH_DATAW_IDX_MAX)
            {
                /* Write the flash line to the page buffer */
                FLASH->CMD = FLASH_CMD_WRITE;
                status = flash_command_sequence(config);
                if (kStatus_FLASH_Success != status)
                {
                    return status;
                }
                FLASH->INT_CLR_STATUS = FLASH_INT_CLR_STATUS_FAIL_MASK | FLASH_INT_CLR_STATUS_ERR_MASK |
                                        FLASH_INT_CLR_STATUS_DONE_MASK | FLASH_INT_CLR_STATUS_ECC_ERR_MASK;
            }
        }

        /* Write the last incomplete flash word */
        datawIndex = (datawAddr >> FLASH_DATAW_IDX_SHIFT) & FLASH_DATAW_IDX_MASK;
        if (datawIndex)
        {
            FLASH->CMD = FLASH_CMD_WRITE;
            status = flash_command_sequence(config);
            if (kStatus_FLASH_Success != status)
            {
                return status;
            }
        }

        /* Start program operation */
        FLASH->CMD = FLASH_CMD_PROGRAM;
        status = flash_command_sequence(config);
        if (kStatus_FLASH_Success != status)
        {
            return status;
        }

        /* Prepare for the next page */
        start += payloadSize;
        lengthInBytes -= payloadSize;
    }

    flash_cache_clear(config);

    return status;
}

status_t FLASH_VerifyProgram(flash_config_t *config,
                             uint32_t start,
                             uint32_t lengthInBytes,
                             const uint8_t *expectedData,
                             uint32_t *failedAddress,
                             uint32_t *failedData)
{
    status_t status = kStatus_Fail;

    if (expectedData == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    status = flash_check_range(config, start, lengthInBytes, 1);
    if (kStatus_FLASH_Success != status)
    {
        return status;
    }

    while (lengthInBytes)
    {
        uint32_t readbackData[FLASH_DATAW_IDX_MAX + 1];
        uint32_t memReadLen;
        if ((!(start % kFLASH_AlignementUnitSingleWordRead)))
        {
            if (lengthInBytes >= kFLASH_AlignementUnitSingleWordRead)
            {
                memReadLen = kFLASH_AlignementUnitSingleWordRead;
            }
            else
            {
                memReadLen = lengthInBytes;
            }
        }
        else
        {
            uint32_t alignedStart = ALIGN_UP(start, kFLASH_AlignementUnitSingleWordRead);
            if ((start + lengthInBytes) >= alignedStart)
            {
                memReadLen = alignedStart - start;
            }
            else
            {
                memReadLen = lengthInBytes;
            }
        }

        status = FLASH_Read(config, start, (uint8_t *)readbackData, memReadLen);
        if (kStatus_FLASH_Success != status)
        {
            if (failedAddress)
            {
                *failedAddress = start;
            }
            if (failedData)
            {
                *failedData = readbackData[0];
            }
            break;
        }
        else
        {
            for (uint32_t i = 0; i < memReadLen; i++)
            {
                if ((*expectedData) != (*((uint8_t *)readbackData + i)))
                {
                    return kStatus_FLASH_CompareError;
                }
                expectedData++;
                start++;
                lengthInBytes--;
            }
        }
    }

    return status;
}

status_t FLASH_GetProperty(flash_config_t *config, flash_property_tag_t whichProperty, uint32_t *value)
{
    if ((config == NULL) || (value == NULL))
    {
        return kStatus_FLASH_InvalidArgument;
    }

    switch (whichProperty)
    {
        case kFLASH_PropertyPflashSectorSize:
            *value = config->PFlashSectorSize;
            break;

        case kFLASH_PropertyPflashTotalSize:
            *value = config->PFlashTotalSize;
            break;

        case kFLASH_PropertyPflashBlockSize:
            *value = config->PFlashTotalSize / (uint32_t)config->PFlashBlockCount;
            break;

        case kFLASH_PropertyPflashBlockCount:
            *value = (uint32_t)config->PFlashBlockCount;
            break;

        case kFLASH_PropertyPflashBlockBaseAddr:
            *value = config->PFlashBlockBase;
            break;

        case kFLASH_PropertyPflashPageSize:
            *value = config->PFlashPageSize;
            break;

        case kFLASH_PropertyPflashSystemFreq:
            *value = config->modeConfig.sysFreqInMHz;
            break;

        case kFLASH_PropertyFfrSectorSize:
            *value = config->PFlashSectorSize;
            break;

        case kFLASH_PropertyFfrTotalSize:
            *value = config->ffrConfig.ffrTotalSize;
            break;

        case kFLASH_PropertyFfrBlockBaseAddr:
            *value = config->ffrConfig.ffrBlockBase;
            break;

        case kFLASH_PropertyFfrPageSize:
            *value = config->ffrConfig.ffrPageSize;
            break;

        default: /* catch inputs that are not recognized */
            return kStatus_FLASH_UnknownProperty;
    }

    return kStatus_FLASH_Success;
}

status_t FLASH_SetProperty(flash_config_t *config, flash_property_tag_t whichProperty, uint32_t value)
{
    status_t status = kStatus_Fail;

    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    switch (whichProperty)
    {
        case kFLASH_PropertyPflashSectorSize:
        case kFLASH_PropertyPflashTotalSize:
        case kFLASH_PropertyPflashBlockSize:
        case kFLASH_PropertyPflashBlockCount:
        case kFLASH_PropertyPflashBlockBaseAddr:
        case kFLASH_PropertyPflashPageSize:
        case kFLASH_PropertyFfrSectorSize:
        case kFLASH_PropertyFfrTotalSize:
        case kFLASH_PropertyFfrBlockBaseAddr:
        case kFLASH_PropertyFfrPageSize:
            status = kStatus_FLASH_ReadOnlyProperty;
            break;
        case kFLASH_PropertyPflashSystemFreq:
            config->modeConfig.sysFreqInMHz = value;
            break;
        default: /* catch inputs that are not recognized */
            status = kStatus_FLASH_UnknownProperty;
            break;
    }

    return status;
}

status_t FLASH_Read(flash_config_t *config, uint32_t start, uint8_t *dest, uint32_t lengthInBytes)
{
    status_t status = kStatus_Fail;

    status = flash_check_range(config, start, lengthInBytes, 1);
    if (kStatus_FLASH_Success != status)
    {
        return status;
    }

    uint32_t readbackData[FLASH_DATAW_IDX_MAX + 1];
    while (lengthInBytes)
    {
        uint32_t alignedStart = ALIGN_DOWN(start, kFLASH_AlignementUnitSingleWordRead);
        status = FLASH_ReadSingleWord(config, alignedStart, readbackData);
        if (status != kStatus_FLASH_Success)
        {
            break;
        }
        for (uint32_t i = 0; i < sizeof(readbackData); i++)
        {
            if ((alignedStart == start) && lengthInBytes)
            {
                *dest = *((uint8_t *)readbackData + i);
                dest++;
                start++;
                lengthInBytes--;
            }
            alignedStart++;
        }
    }

    return status;
}

status_t FLASH_Initialization(flash_config_t *config)
{
    status_t status = kStatus_Fail;

    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    /* Initialization notes:
       1. The controller reads then 18 locations in the last two pages of the flash.
          For each location read, it initializes the corresponding volatile storage in the
          controller: flash trim values, flash repair info, gpo trim bus.
       2. if an uncorrectable ECC error is detected, the initialization is immediately
          terminated with the FAIL flag set.
       3. If the FAIL flag is set during the initialization automatically performed at exit from reset.
          if initialization reports a FAIL, the flash was not correctly configured,
          and must not be used (read data may be incorrect, and writing may corrupt the content).
    */
    FLASH->INT_CLR_STATUS =
        FLASH_INT_CLR_STATUS_FAIL_MASK | FLASH_INT_CLR_STATUS_ERR_MASK | FLASH_INT_CLR_STATUS_ECC_ERR_MASK;
    FLASH->CMD = FLASH_CMD_INIT;
    status = flash_command_sequence(config);

    return status;
}

status_t FLASH_SetReadModes(flash_config_t *config)
{
    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    /* Set read parameters*/
    FLASH_READPARAM_REG = (FLASH_READPARAM_REG & (~FLASH_READPARAM_WAIT_STATE_MASK)) |
                          FLASH_READPARAM_WAIT_STATE(config->modeConfig.setReadMode.readWaitStates);

    /* This starts the Set read mode command, no need to wait until command is
        completed: further accesses are stalled until the command is completed. */
    FLASH->CMD = FLASH_CMD_SET_READ_MODE;

    return kStatus_FLASH_Success;
}

status_t FLASH_SetWriteMode(flash_config_t *config)
{
    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    /* SetWriteMode notes:
       The flash needs two timing references during program and erase: clk and clkrmp.
       Program and erase commands use the specified default frequencies, unless overridden
          by the Set write mode command.
       Once overridden, the newly selected ramp frequencies are in effect until a controller reset.
    */

    /* Set write parameter*/
    FLASH_WRITEPARAM_REG = FLASH_WRITEPARAM_ERASE_RAMP_CTRL(config->modeConfig.setWriteMode.eraseRampControl) |
                           FLASH_WRITEPARAM_PROGRAM_RAMP_CTRL(config->modeConfig.setWriteMode.programRampControl);

    /* This starts the Set write mode command, no need to wait until command is
        completed: further accesses are stalled until the command is completed. */
    FLASH->CMD = FLASH_CMD_SET_WRITE_MODE;

    return kStatus_FLASH_Success;
}

status_t FLASH_ErasePageRange(flash_config_t *config, uint32_t firstPageIndex, uint32_t lastPageIndex)
{
    status_t status = kStatus_Fail;
    uint32_t firstPageStart = firstPageIndex * config->PFlashPageSize;
    uint32_t lastPageStart = lastPageIndex * config->PFlashPageSize;

    FLASH->INT_CLR_STATUS = FLASH_INT_CLR_STATUS_FAIL_MASK | FLASH_INT_CLR_STATUS_ERR_MASK |
                            FLASH_INT_CLR_STATUS_DONE_MASK | FLASH_INT_CLR_STATUS_ECC_ERR_MASK;
    FLASH->STARTA = firstPageStart >> 4;
    FLASH->STOPA = lastPageStart >> 4;
    FLASH->CMD = FLASH_CMD_ERASE_RANGE;

    status = flash_command_sequence(config);

    flash_cache_clear(config);

    return status;
}

status_t FLASH_BlankCheckPageRange(flash_config_t *config, uint32_t firstPageIndex, uint32_t lastPageIndex)
{
    status_t status = kStatus_Fail;
    uint32_t firstPageStart = firstPageIndex * config->PFlashPageSize;
    uint32_t lastPageStart = lastPageIndex * config->PFlashPageSize;

    FLASH->INT_CLR_STATUS = FLASH_INT_CLR_STATUS_FAIL_MASK | FLASH_INT_CLR_STATUS_ERR_MASK |
                            FLASH_INT_CLR_STATUS_DONE_MASK | FLASH_INT_CLR_STATUS_ECC_ERR_MASK;
    FLASH->STARTA = firstPageStart >> 4;
    FLASH->STOPA = lastPageStart >> 4;
    FLASH->CMD = FLASH_CMD_BLANK_CHECK;

    status = flash_command_sequence(config);

    return status;
}

status_t FLASH_MarginCheckPageRange(flash_config_t *config, uint32_t firstPageIndex, uint32_t lastPageIndex)
{
    status_t status = kStatus_Fail;
    uint32_t firstPageStart = firstPageIndex * config->PFlashPageSize;
    uint32_t lastPageStart = lastPageIndex * config->PFlashPageSize;

    FLASH->INT_CLR_STATUS = FLASH_INT_CLR_STATUS_FAIL_MASK | FLASH_INT_CLR_STATUS_ERR_MASK |
                            FLASH_INT_CLR_STATUS_DONE_MASK | FLASH_INT_CLR_STATUS_ECC_ERR_MASK;
    FLASH->STARTA = firstPageStart >> 4;
    FLASH->STOPA = lastPageStart >> 4;
    FLASH->CMD = FLASH_CMD_MARGIN_CHECK;

    status = flash_command_sequence(config);

    return status;
}

status_t FLASH_ReadSingleWord(flash_config_t *config, uint32_t start, uint32_t *readbackData)
{
    status_t status = kStatus_Fail;
    uint32_t byteSizes = sizeof(uint32_t) * (FLASH_DATAW_IDX_MAX + 1);

    if (readbackData == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    status = flash_check_range(config, start, byteSizes, kFLASH_AlignementUnitSingleWordRead);
    if (kStatus_FLASH_Success != status)
    {
        return status;
    }

    FLASH->INT_CLR_STATUS = FLASH_INT_CLR_STATUS_FAIL_MASK | FLASH_INT_CLR_STATUS_ERR_MASK |
                            FLASH_INT_CLR_STATUS_DONE_MASK | FLASH_INT_CLR_STATUS_ECC_ERR_MASK;

    /* Set start address */
    FLASH->STARTA = start >> 4;

    /* ReadSingleWord notes:
        Flash contains one DMACC word per page. Such words are not readable through
          the read interface. DMACC words are managed internally by the controller in
          order to store a flag (all1), which can be used to verify whether a programming
          operation was prematurely terminated.
        DMACC words are all_0 for an erased page, all_1 for a programmed page
    */

    /* Set read modes */
    FLASH_READMODE_REG = FLASH_READMODE_ECC(config->modeConfig.readSingleWord.readWithEccOff) |
                         FLASH_READMODE_MARGIN(config->modeConfig.readSingleWord.readMarginLevel) |
                         FLASH_READMODE_DMACC(config->modeConfig.readSingleWord.readDmaccWord);

    /* Calling flash command sequence function to execute the command */
    FLASH->CMD = FLASH_CMD_READ_SINGLE_WORD;
    status = flash_command_sequence(config);

    if (kStatus_FLASH_Success == status)
    {
        for (uint32_t datawIndex = 0; datawIndex <= FLASH_DATAW_IDX_MAX; datawIndex++)
        {
            *readbackData++ = FLASH->DATAW[datawIndex];
        }
    }

    return status;
}

status_t FLASH_ReportEccLog(flash_config_t *config, flash_ecc_log_t *eccLog)
{
    status_t status = kStatus_Fail;

    if (eccLog == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    FLASH->INT_CLR_STATUS = FLASH_INT_CLR_STATUS_FAIL_MASK | FLASH_INT_CLR_STATUS_ERR_MASK |
                            FLASH_INT_CLR_STATUS_DONE_MASK | FLASH_INT_CLR_STATUS_ECC_ERR_MASK;

    /* ReportEccLog notes:
       All ECC events are logged, both for reads performed by user code and for
        internally-generated reads (e.g. checksum and "read word" commands, initialization).
       This command copies logging information to the DATAW0-2 registers, and then clears the log,
        zeroing the counters. [S-ECCLG]
       20-bit counters are used. When they reach their maximum value, further incrementing
           is prevented (i.e. they saturate rather than wrapping around). [S-ECSAT]
       As the DMACC word is not meant to contain ECC-encoded data, ECC errors are not logged for it. [S-EDMAC]
    */

    /* Calling flash command sequence function to execute the command */
    FLASH->CMD = FLASH_CMD_REPORT_ECC;
    status = flash_command_sequence(config);

    if (kStatus_FLASH_Success == status)
    {
        eccLog->firstEccEventAddress = FLASH->DATAW[0];
        eccLog->eccErrorCount = FLASH->DATAW[1];
        eccLog->eccCorrectionCount = FLASH->DATAW[2];
    }

    return status;
}

void flash_get_size(uint32_t *size)
{
    uint32_t flashSizeCode;
    //flashSizeCode = (SYSCON->FLASHSIZECFG & SYSCON_FLASHSIZECFG_FLASHSIZE_MASK) >> SYSCON_FLASHSIZECFG_FLASHSIZE_SHIFT;
    flashSizeCode = ((*(volatile uint32_t *)(SYSCON_BASE + 0xFE0)) & 0x1FFU) >> 0;

    bool hasFound = false;
    for (uint32_t i = 0u; i < ARRAY_SIZE(k_flashSizeCodeArray); i++)
    {
        if (k_flashSizeCodeArray[i].flashSizeCode == flashSizeCode)
        {
            hasFound = true;
            *size = k_flashSizeCodeArray[i].flashSize;
            break;
        }
    }
    if (!hasFound)
    {
        *size = k_flashSizeCodeArray[ARRAY_SIZE(k_flashSizeCodeArray) - 1].flashSize;
    }
}

void flash_reset(void)
{
    /* sw reset notes:
       1. When entering the reset mode, all controller registers will be initialized to
       the value specified in the relative register description. Any command or bus
       transaction in progress is interrupted as well, with no regards for data integrity.
       2. Depending on the context and on the suspected reason of the failure, a SW reset may be
       more or less suited than writing an initialization command code to the CMD register.
       In the reset case the full controller state is reset, in the command case it is kept
       (except for the side effect of initialization).
    */
    FLASH->EVENT |= FLASH_EVENT_RST_MASK;

    while (!(FLASH->INT_STATUS & FLASH_INT_STATUS_DONE_MASK))
        ;
}

/*!
 * @brief Flash Command Sequence
 *
 * This function is used to perform the command write sequence to the flash.
 *
 * @param driver Pointer to storage for the driver runtime state.
 * @return An error code or kStatus_FLASH_Success
 */
static status_t flash_command_sequence(flash_config_t *config)
{
    status_t status = kStatus_Fail;
    uint32_t registerValue;

    while (!(FLASH->INT_STATUS & FLASH_INT_STATUS_DONE_MASK))
        ;

    /* Check error bits */
    /* Get flash status register value */
    registerValue = FLASH->INT_STATUS;

    /* Checking access error */
    if (registerValue & FLASH_INT_STATUS_FAIL_MASK)
    {
        status = kStatus_FLASH_CommandFailure;
    }
    else if (registerValue & FLASH_INT_STATUS_ERR_MASK)
    {
        status = kStatus_FLASH_CommandNotSupported;
    }
    else if (registerValue & FLASH_INT_STATUS_ECC_ERR_MASK)
    {
        status = kStatus_FLASH_EccError;
    }
    //else if (registerValue & FLASH_INT_STATUS_OVL_MASK)
    else if (registerValue & 0x10U)
    {
        status = kStatus_FLASH_RegulationLoss;
    }
    else
    {
        status = kStatus_FLASH_Success;
    }

    return status;
}

/*! @brief Validates the range and alignment of the given address range.*/
static status_t flash_check_range(flash_config_t *config,
                                  uint32_t startAddress,
                                  uint32_t lengthInBytes,
                                  uint32_t alignmentBaseline)
{
    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    /* Verify the start and length are alignmentBaseline aligned. */
    if ((startAddress & (alignmentBaseline - 1)) || (lengthInBytes & (alignmentBaseline - 1)))
    {
        return kStatus_FLASH_AlignmentError;
    }

    /* check for valid range of the target addresses */
    if (((startAddress >= config->PFlashBlockBase) &&
         ((startAddress + lengthInBytes) <= (config->PFlashBlockBase + config->PFlashTotalSize))))
    {
        return kStatus_FLASH_Success;
    }

    if (((startAddress >= config->ffrConfig.ffrBlockBase) &&
         ((startAddress + lengthInBytes) <= (config->ffrConfig.ffrBlockBase + config->ffrConfig.ffrTotalSize))))
    {
        return kStatus_FLASH_Success;
    }

    return kStatus_FLASH_AddressError;
}

/*!
 * @brief Flash Cache Clear
 *
 * This function is used to perform the cache and prefetch speculation clear to the flash.
 */
void flash_cache_clear(flash_config_t *config)
{
    //SYSCON->FMC_FLUSH = 1;
    *(volatile uint32_t *)(SYSCON_BASE + 0x41C) = 1;
}

/*! @brief Validates the given user key for flash erase APIs.*/
static status_t flash_check_user_key(uint32_t key)
{
    /* Validate the user key */
    if (key != kFLASH_ApiEraseKey)
    {
        return kStatus_FLASH_EraseKeyError;
    }

    return kStatus_FLASH_Success;
}

/*! @brief Reads word from byte address.*/
static uint32_t flash_read_word_from_byte_address(uint8_t *src)
{
    uint32_t word = 0;

    if (!((uint32_t)src % 4))
    {
        word = *(uint32_t *)src;
    }
    else
    {
        for (uint32_t i = 0; i < 4; i++)
        {
            word |= (uint32_t)(*src) << (i * 8);
            src++;
        }
    }

    return word;
}

/*! @brief Writes word to byte address.*/
static void flash_write_word_to_byte_address(uint8_t *dst, uint32_t word)
{
    if (!((uint32_t)dst % 4))
    {
        *(uint32_t *)dst = word;
    }
    else
    {
        for (uint32_t i = 0; i < 4; i++)
        {
            *dst = (uint8_t)((word >> (i * 8)) & 0xFFU);
            dst++;
        }
    }
}
