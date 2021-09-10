/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "fsl_iap.h"
#include "fsl_iap_kbp.h"
#include "fsl_iap_skboot_authenticate.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.iap"
#endif

/*!
 * @addtogroup rom_api
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief FLEXSPI Flash driver API Interface */
typedef struct
{
    uint32_t version;
    status_t (*init)(uint32_t instance, flexspi_nor_config_t *config);
    status_t (*page_program)(uint32_t instance, flexspi_nor_config_t *config, uint32_t dstAddr, const uint32_t *src);
    status_t (*erase_all)(uint32_t instance, flexspi_nor_config_t *config);
    status_t (*erase)(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length);
    status_t (*erase_sector)(uint32_t instance, flexspi_nor_config_t *config, uint32_t address);
    status_t (*erase_block)(uint32_t instance, flexspi_nor_config_t *config, uint32_t address);
    status_t (*get_config)(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option);
    status_t (*read)(uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes);
    status_t (*xfer)(uint32_t instance, flexspi_xfer_t *xfer);
    status_t (*update_lut)(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq);
    status_t (*set_clock_source)(uint32_t clockSrc);
    void (*config_clock)(uint32_t instance, uint32_t freqOption, uint32_t sampleClkMode);
} flexspi_nor_flash_driver_t;

/*! @brief OTP driver API Interface */
typedef struct
{
    status_t (*init)(uint32_t src_clk_freq);
    status_t (*deinit)(void);
    status_t (*fuse_read)(uint32_t addr, uint32_t *data);
    status_t (*fuse_program)(uint32_t addr, uint32_t data, bool lock);
    status_t (*crc_calc)(uint32_t *src, uint32_t numberOfWords, uint32_t *crcChecksum);
    status_t (*reload)(void);
    status_t (*crc_check)(uint32_t start_addr, uint32_t end_addr, uint32_t crc_addr);
} ocotp_driver_t;

/*!
 * @name Bootloader API and image authentication Structure
 * @{
 */

/*! @brief Interface for Bootloader API functions. */
typedef struct _kb_interface
{
    /*!< Initialize the API. */
    status_t (*kb_init_function)(kb_session_ref_t **session, const kb_options_t *options);
    status_t (*kb_deinit_function)(kb_session_ref_t *session);
    status_t (*kb_execute_function)(kb_session_ref_t *session, const uint8_t *data, uint32_t dataLength);
} kb_interface_t;

/*! @brief Interface for image authentication API */
typedef struct _skboot_authenticate_interface
{
    skboot_status_t (*skboot_authenticate_function)(const uint8_t *imageStartAddr, secure_bool_t *isSignVerified);
    void (*skboot_hashcrypt_irq_handler)(void);
} skboot_authenticate_interface_t;

/*!
 * @brief Root of the bootloader API tree.
 *
 * An instance of this struct resides in read-only memory in the bootloader. It
 * provides a user application access to APIs exported by the bootloader.
 *
 * @note The order of existing fields must not be changed.
 */
typedef struct BootloaderTree
{
    void (*runBootloader)(iap_boot_option_t *arg); /*!< Function to start the bootloader executing. */
    uint32_t version;                 /*!< Bootloader version number. */
    const char *copyright;            /*!< Copyright string. */
    const uint32_t reserved0;
    const kb_interface_t *kbApi;      /*!< Bootloader API. */
    const uint32_t reserved2;
    const uint32_t reserved3;
    const flexspi_nor_flash_driver_t *flexspiNorDriver; /*!< FlexSPI NOR FLASH Driver API. */
    const ocotp_driver_t *otpDriver;                    /*!< OTP driver API. */
    const skboot_authenticate_interface_t *skbootAuthenticate; /*!< Image authentication API. */
} bootloader_tree_t;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ROM_API_TREE                ((uint32_t *)FSL_ROM_API_BASE_ADDR)
#define BOOTLOADER_API_TREE_POINTER ((bootloader_tree_t *)ROM_API_TREE)

/*! Get pointer to flexspi/otp driver API table in ROM. */
#define FLEXSPI_API_TREE BOOTLOADER_API_TREE_POINTER->flexspiNorDriver
#define OTP_API_TREE     BOOTLOADER_API_TREE_POINTER->otpDriver

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * runBootloader API
 ******************************************************************************/
void IAP_RunBootLoader(iap_boot_option_t *option)
{
    BOOTLOADER_API_TREE_POINTER->runBootloader(option);
}

/*******************************************************************************
 * FlexSPI NOR driver
 ******************************************************************************/
status_t IAP_FlexspiNorInit(uint32_t instance, flexspi_nor_config_t *config)
{
    return FLEXSPI_API_TREE->init(instance, config);
}

status_t IAP_FlexspiNorPageProgram(uint32_t instance,
                                   flexspi_nor_config_t *config,
                                   uint32_t dstAddr,
                                   const uint32_t *src)
{
    return FLEXSPI_API_TREE->page_program(instance, config, dstAddr, src);
}

status_t IAP_FlexspiNorEraseAll(uint32_t instance, flexspi_nor_config_t *config)
{
    return FLEXSPI_API_TREE->erase_all(instance, config);
}

status_t IAP_FlexspiNorErase(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length)
{
    return FLEXSPI_API_TREE->erase(instance, config, start, length);
}

status_t IAP_FlexspiNorEraseSector(uint32_t instance, flexspi_nor_config_t *config, uint32_t address)
{
    return FLEXSPI_API_TREE->erase_sector(instance, config, address);
}

status_t IAP_FlexspiNorEraseBlock(uint32_t instance, flexspi_nor_config_t *config, uint32_t address)
{
    return FLEXSPI_API_TREE->erase_block(instance, config, address);
}

status_t IAP_FlexspiNorGetConfig(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option)
{
    return FLEXSPI_API_TREE->get_config(instance, config, option);
}

status_t IAP_FlexspiNorRead(
    uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes)
{
    return FLEXSPI_API_TREE->read(instance, config, dst, start, bytes);
}

status_t IAP_FlexspiXfer(uint32_t instance, flexspi_xfer_t *xfer)
{
    return FLEXSPI_API_TREE->xfer(instance, xfer);
}

status_t IAP_FlexspiUpdateLut(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq)
{
    return FLEXSPI_API_TREE->update_lut(instance, seqIndex, lutBase, numberOfSeq);
}

status_t IAP_FlexspiSetClockSource(uint32_t clockSrc)
{
    return FLEXSPI_API_TREE->set_clock_source(clockSrc);
}

void IAP_FlexspiConfigClock(uint32_t instance, uint32_t freqOption, uint32_t sampleClkMode)
{
    FLEXSPI_API_TREE->config_clock(instance, freqOption, sampleClkMode);
}

AT_QUICKACCESS_SECTION_CODE(status_t IAP_FlexspiNorAutoConfig(uint32_t instance,
                                                              flexspi_nor_config_t *config,
                                                              serial_nor_config_option_t *option))
{
    /* Wait until the FLEXSPI is idle */
    register uint32_t delaycnt = 10000u;
    while (delaycnt--)
    {
    }
    status_t status = FLEXSPI_API_TREE->get_config(instance, config, option);
    if (status != kStatus_Success)
    {
        return status;
    }

    return FLEXSPI_API_TREE->init(instance, config);
}

/*******************************************************************************
 * OTP driver
 ******************************************************************************/
status_t IAP_OtpInit(uint32_t src_clk_freq)
{
    return OTP_API_TREE->init(src_clk_freq);
}

status_t IAP_OtpDeinit(void)
{
    return OTP_API_TREE->deinit();
}

status_t IAP_OtpFuseRead(uint32_t addr, uint32_t *data)
{
    return OTP_API_TREE->fuse_read(addr, data);
}

status_t IAP_OtpFuseProgram(uint32_t addr, uint32_t data, bool lock)
{
    return OTP_API_TREE->fuse_program(addr, data, lock);
}

status_t IAP_OtpCrcCalc(uint32_t *src, uint32_t numberOfWords, uint32_t *crcChecksum)
{
    return OTP_API_TREE->crc_calc(src, numberOfWords, crcChecksum);
}

status_t IAP_OtpShadowRegisterReload(void)
{
    return OTP_API_TREE->reload();
}

status_t IAP_OtpCrcCheck(uint32_t start_addr, uint32_t end_addr, uint32_t crc_addr)
{
    return OTP_API_TREE->crc_check(start_addr, end_addr, crc_addr);
}

/********************************************************************************
 * Bootloader API
 *******************************************************************************/
/*!
 * @brief Initialize ROM API for a given operation.
 *
 * Inits the ROM API based on the options provided by the application in the second
 * argument. Every call to rom_init() should be paired with a call to rom_deinit().
 */
status_t kb_init(kb_session_ref_t **session, const kb_options_t *options)
{
    assert(BOOTLOADER_API_TREE_POINTER);
    return BOOTLOADER_API_TREE_POINTER->kbApi->kb_init_function(session, options);
}

/*!
 * @brief Cleans up the ROM API context.
 *
 * After this call, the @a context parameter can be reused for another operation
 * by calling rom_init() again.
 */
status_t kb_deinit(kb_session_ref_t *session)
{
    assert(BOOTLOADER_API_TREE_POINTER);
    return BOOTLOADER_API_TREE_POINTER->kbApi->kb_deinit_function(session);
}

/*!
 * Perform the operation configured during init.
 *
 * This application must call this API repeatedly, passing in sequential chunks of
 * data from the boot image (SB file) that is to be processed. The ROM will perform
 * the selected operation on this data and return. The application may call this
 * function with as much or as little data as it wishes, which can be used to select
 * the granularity of time given to the application in between executing the operation.
 *
 * @param context Current ROM context pointer.
 * @param data Buffer of boot image data provided to the ROM by the application.
 * @param dataLength Length in bytes of the data in the buffer provided to the ROM.
 *
 * @retval #kStatus_Success The operation has completed successfully.
 * @retval #kStatus_Fail An error occurred while executing the operation.
 * @retval #kStatus_RomApiNeedMoreData No error occurred, but the ROM needs more data to
 *     continue processing the boot image.
 */
status_t kb_execute(kb_session_ref_t *session, const uint8_t *data, uint32_t dataLength)
{
    assert(BOOTLOADER_API_TREE_POINTER);
    return BOOTLOADER_API_TREE_POINTER->kbApi->kb_execute_function(session, data, dataLength);
}

/********************************************************************************
 * Image authentication API
 *******************************************************************************/

/*!
 * @brief Authenticate entry function with ARENA allocator init
 *
 * This is called by ROM boot or by ROM API g_skbootAuthenticateInterface
 */
skboot_status_t skboot_authenticate(const uint8_t *imageStartAddr, secure_bool_t *isSignVerified)
{
    assert(BOOTLOADER_API_TREE_POINTER);
    return BOOTLOADER_API_TREE_POINTER->skbootAuthenticate->skboot_authenticate_function(imageStartAddr,
                                                                                         isSignVerified);
}

/*!
 * @brief Interface for image authentication API
 */
void HASH_IRQHandler(void)
{
    assert(BOOTLOADER_API_TREE_POINTER);
    BOOTLOADER_API_TREE_POINTER->skbootAuthenticate->skboot_hashcrypt_irq_handler();
}