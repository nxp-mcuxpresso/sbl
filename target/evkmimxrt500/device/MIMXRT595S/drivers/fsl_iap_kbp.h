/*
 * Copyright (c) 2020, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_IAP_KBP_H_
#define _FSL_IAP_KBP_H_

#include "fsl_common.h"

/*!
 * @addtogroup kb_driver
 * @{
 */
/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*! @brief ROM API version. */
enum _rom_api_version
{
    kRomApiVersion = 1,
};

typedef enum _keystore_storage_selection
{
    kQspiFlash_KeyStore = 0,
    kSdCard_KeyStore = 1,
    kMmcCard_KeyStore = 2,
    kSpiFlash_KeyStore = 3,
    kKeyStoreSel_End = kSpiFlash_KeyStore,
    kKeyStoreSel_Start = kQspiFlash_KeyStore,
} keystore_storage_selection_t;
     
/*!
 *  @brief Details of the operation to be performed by the ROM.
 *
 * The #kRomAuthenticateImage operation requires the entire signed image to be
 * available to the application.
 */
typedef enum _kb_operation
{
    kRomAuthenticateImage = 1, /*!< Authenticate a signed image.*/
    kRomLoadImage         = 2, /*!< Load SB file.*/
    kRomOperationCount    = 3,
} kb_operation_t;

/*!
 * @brief Security constraint flags, Security profile flags.
 */
enum _kb_security_profile
{
    kKbootMinRSA4096 = (1 << 16),
};

/*!
 * @brief Memory region definition.
 */
typedef struct _kb_region
{
    uint32_t address;
    uint32_t length;
} kb_region_t;

typedef struct _kb_load_sb
{
    uint32_t profile;
    uint32_t minBuildNumber;
    uint32_t overrideSBBootSectionID;
    uint32_t *userSBKEK;
    uint32_t regionCount;
    const kb_region_t *regions;
} kb_load_sb_t;

typedef struct _kb_authenticate
{
    uint32_t profile;
    uint32_t minBuildNumber;
    uint32_t maxImageLength;
    uint32_t *userRHK;
} kb_authenticate_t;

typedef struct _kb_options
{
    uint32_t version; /*!< Should be set to kKbootApiVersion.*/
    uint8_t *buffer;  /*!< Caller-provided buffer used by Kboot.*/
    uint32_t bufferLength;
    kb_operation_t op;
    union
    {
        kb_authenticate_t authenticate; /*! Settings for kKbootAuthenticate operation.*/
        kb_load_sb_t loadSB;            /*! Settings for kKbootLoadSB operation.*/
    };
} kb_options_t;

/*!
 * @brief Interface to memory operations for one region of memory.
 */
typedef struct _memory_region_interface
{
    status_t (*init)(void);
    status_t (*read)(uint32_t address, uint32_t length, uint8_t *buffer);
    status_t (*write)(uint32_t address, uint32_t length, const uint8_t *buffer);
    status_t (*fill)(uint32_t address, uint32_t length, uint32_t pattern);
    status_t (*flush)(void);
    status_t (*erase)(uint32_t address, uint32_t length);
    status_t (*config)(uint32_t *buffer);
    status_t (*erase_all)(void);
} memory_region_interface_t;

/*!
 * @brief Structure of a memory map entry.
 */
typedef struct _memory_map_entry
{
    uint32_t startAddress;
    uint32_t endAddress;
    uint32_t memoryProperty;
    uint32_t memoryId;
    const memory_region_interface_t *memoryInterface;
} memory_map_entry_t;

typedef struct _kb_opaque_session_ref
{
    kb_options_t context;
    bool cau3Initialized;
    memory_map_entry_t *memoryMap;
} kb_session_ref_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize ROM API for a given operation.
 *
 * Inits the ROM API based on the options provided by the application in the second
 * argument. Every call to rom_init() should be paired with a call to rom_deinit().
 */
status_t kb_init(kb_session_ref_t **session, const kb_options_t *options);

/*!
 * @brief Cleans up the ROM API context.
 *
 * After this call, the context parameter can be reused for another operation
 * by calling rom_init() again.
 */
status_t kb_deinit(kb_session_ref_t *session);

/*!
 * Perform the operation configured during init.
 *
 * This application must call this API repeatedly, passing in sequential chunks of
 * data from the boot image (SB file) that is to be processed. The ROM will perform
 * the selected operation on this data and return. The application may call this
 * function with as much or as little data as it wishes, which can be used to select
 * the granularity of time given to the application in between executing the operation.
 *
 * @param session Current ROM context pointer.
 * @param data Buffer of boot image data provided to the ROM by the application.
 * @param dataLength Length in bytes of the data in the buffer provided to the ROM.
 *
 * @retval kStatus_RomApiExecuteSuccess ROM successfully process the part of sb file/boot image.
 * @retval kStatus_RomApiExecuteCompleted ROM successfully process the whole sb file/boot image.
 * @retval kStatus_Fail An error occurred while executing the operation.
 * @retval kStatus_RomApiNeedMoreData No error occurred, but the ROM needs more data to
 *     continue processing the boot image.
 * @retval kStatus_RomApiBufferSizeNotEnough user buffer is not enough for
 *     use by Kboot during execution of the operation.
 * @retval kStatus_RomApiBufferNotOkForArena user buffer does't meet the requirement
 *     of arena memory.
 */
status_t kb_execute(kb_session_ref_t *session, const uint8_t *data, uint32_t dataLength);

#if defined(__cplusplus)
}
#endif

/*!
 *@}
 */

#endif /* _FSL_IAP_KBP_H_ */
