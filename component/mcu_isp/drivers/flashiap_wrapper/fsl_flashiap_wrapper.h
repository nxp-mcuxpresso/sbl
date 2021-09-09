/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_FLASHIAP_WRAPPER_H_
#define _FSL_FLASHIAP_WRAPPER_H_

#include "fsl_common.h"
#include "flashiap/fsl_flashiap.h"

/*!
 * @addtogroup flashiap_wrapper_driver
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Flashiap driver version for ROM*/
enum _flashiap_driver_version_constants
{
    kFLASHIAP_DriverVersionName = 'F', /*!< Flashiap driver version name.*/
    kFLASHIAP_DriverVersionMajor = 2,  /*!< Major flashiap driver version.*/
    kFLASHIAP_DriverVersionMinor = 0,  /*!< Minor flashiap driver version.*/
    kFLASHIAP_DriverVersionBugfix = 0  /*!< Bugfix for flashiap driver version.*/
};
/*@}*/

/*!
 * @brief Flashiap status codes.
 */
enum _flashiap_wrapper_status
{
    kStatus_FLASHIAP_InvalidArgument = MAKE_STATUS(kStatusGroup_FLASHIAP, 50U), /*!< Invalid argument*/
    kStatus_FLASHIAP_UnknownProperty = MAKE_STATUS(kStatusGroup_FLASHIAP, 51U), /*!< Unknown property.*/
    kStatus_FLASHIAP_AlignmentError =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 52U), /*!< Parameter is not aligned with the specified baseline*/
    kStatus_FLASHIAP_AddressError = MAKE_STATUS(kStatusGroup_FLASHIAP, 53U),  /*!< Address is out of range */
    kStatus_FLASHIAP_EraseKeyError = MAKE_STATUS(kStatusGroup_FLASHIAP, 54U), /*!< API erase key is invalid.*/
    kStatus_FLASHIAP_MemoryNotblank =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 55U), /*!< memory to be verified are not blank.*/
};

/*!
 * @name Flashiap API key
 * @{
 */
/*! @brief Constructs the four character code for the Flashiap driver API key. */
#if !defined(FOUR_CHAR_CODE)
#define FOUR_CHAR_CODE(a, b, c, d) (((d) << 24) | ((c) << 16) | ((b) << 8) | ((a)))
#endif

/*!
 * @brief Enumeration for Flashiap driver API keys.
 *
 * @note The resulting value is built with a byte order such that the string
 * being readable in expected order when viewed in a hex editor, if the value
 * is treated as a 32-bit little endian value.
 */
enum _flashiap_driver_api_keys
{
    kFLASHIAP_ApiEraseKey =
        FOUR_CHAR_CODE('l', 'f', 'e', 'k') /*!< Key value used to validate all flashiap erase APIs.*/
};
/*@}*/

/*!
 * @brief Enumeration for various flashiap properties.
 */
typedef enum _flash_property_tag
{
    kFLASHIAP_PropertyPflashSectorSize = 0x00U,    /*!< Pflash sector size property.*/
    kFLASHIAP_PropertyPflashTotalSize = 0x01U,     /*!< Pflash total size property.*/
    kFLASHIAP_PropertyPflashBlockBaseAddr = 0x04U, /*!< Pflash block base address property.*/
    kFLASHIAP_PropertyFlashMemoryIndex = 0x20U,    /*!< Flash memory index property.*/
    kFLASHIAP_PropertyPflashPageSize = 0x30U,      /*!< Pflash page size property.*/
} flash_property_tag_t;

/*!
 * @brief Enumeration for flashiap property.
 */
enum _flashiap_memory_property
{
    kFLASHIAP_PageSize = FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES, /*!< Pflash page size in bytes.*/
    kFLASHIAP_RemappedMemoryBase = 0,                              /*!< Pflash memory base re-mapped to ROM.*/
    kFLASHIAP_RemappedMemorySize = 512,                            /*!< Pflash memory size re-mapped to ROM.*/
    kFLASHIAP_ErasedValue = ~0
};

/*!
 * @brief Enumeration for flashiap alignment property.
 */
enum _flashiap_alignment_property
{
    kFLASHIAP_AlignementUnitVerifyErase = 4,  /*!< The alignment unit in bytes used for verify erase operation.*/
    kFLASHIAP_AlignementUnitVerifyProgram = 4 /*!< The alignment unit in bytes used for verify program operation.*/
};

/*! @brief A callback type used for the Pflash block*/
typedef void (*flash_callback_t)(void);

/*! @brief Flashiap driver state information.
 *
 * An instance of this structure is allocated by the user of the flashiap driver and
 * passed into each of the driver APIs.
 */
typedef struct _flashiap_config
{
    uint32_t PFlashBlockBase;        /*!< A base address of the first PFlash block */
    uint32_t PFlashTotalSize;        /*!< The size of the combined PFlash block. */
    uint8_t Reserved0;               /*!< Reserved field 0 */
    uint8_t FlashMemoryIndex;        /*!< 0 - primary flash; 1 - secondary flash*/
    uint16_t PFlashPageSize;         /*!< The size in bytes of a page of PFlash. */
    uint32_t PFlashSectorSize;       /*!< The size in bytes of a sector of PFlash. */
    flash_callback_t PFlashCallback; /*!< The callback function for the flash API. */
} flashiap_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization
 * @{
 */

/*!
 * @brief Initializes the global flash properties structure members.
 *
 * This function checks and initializes the Flash module for the other Flash APIs.
 *
 * @param config Pointer to the storage for the driver runtime state.
 *
 * @retval #kStatus_FLASHIAP_Success API was executed successfully.
 * @retval #kStatus_FLASHIAP_InvalidArgument An invalid argument is provided.
 */
status_t FLASHIAP_Init(flashiap_config_t *config);

/*!
 * @brief Sets the desired flash callback function.
 *
 * @param config Pointer to the storage for the driver runtime state.
 * @param callback A callback function to be stored in the driver.
 *
 * @retval #kStatus_FLASHIAP_Success API was executed successfully.
 * @retval #kStatus_FLASHIAP_InvalidArgument An invalid argument is provided.
 */
status_t FLASHIAP_SetCallback(flashiap_config_t *config, flash_callback_t callback);

/*@}*/

/*!
 * @name Erasing
 * @{
 */

/*!
 * @brief Erases the flash sectors encompassed by parameters passed into function.
 *
 * This function erases the appropriate number of flash sectors based on the
 * desired start address and length.
 *
 * @param config The pointer to the storage for the driver runtime state.
 * @param start The start address of the desired flash memory to be erased.
 *              The start address does not need to be sector-aligned.
 * @param lengthInBytes The length, given in bytes (not words or long-words)
 *                      to be erased. Must be word-aligned.
 * @param key The value used to validate all flash erase APIs.
 *
 * @retval #kStatus_FLASHIAP_Success API was executed successfully.
 * @retval #kStatus_FLASHIAP_InvalidArgument An invalid argument is provided.
 * @retval #kStatus_FLASHIAP_AlignmentError The parameter is not aligned with the specified baseline.
 * @retval #kStatus_FLASHIAP_AddressError The address is out of range.
 * @retval #kStatus_FLASHIAP_EraseKeyError The API erase key is invalid.
 */
status_t FLASHIAP_Erase(flashiap_config_t *config, uint32_t start, uint32_t lengthInBytes, uint32_t key);

/*@}*/

/*!
 * @name Programming
 * @{
 */

/*!
 * @brief Programs flash with data at locations passed in through parameters.
 *
 * This function programs the flash memory with the desired data for a given
 * flash area as determined by the start address and the length.
 *
 * @param config A pointer to the storage for the driver runtime state.
 * @param start The start address of the desired flash memory to be programmed. Must be
 *              word-aligned.
 * @param src A pointer to the source buffer of data that is to be programmed
 *            into the flash.
 * @param lengthInBytes The length, given in bytes (not words or long-words),
 *                      to be programmed. Must be word-aligned.
 *
 * @retval #kStatus_FLASHIAP_Success API was executed successfully.
 * @retval #kStatus_FLASHIAP_InvalidArgument An invalid argument is provided.
 * @retval #kStatus_FLASHIAP_AlignmentError Parameter is not aligned with the specified baseline.
 * @retval #kStatus_FLASHIAP_AddressError Address is out of range.
 */
status_t FLASHIAP_Program(flashiap_config_t *config, uint32_t start, uint32_t *src, uint32_t lengthInBytes);

/*@}*/

/*!
 * @name Verification
 * @{
 */

/*!
 * @brief Verifies an erasure of the desired flash area.
 *
 * This function checks the appropriate number of flash sectors based on
 * the desired start address and length to check whether the flash is erased
 * to the specified read margin level.
 *
 * @param config A pointer to the storage for the driver runtime state.
 * @param start The start address of the desired flash memory to be verified.
 *        The start address does not need to be sector-aligned but must be word-aligned.
 * @param lengthInBytes The length, given in bytes (not words or long-words),
 *        to be verified. Must be word-aligned.
 *
 * @retval #kStatus_FLASHIAP_Success API was executed successfully.
 * @retval #kStatus_FLASHIAP_InvalidArgument An invalid argument is provided.
 * @retval #kStatus_FLASHIAP_AlignmentError Parameter is not aligned with specified baseline.
 * @retval #kStatus_FLASHIAP_AddressError Address is out of range.
 */
status_t FLASHIAP_VerifyErase(flashiap_config_t *config, uint32_t start, uint32_t lengthInBytes);

/*!
 * @brief Verifies programming of the desired flash area.
 *
 * This function verifies the data programed in the flash memory using the
 * Flash Program Check Command and compares it to the expected data for a given
 * flash area as determined by the start address and length.
 *
 * @param config A pointer to the storage for the driver runtime state.
 * @param start The start address of the desired flash memory to be verified. Must be word-aligned.
 * @param lengthInBytes The length, given in bytes (not words or long-words),
 *        to be verified. Must be word-aligned.
 * @param expectedData A pointer to the expected data that is to be
 *        verified against.
 * @param failedAddress A pointer to the returned failing address.
 * @param failedData A pointer to the returned failing data.
 *
 * @retval #kStatus_FLASHIAP_Success API was executed successfully.
 * @retval #kStatus_FLASHIAP_InvalidArgument An invalid argument is provided.
 * @retval #kStatus_FLASHIAP_AlignmentError Parameter is not aligned with specified baseline.
 * @retval #kStatus_FLASHIAP_AddressError Address is out of range.
 */
status_t FLASHIAP_VerifyProgram(flashiap_config_t *config,
                                uint32_t start,
                                uint32_t lengthInBytes,
                                const uint32_t *expectedData,
                                uint32_t *failedAddress,
                                uint32_t *failedData);

/*@}*/

/*!
 * @name Properties
 * @{
 */

/*!
 * @brief Returns the desired flash property.
 *
 * @param config A pointer to the storage for the driver runtime state.
 * @param whichProperty The desired property from the list of properties in
 *        enum flash_property_tag_t
 * @param value A pointer to the value returned for the desired flash property.
 *
 * @retval #kStatus_FLASHIAP_Success API was executed successfully.
 * @retval #kStatus_FLASHIAP_InvalidArgument An invalid argument is provided.
 * @retval #kStatus_FLASHIAP_UnknownProperty An unknown property tag.
 */
status_t FLASHIAP_GetProperty(flashiap_config_t *config, flash_property_tag_t whichProperty, uint32_t *value);

/*@}*/

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* _FSL_FLASHIAP_WRAPPER_H_ */
