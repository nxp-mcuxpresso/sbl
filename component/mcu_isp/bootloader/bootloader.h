/*
 * Copyright (c) 2013-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(__BOOTLOADER_H__)
#define __BOOTLOADER_H__

#include "bootloader_common.h"
#include "bootloader/bl_peripheral.h"
#include "bootloader/bl_command.h"
#include "bootloader/bl_context.h"
#include "bootloader/bl_version.h"
#include "bootloader/bl_peripheral_interface.h"
#include "bootloader/bl_shutdown_cleanup.h"
#include "bootloader/bl_cache.h"
#include "bootloader/bl_log.h"
#include "memory/memory.h"
#include "property/property.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Bootloader status codes.
//! @ingroup bl_core
enum _bootloader_status
{
    kStatus_UnknownCommand = MAKE_STATUS(kStatusGroup_Bootloader, 0),
    kStatus_SecurityViolation = MAKE_STATUS(kStatusGroup_Bootloader, 1),
    kStatus_AbortDataPhase = MAKE_STATUS(kStatusGroup_Bootloader, 2),
    kStatus_Ping = MAKE_STATUS(kStatusGroup_Bootloader, 3),
    kStatus_NoResponse = MAKE_STATUS(kStatusGroup_Bootloader, 4),
    kStatus_NoResponseExpected = MAKE_STATUS(kStatusGroup_Bootloader, 5),
    kStatus_CommandUnsupported = MAKE_STATUS(kStatusGroup_Bootloader, 6),
};

//!@brief Secure Bool definition
typedef enum _secure_bool
{
    kSecure_True = 0x3c6a9559u,
    kSecure_False = 0xc3956aa6u,
} secure_bool_t;

// !@brief secure substract
#ifndef SECURE_SUB
#define SECURE_SUB(c, a, b, r) \
    do                         \
    {                          \
        if ((b) > (a))         \
        {                      \
            r = kSecure_False; \
        }                      \
        else                   \
        {                      \
            c = a - b;         \
            r = kSecure_True;  \
        }                      \
    } while (0)
#endif

//!@brief secure add
#ifndef SECURE_ADD
#define SECURE_ADD(c, a, b, r)              \
    do                                      \
    {                                       \
        uint64_t sum = (uint64_t)(a) + (b); \
        if (sum > UINT32_MAX)               \
        {                                   \
            r = kSecure_False;              \
        }                                   \
        else                                \
        {                                   \
            c = (a) + (b);                  \
            r = kSecure_True;               \
        }                                   \
    } while (0)
#endif

//! @brief Root of the bootloader API tree.
//!
//! An instance of this struct resides in read-only memory in the bootloader. It
//! provides a user application access to APIs exported by the bootloader.
//!
//! @note The order of existing fields must not be changed.
//!
//! @ingroup context
#if 0 // Moved into each SOC based header file in future !!!!!!!!!!!!!
typedef struct BootloaderTree
{
    void (*runBootloader)(void *arg);            //!< Function to start the bootloader executing.
    standard_version_t version;                  //!< Bootloader version number.
    const char *copyright;                       //!< Copyright string.
    const bootloader_context_t *runtimeContext;  //!< Pointer to the bootloader's runtime context.
    const flash_driver_interface_t *flashDriver; //!< Flash driver API.
    const aes_driver_interface_t *aesDriver;     //!< AES driver API.
} bootloader_tree_t;
#endif

//! @brief Boot mode definitions
//! @ingroup bl_core
enum
{
    kBootMode_Master = 0x00,
    kBootMode_Isp = 0x5,
    kBootMode_Test = 0x1,
    kBootMode_Passive = 0x0c,
    kBootMode_DebugSession = 0x0a,
    kBootMode_Reserved = 0xF,
};

///////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Verify that a given address is ok to jump to.
 *
 * @param applicationAddress The entry point address to validate.
 * @return Boolean indicating whether the address is valid.
 *
 * @ingroup bl_core
 */
bool is_valid_application_location(uint32_t applicationAddress);

/*!
 * @brief Verify that a given address is ok to set as stack pointer base address.
 *
 * @param stackpointerAddress The stack pointer address to validate.
 * @return Boolean indicating whether the address is valid.
 *
 * @ingroup bl_core
 */
bool is_valid_stackpointer_location(uint32_t stackpointerAddress);

/*!
 * @brief Go passive boot flow
 */
int go_passive_boot(bool isInfiniteIsp);

//!@brief Go Fatal flow
void go_fatal_mode(void);

/*!
 * @brief Go ISP boot flow
 */
void go_isp_boot(void);

//!@brief Get isp pin status
bool is_isp_pin_asserted(void);

/*!
 * @brief Reset the device
 */
void target_reset(void);

/*!
 * @brief stack check initialization
 */
void stack_check_init(void);

#if defined(__cplusplus)
}
#endif

#endif // __BOOTLOADER_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
