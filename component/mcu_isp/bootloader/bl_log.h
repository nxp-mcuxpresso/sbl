/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __BL_LOG_H__
#define __BL_LOG_H__

#include "bootloader_common.h"

#ifndef BL_LOG_MAX_ENTRY_NUM
#define BL_LOG_MAX_ENTRY_NUM 64ul
#endif // BL_LOG_MAX_ENTRY_NUM

//!@brief Log Boot flow defifntions
typedef enum _log_boot_state
{
    kLog_Startup = 0ul,
    kLog_HardwareInit = 1ul,
    kLog_ROMPatch = 2ul,
    kLog_SecureBoot = 3ul,
    kLog_BootMode = 4ul,
    kLog_MasterBoot = 5ul,
    kLog_PassiveBoot = 6ul,
    kLog_RecoveryBoot = 7ul,
    kLog_IspBoot = 8ul,
    kLog_BootDevice = 9ul,
    kLog_ImgLoad = 0xaul,
    kLog_Auth = 0xbul,
    kLog_Jump = 0xcul,
    kLog_Fatal = 0xFFul,

} log_boot_flow_t;

typedef struct
{
    uint8_t remainingEntries;
    uint8_t status;
    uint8_t subState;
    uint8_t bootState;
} log_entry_t;

typedef enum _log_status
{
    //@!Status defintions, the value 0x00-0x2F is flow specific
    kLog_Status_Pass = 0xf0ul,
    kLog_Status_Fail = 0xf3ul,
    kLog_Status_Fatal = 0xf5ul,
    kLog_Status_Invalid = 0xfcul,
    kLog_Status_Default = 0xfful,

    kLog_Status_Disabled = 0xdcul,
    kLog_Status_Enabled = 0xeaul,

    kLog_Status_MasterBoot = 0x01ul,
    kLog_Status_PassiveBoot = 0x02ul,
    kLog_Status_RecoveryBoot = 0x03ul,
    kLog_Status_IspBoot = 0x04ul,
} log_status_t;

//!@brief Log Sub state definitions
//!       0x00-0x1F common sub states
//!       0x20-0x2F  Image Load sub states
//!       0x30-0x6F  Image authentication sub states
//!       0x7F-0xDF  Reserved
//!       0xE0-0xFF  Device specific sub states
typedef enum _log_substate
{
    //!@ Commmon Sub States, 0x00-0x1F
    kLog_SubState_Common = 0x00ul,
    kLog_SubState_Init = 0x01ul,
    kLog_SubState_Call = 0x02ul,
    kLog_SubState_DeInit = 0x03ul,
    kLog_SubState_Source = 0x04ul,
    kLog_SubState_Check = 0x05ul,
} log_substate_t;

//!@brief Image load Substate defintions
typedef enum _log_ImgLoad_substate
{
    //!@ Image Load substate:0x20-0x2F
    kLog_SubState_InitialLoad = 0x20ul,
    kLog_SubState_RemainingLoad = 0x21ul,
} log_ImgLoad_substate_t;

//!@brief Image authentication Substate defintions
typedef enum _log_auth_substate
{
    //!@ Image authentication Substats: 0x30-0x6F
    kLog_AuthState_CheckSecureState = 0x30ul,
    kLog_AuthState_ImageTypeCheck,
    kLog_AuthState_ReadKeyStore,
    kLog_AuthState_ReadMacKey,
    kLog_AuthState_VerifyHMAC,
    kLog_AuthState_ImageEntryCheck,
    kLog_AuthState_Authenticate,
    kLog_AuthState_CrcCheck,
} log_auth_substate_t;

#define MAKE_LOG_ENTRY(state, substate, status, entries) \
    (((uint32_t)(state) << 24u) | ((substate) << 16u) | ((status) << 8u) | (entries))

//!@brief ROM Log constants
enum
{
    // Log for startup
    kLog_Startup_Init_Pass = MAKE_LOG_ENTRY(kLog_Startup, kLog_SubState_Init, kLog_Status_Pass, 0),
    kLog_Startup_Init_Fail = MAKE_LOG_ENTRY(kLog_Startup, kLog_SubState_Init, kLog_Status_Fail, 0),

    // Log for Hardware initialization
    kLog_HardwareInit_Pass = MAKE_LOG_ENTRY(kLog_HardwareInit, kLog_SubState_Init, kLog_Status_Pass, 0),
    kLog_HardwareInit_Fail = MAKE_LOG_ENTRY(kLog_HardwareInit, kLog_SubState_Init, kLog_Status_Fail, 0),

    kLog_HardwareDeInit_Pass = MAKE_LOG_ENTRY(kLog_HardwareInit, kLog_SubState_DeInit, kLog_Status_Pass, 0),
    kLog_HardwareDeInit_Fail = MAKE_LOG_ENTRY(kLog_HardwareInit, kLog_SubState_DeInit, kLog_Status_Fail, 0),

    // Log for ROM patch
    kLog_Rompatch_Init_Pass = MAKE_LOG_ENTRY(kLog_ROMPatch, kLog_SubState_Init, kLog_Status_Pass, 0),
    kLog_Rompatch_Init_Fail = MAKE_LOG_ENTRY(kLog_ROMPatch, kLog_SubState_Init, kLog_Status_Fail, 0),

    // Log for secure boot
    kLog_SecureBoot_Disabled = MAKE_LOG_ENTRY(kLog_SecureBoot, 0, kLog_Status_Disabled, 0),
    kLog_SecureBoot_Enabled = MAKE_LOG_ENTRY(kLog_SecureBoot, 0, kLog_Status_Enabled, 0),
    kLog_SecureBoot_Init_Pass = MAKE_LOG_ENTRY(kLog_SecureBoot, kLog_SubState_Init, kLog_Status_Pass, 0),
    kLog_SecureBoot_Init_Fail = MAKE_LOG_ENTRY(kLog_SecureBoot, kLog_SubState_Init, kLog_Status_Fail, 0),

    // Log for boot mode
    kLog_BootMode_Source_Fuse = MAKE_LOG_ENTRY(kLog_BootMode, kLog_SubState_Source, 1, 0),
    kLog_BootMode_Source_BootPin = MAKE_LOG_ENTRY(kLog_BootMode, kLog_SubState_Source, 2, 0),
    kLog_BootMode_Masterboot = MAKE_LOG_ENTRY(kLog_BootMode, kLog_SubState_Check, kLog_Status_MasterBoot, 0),
    kLog_BootMode_Passiveboot = MAKE_LOG_ENTRY(kLog_BootMode, kLog_SubState_Check, kLog_Status_PassiveBoot, 0),
    kLog_BootMode_Recoveryboot = MAKE_LOG_ENTRY(kLog_BootMode, kLog_SubState_Check, kLog_Status_RecoveryBoot, 0),
    kLog_BootMode_Ispboot = MAKE_LOG_ENTRY(kLog_BootMode, kLog_SubState_Check, kLog_Status_IspBoot, 0),

    // Log for Master boot
    kLog_Masterboot_Init_Pass = MAKE_LOG_ENTRY(kLog_MasterBoot, kLog_SubState_Init, kLog_Status_Pass, 0),
    kLog_Masterboot_Init_Fail = MAKE_LOG_ENTRY(kLog_MasterBoot, kLog_SubState_Init, kLog_Status_Fail, 0),

    // Log for passive boot
    kLog_Passiveboot_Init_Pass = MAKE_LOG_ENTRY(kLog_PassiveBoot, kLog_SubState_Init, kLog_Status_Pass, 0),
    kLog_Passiveboot_Init_Fail = MAKE_LOG_ENTRY(kLog_PassiveBoot, kLog_SubState_Init, kLog_Status_Fail, 0),

    // Log for recovery boot
    kLog_Recoveryboot_Init_Pass = MAKE_LOG_ENTRY(kLog_RecoveryBoot, kLog_SubState_Init, kLog_Status_Pass, 0),
    kLog_Recoveryboot_Init_Fail = MAKE_LOG_ENTRY(kLog_RecoveryBoot, kLog_SubState_Init, kLog_Status_Fail, 0),
    kLog_Recoveryboot_Fail_Reason = MAKE_LOG_ENTRY(kLog_RecoveryBoot, kLog_SubState_Call, kLog_Status_Fail, 1),

    // Log for ISP boot
    kLog_Ispboot_Init_Pass = MAKE_LOG_ENTRY(kLog_IspBoot, kLog_SubState_Init, kLog_Status_Pass, 0),
    kLog_Ispboot_Init_Fail = MAKE_LOG_ENTRY(kLog_IspBoot, kLog_SubState_Init, kLog_Status_Fail, 0),

    // Image Load
    kLog_ImgLoad_InitLoad_Pass = MAKE_LOG_ENTRY(kLog_ImgLoad, kLog_SubState_InitialLoad, kLog_Status_Pass, 0),
    kLog_ImgLoad_InitLoad_Fail = MAKE_LOG_ENTRY(kLog_ImgLoad, kLog_SubState_InitialLoad, kLog_Status_Fail, 0),

    kLog_ImgLoad_RemainingLoad_Pass = MAKE_LOG_ENTRY(kLog_ImgLoad, kLog_SubState_RemainingLoad, kLog_Status_Pass, 0),
    kLog_ImgLoad_RemainingLoad_Fail = MAKE_LOG_ENTRY(kLog_ImgLoad, kLog_SubState_RemainingLoad, kLog_Status_Fail, 0),

    // Log for boot interface
    kLog_BootDevice_Check = MAKE_LOG_ENTRY(kLog_BootDevice, kLog_SubState_Check, kLog_Status_Pass, 1),
    kLog_BootDevice_Init_Pass = MAKE_LOG_ENTRY(kLog_BootDevice, kLog_SubState_Init, kLog_Status_Pass, 0),
    kLog_BootDevice_Init_Fail = MAKE_LOG_ENTRY(kLog_BootDevice, kLog_SubState_Init, kLog_Status_Fail, 0),
    kLog_BootDevice_Read_Pass = MAKE_LOG_ENTRY(kLog_BootDevice, kLog_SubState_Call, kLog_Status_Pass, 0),
    kLog_BootDevice_Read_Fail = MAKE_LOG_ENTRY(kLog_BootDevice, kLog_SubState_Call, kLog_Status_Fail, 0),
    kLog_BootDevice_Read_Invalid = MAKE_LOG_ENTRY(kLog_BootDevice, kLog_SubState_Call, kLog_Status_Invalid, 0),

    // Log for authentication
    kLog_Auth_SecureBootEnabled = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_CheckSecureState, kLog_Status_Enabled, 0),
    kLog_Auth_SecureBootDisabled = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_CheckSecureState, kLog_Status_Disabled, 0),

    kLog_Auth_ImageTypeCheck_Pass = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_ImageTypeCheck, kLog_Status_Pass, 0),
    kLog_Auth_ImageTypeCheck_Fail = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_ImageTypeCheck, kLog_Status_Fail, 0),

    kLog_Auth_ImageEntryCheck_Pass = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_ImageEntryCheck, kLog_Status_Pass, 0),
    kLog_Auth_ImageEntryCheck_Fail = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_ImageEntryCheck, kLog_Status_Fail, 0),

    kLog_Auth_ReadKeyStore_Pass = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_ReadKeyStore, kLog_Status_Pass, 0),
    kLog_Auth_ReadKeyStore_Fail = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_ReadKeyStore, kLog_Status_Fail, 0),

    kLog_Auth_ReadMacKey_Pass = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_ReadMacKey, kLog_Status_Pass, 0),
    kLog_Auth_ReadMacKey_Fail = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_ReadMacKey, kLog_Status_Fail, 0),

    kLog_Auth_VerifyHMAC_Pass = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_VerifyHMAC, kLog_Status_Pass, 0),
    kLog_Auth_VerifyHMAC_Fail = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_VerifyHMAC, kLog_Status_Fail, 0),

    kLog_Auth_Auth_Pass = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_Authenticate, kLog_Status_Pass, 0),
    kLog_Auth_Auth_Pass_Time = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_Authenticate, kLog_Status_Pass, 1),
    kLog_Auth_Auth_Fail = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_Authenticate, kLog_Status_Fail, 0),
    kLog_Auth_Auth_Fail_Time = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_Authenticate, kLog_Status_Fail, 1),

    kLog_Auth_CrcCheck_Pass = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_CrcCheck, kLog_Status_Pass, 0),
    kLog_Auth_CrcCheck_Pass_Time = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_CrcCheck, kLog_Status_Pass, 1),
    kLog_Auth_CrcCheck_Fail = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_CrcCheck, kLog_Status_Fail, 0),
    kLog_Auth_CrcCheck_Fail_Time = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_CrcCheck, kLog_Status_Fail, 1),
    kLog_Auth_CrcCheck_Invalid = MAKE_LOG_ENTRY(kLog_Auth, kLog_AuthState_CrcCheck, kLog_Status_Invalid, 0),

    // Log for jump
    kLog_Jump_Pass = MAKE_LOG_ENTRY(kLog_Jump, 0, kLog_Status_Pass, 0),
    kLog_Jump_Pass_Entry = MAKE_LOG_ENTRY(kLog_Jump, 0, kLog_Status_Pass, 1),
    kLog_Jump_Fail = MAKE_LOG_ENTRY(kLog_Jump, 0, kLog_Status_Fail, 0),
    kLog_Jump_Fail_Reason = MAKE_LOG_ENTRY(kLog_Jump, 0, kLog_Status_Fail, 1),
    kLog_Jump_Fail_Fatal = MAKE_LOG_ENTRY(kLog_Jump, 0, kLog_Status_Fatal, 0),

    kLog_FatalError = (int32_t)MAKE_LOG_ENTRY(kLog_Fatal, 0, 0, 0),

};

#if BL_FETAURE_LOG_ENABLE
#define LOG_INIT bl_log_init
#define LOG_ADD_ENTRY bl_log_add_entry
#define LOG_ADD_ENTRY_DATA bl_log_add_entry_data
#define LOG_TRAVERSE bl_log_tranverse
#define LOG_GET_LAST_ERROR bl_log_get_last_error
#else
#define LOG_INIT()
#define LOG_ADD_ENTRY(...)
#define LOG_ADD_ENTRY_DATA(...)
#define LOG_TRAVERSE()
#define LOG_GET_LAST_ERROR(...)
#endif

void bl_log_init(void);
void bl_log_add_entry(uint32_t log_entry);
void bl_log_add_entry_data(uint32_t log_entry, uint32_t *log_data, uint32_t numberOfEntries);

void bl_log_get_last_error(const void **buffer, uint32_t *count);

void bl_log_tranverse(void);

#endif // __BL_LOG_H__
