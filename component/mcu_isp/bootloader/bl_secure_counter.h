/*
* Copyright 2014-2016 Freescale Semiconductor, Inc.
* Copyright 2016-2018 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*
*/

#ifndef _BL_SECURE_COUNTER_H_
#define _BL_SECURE_COUNTER_H_

#include <stdint.h>
#include "bootloader_config.h"

/*!
 * @addtogroup bl_secure_counter
 * @{
 */
////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
typedef uint64_t secure_counter_t;

#define SC_SECURE_INIT_VAL (0x0f963c5a)

#if BL_FEATURE_SECURE_COUNTER
#define SC_INIT(val)                           \
    do                                         \
    {                                          \
        g_secure_counter = SC_SECURE_INIT_VAL; \
    } while (0)
#define SC_VAL() (g_secure_counter)
#define SC_PRINT()                                                     \
    do                                                                 \
    {                                                                  \
        debug_printf("Bootloader: SC value = %x\n", g_secure_counter); \
    } while (0)
#define SC_SET(val)               \
    do                            \
    {                             \
        g_secure_counter = (val); \
    } while (0)
#define SC_ADD(val)                                  \
    do                                               \
    {                                                \
        g_secure_counter += (secure_counter_t)(val); \
    } while (0)
#define SC_SUB(val)                                  \
    do                                               \
    {                                                \
        g_secure_counter -= (secure_counter_t)(val); \
    } while (0)
#if 1
#define SC_ASSERT(val)                                                                             \
    do                                                                                             \
    {                                                                                              \
        if (SC_VAL() ^ (secure_counter_t)(val))                                                    \
        {                                                                                          \
            debug_printf("Bootloader: SC assert fail, expecting %x, but get %x\n", val, SC_VAL()); \
            secure_counter_assert_fail();                                                          \
        }                                                                                          \
    } while (0)
#else
#define SC_ASSERT(val)
#endif
#else // #if BL_FEATURE_SECURE_COUNTER
#define SC_INIT(val)
#define SC_VAL()
#define SC_PRINT()
#define SC_SET(val)
#define SC_ADD(val)
#define SC_SUB(val)
#define SC_ASSERT(val)
#endif // #if BL_FEATURE_SECURE_COUNTER

#if BL_FEATURE_SECURE_FUNCTION_CALL
#if !defined(BL_FEATURE_SECURE_COUNTER) || !BL_FEATURE_SECURE_COUNTER
#warning SECURE FUNCTION CALL needs SECURE COUNTER to be enabled.
#endif
#define SFC_SECURE_ARGUMENT (0x5a3c960f)

#define SFC_ENTRY_ARG0(func, arg0)   \
    do                               \
    {                                \
        SC_ADD(func);                \
        SC_ADD(SFC_SECURE_ARGUMENT); \
    } while (0)
#define SFC_ENTRY_ARG1(func, arg1)   \
    do                               \
    {                                \
        SC_ADD(func);                \
        SC_ADD(SFC_SECURE_ARGUMENT); \
        SC_ADD(arg1);                \
    } while (0)
#define SFC_ENTRY_ARG2(func, arg1, arg2) \
    do                                   \
    {                                    \
        SC_ADD(func);                    \
        SC_ADD(SFC_SECURE_ARGUMENT);     \
        SC_ADD(arg1);                    \
        SC_ADD(arg2);                    \
    } while (0)
#define SFC_ENTRY_ARG3(func, arg1, arg2, arg3) \
    do                                         \
    {                                          \
        SC_ADD(func);                          \
        SC_ADD(SFC_SECURE_ARGUMENT);           \
        SC_ADD(arg1);                          \
        SC_ADD(arg2);                          \
        SC_ADD(arg3);                          \
    } while (0)
#define SFC_ENTRY_ARG4(func, arg1, arg2, arg3, arg4) \
    do                                               \
    {                                                \
        SC_ADD(func);                                \
        SC_ADD(SFC_SECURE_ARGUMENT);                 \
        SC_ADD(arg1);                                \
        SC_ADD(arg2);                                \
        SC_ADD(arg3);                                \
        SC_ADD(arg4);                                \
    } while (0)
#define SFC_ENTRY_ARG5(func, arg1, arg2, arg3, arg4, arg5) \
    do                                                     \
    {                                                      \
        SC_ADD(func);                                      \
        SC_ADD(SFC_SECURE_ARGUMENT);                       \
        SC_ADD(arg1);                                      \
        SC_ADD(arg2);                                      \
        SC_ADD(arg3);                                      \
        SC_ADD(arg4);                                      \
        SC_ADD(arg5);                                      \
    } while (0)

#define SFC_ENTRY_CHECK_ARG0(func, arg0)       \
    do                                         \
    {                                          \
        SC_ASSERT(func + SFC_SECURE_ARGUMENT); \
    } while (0)
#define SFC_ENTRY_CHECK_ARG1(func, arg1)       \
    do                                         \
    {                                          \
        SC_SUB(arg1);                          \
        SC_ASSERT(func + SFC_SECURE_ARGUMENT); \
    } while (0)
#define SFC_ENTRY_CHECK_ARG2(func, arg1, arg2) \
    do                                         \
    {                                          \
        SC_SUB(arg1);                          \
        SC_SUB(arg2);                          \
        SC_ASSERT(func + SFC_SECURE_ARGUMENT); \
    } while (0)
#define SFC_ENTRY_CHECK_ARG3(func, arg1, arg2, arg3) \
    do                                               \
    {                                                \
        SC_SUB(arg1);                                \
        SC_SUB(arg2);                                \
        SC_SUB(arg3);                                \
        SC_ASSERT(func + SFC_SECURE_ARGUMENT);       \
    } while (0)
#define SFC_ENTRY_CHECK_ARG4(func, arg1, arg2, arg3, arg4) \
    do                                                     \
    {                                                      \
        SC_SUB(arg1);                                      \
        SC_SUB(arg2);                                      \
        SC_SUB(arg3);                                      \
        SC_SUB(arg4);                                      \
        SC_ASSERT(func + SFC_SECURE_ARGUMENT);             \
    } while (0)
#define SFC_ENTRY_CHECK_ARG5(func, arg1, arg2, arg3, arg4, arg5) \
    do                                                           \
    {                                                            \
        SC_SUB(arg1);                                            \
        SC_SUB(arg2);                                            \
        SC_SUB(arg3);                                            \
        SC_SUB(arg4);                                            \
        SC_SUB(arg5);                                            \
        SC_ASSERT(func + SFC_SECURE_ARGUMENT);                   \
    } while (0)

#define SFC_EXIT(func, result)          \
    do                                  \
    {                                   \
        SC_SUB(func);                   \
        SC_ASSERT(SFC_SECURE_ARGUMENT); \
        SC_ADD(result);                 \
    } while (0)
#define SFC_EXIT_NORETURN(func)         \
    do                                  \
    {                                   \
        SC_SUB(func);                   \
        SC_ASSERT(SFC_SECURE_ARGUMENT); \
    } while (0)

#define SFC_EXIT_CHECK(func, result)    \
    do                                  \
    {                                   \
        SC_SUB(result);                 \
        SC_ASSERT(SFC_SECURE_ARGUMENT); \
    } while (0)
#define SFC_EXIT_CHECK_NORETURN(func)   \
    do                                  \
    {                                   \
        SC_ASSERT(SFC_SECURE_ARGUMENT); \
    } while (0)

#define SECURE_FUNCTION_CALL(func, result, argnum, ...)                                \
    do                                                                                 \
    {                                                                                  \
        volatile secure_counter_t s_secure_counter_##func##_backup = g_secure_counter; \
        SFC_ENTRY_ARG##argnum(func, __VA_ARGS__);                                      \
        result = func(__VA_ARGS__);                                                    \
        SFC_EXIT_CHECK(func, result);                                                  \
        g_secure_counter = s_secure_counter_##func##_backup;                           \
    } while (0)
#define SECURE_FUNCTION_CALL_NORETURN(func, argnum, ...)                               \
    do                                                                                 \
    {                                                                                  \
        volatile secure_counter_t s_secure_counter_##func##_backup = g_secure_counter; \
        SFC_ENTRY_ARG##argnum(func, __VA_ARGS__);                                      \
        func(__VA_ARGS__);                                                             \
        SFC_EXIT_CHECK_NORETURN(func);                                                 \
        g_secure_counter = s_secure_counter_##func##_backup;                           \
    } while (0)

#define SECURE_FUNCTION_ENTRY_CHECK(func, argnum, ...)  \
    do                                                  \
    {                                                   \
        SFC_ENTRY_CHECK_ARG##argnum(func, __VA_ARGS__); \
    } while (0)

#define SECURE_FUNCTION_EXIT(func, result) \
    do                                     \
    {                                      \
        SFC_EXIT(func, result)             \
    } while (0)
#define SECURE_FUNCTION_EXIT_NORETURN(func) \
    do                                      \
    {                                       \
        SFC_EXIT_NORETURN(func)             \
    } while (0)
#else
#define SFC_SECURE_ARGUMENT
#define SFC_ENTRY_ARG0(func, arg0)
#define SFC_ENTRY_ARG1(func, arg1)
#define SFC_ENTRY_ARG2(func, arg1, arg2)
#define SFC_ENTRY_ARG3(func, arg1, arg2, arg3)
#define SFC_ENTRY_ARG4(func, arg1, arg2, arg3, arg4)
#define SFC_ENTRY_ARG5(func, arg1, arg2, arg3, arg4, arg5)
#define SFC_ENTRY_CHECK_ARG0(func, arg0)
#define SFC_ENTRY_CHECK_ARG1(func, arg1)
#define SFC_ENTRY_CHECK_ARG2(func, arg1, arg2)
#define SFC_ENTRY_CHECK_ARG3(func, arg1, arg2, arg3)
#define SFC_ENTRY_CHECK_ARG4(func, arg1, arg2, arg3, arg4)
#define SFC_ENTRY_CHECK_ARG5(func, arg1, arg2, arg3, arg4, arg5)
#define SFC_EXIT(func, result)
#define SFC_EXIT_NORETURN(func)
#define SFC_EXIT_CHECK(func, result)
#define SFC_EXIT_CHECK_NORETURN(func)
#define SECURE_FUNCTION_CALL(func, result, argnum, ...)
#define SECURE_FUNCTION_CALL_NORETURN(func, argnum, ...)
#define SECURE_FUNCTION_ENTRY_CHECK(func, argnum, ...)
#define SECURE_FUNCTION_EXIT(func, result)
#define SECURE_FUNCTION_EXIT_NORETURN(func)
#endif // #if BL_FEATURE_ARGUMENT_CHECK

#define SC_VALUE_SYSTEM_INIT (SC_SECURE_INIT_VAL)
#ifndef BL_TARGET_RAM
#define SC_VALUE_SYSTEM_INIT_INNER_STEPS (8)
#else
#define SC_VALUE_SYSTEM_INIT_INNER_STEPS (0)
#endif

#define SC_VALUE_INIT_DATA_BSS_COMMON (0xb0fd)
#define SC_VALUE_MAIN (0x1f57)
#define SC_VALUE_BOOTLOADER_INIT (0x5a8c)
#define SC_VALUE_INIT_HARDWARE (0x7546)
#if BL_FEATURE_ROMPATCH_MODULE
#define SC_VALUE_INIT_HARDWARE_INNER_STEPS_1 (1)
#else
#define SC_VALUE_INIT_HARDWARE_INNER_STEPS_1 (0)
#endif
#if BL_FEATURE_SECURE_BOOT
#define SC_VALUE_INIT_HARDWARE_INNER_STEPS_2 (3)
#else
#define SC_VALUE_INIT_HARDWARE_INNER_STEPS_2 (0)
#endif
#define SC_VALUE_INIT_HARDWARE_INNER_STEPS \
    (8 + SC_VALUE_INIT_HARDWARE_INNER_STEPS_1 + SC_VALUE_INIT_HARDWARE_INNER_STEPS_2)

#define SC_VALUE_MEMORY_INIT (0xac8f)
#define SC_VALUE_PROPERTY_INIT (0xe9fc)

#define SC_VALUE_BOOTLOADER_RUN (0xacc1)

#define SC_VALUE_GO_PASSIVE_BOOT (0x4a9c)
#define SC_VALUE_GO_PASSIVE_BOOT_INNER_STEPS (1)

#define SC_VALUE_ISP_PIN_ASSERTED (0x8d8c)
#define SC_VALUE_ISP_PIN_NOT_ASSERTED (0xace9)

#define SC_VALUE_DEFAULT_VECTOR_TABLE_SECURE_DUMMY (0x167e)

#define SC_VALUE_GO_ISP_BOOT (0x1a71)
#define SC_VALUE_GO_ISP_BOOT_INNER_STEPS (4)

#define SC_VALUE_GET_ACTIVE_PERIPHERAL (0x58bd)

#define SC_VALUE_GO_MASTER_BOOT (0x49fc)

#define SC_VALUE_MASTERBOOT_INIT (0x33dc)
#define SC_VALUE_MASTERBOOT_INIT_INNER_STEPS (5)

#define SC_VALUE_MASTERBOOT_RUN (0x5e87)

#define SC_VALUE_MASTERBOOT_FROM_INTERFACE (0x14af)
#define SC_VALUE_MASTERBOOT_FROM_INTERFACE_INNER_STEPS (20)

#define SC_VALUE_MASTERBOOT_DOWNLOAD_INITIAL_IMAGE (0xac57)
#define SC_VALUE_MASTERBOOT_DOWNLOAD_INITIAL_IMAGE_INNER_STEPS (2)

#define SC_VALUE_RECOVERYBOOT_RUN (SC_VALUE_MASTERBOOT_RUN)

#define SC_VALUE_GO_FAIL_THROUGH_MODE (0x973c)

#define SC_ASSERT_VALUE_SYSTEM_INIT (SC_VALUE_SYSTEM_INIT)
#define SC_ASSERT_VALUE_INIT_DATA_BSS_COMMON (SC_ASSERT_VALUE_SYSTEM_INIT + SC_VALUE_SYSTEM_INIT_INNER_STEPS)
#define SC_ASSERT_VALUE_MAIN (SC_ASSERT_VALUE_INIT_DATA_BSS_COMMON + SC_VALUE_INIT_DATA_BSS_COMMON)

#define SC_ASSERT_VALUE_BOOTLOADER_INIT (SC_ASSERT_VALUE_MAIN + SC_VALUE_MAIN)
#define SC_ASSERT_VALUE_INIT_HARDWARE                                                      \
    (SC_ASSERT_VALUE_BOOTLOADER_INIT + SC_VALUE_BOOTLOADER_INIT + SC_VALUE_INIT_HARDWARE + \
     SC_VALUE_INIT_HARDWARE_INNER_STEPS)
#define SC_ASSERT_VALUE_MEMORY_INIT (SC_ASSERT_VALUE_INIT_HARDWARE + SC_VALUE_MEMORY_INIT)
#define SC_ASSERT_VALUE_PROPERTY_INIT (SC_ASSERT_VALUE_MEMORY_INIT + SC_VALUE_PROPERTY_INIT)
#define SC_ASSERT_VALUE_BOOTLOADER_RUN (SC_ASSERT_VALUE_PROPERTY_INIT)

#define SC_ASSERT_VALUE_GO_PASSIVE_BOOT (SC_ASSERT_VALUE_BOOTLOADER_RUN + SC_VALUE_BOOTLOADER_RUN)

#if BL_FEATURE_MASTER_BOOT
#define SC_ASSERT_VALUE_GO_ISP_BOOT (SC_ASSERT_VALUE_BOOTLOADER_RUN + SC_VALUE_BOOTLOADER_RUN)
#else
#define SC_ASSERT_VALUE_GO_ISP_BOOT \
    (SC_ASSERT_VALUE_GO_PASSIVE_BOOT + SC_VALUE_GO_PASSIVE_BOOT + SC_VALUE_GO_PASSIVE_BOOT_INNER_STEPS)
#endif
#define SC_ASSERT_VALUE_GET_ACTIVE_PERIPHERAL                                                \
    (SC_ASSERT_VALUE_GO_ISP_BOOT + SC_VALUE_GO_ISP_BOOT + SC_VALUE_GO_ISP_BOOT_INNER_STEPS + \
     SC_VALUE_GET_ACTIVE_PERIPHERAL)

#define SC_ASSERT_VALUE_GO_MASTER_BOOT (SC_ASSERT_VALUE_BOOTLOADER_RUN + SC_VALUE_BOOTLOADER_RUN)
#define SC_ASSERT_VALUE_MASTERBOOT_INIT (SC_ASSERT_VALUE_GO_MASTER_BOOT + SC_VALUE_GO_MASTER_BOOT)
#define SC_ASSERT_VALUE_MASTERBOOT_RUN \
    (SC_ASSERT_VALUE_MASTERBOOT_INIT + SC_VALUE_MASTERBOOT_INIT + SC_VALUE_MASTERBOOT_INIT_INNER_STEPS)
#define SC_ASSERT_VALUE_MASTERBOOT_FROM_INTERFACE (SC_ASSERT_VALUE_MASTERBOOT_RUN + SC_VALUE_MASTERBOOT_RUN)
#define SC_ASSERT_VALUE_MASTERBOOT_DOWNLOAD_INITIAL_IMAGE \
    (SC_ASSERT_VALUE_MASTERBOOT_FROM_INTERFACE + SC_VALUE_MASTERBOOT_FROM_INTERFACE)
#define SC_ASSERT_VALUE_JUMP_TO_BOOT_IMAGE                                                            \
    (SC_ASSERT_VALUE_MASTERBOOT_DOWNLOAD_INITIAL_IMAGE + SC_VALUE_MASTERBOOT_DOWNLOAD_INITIAL_IMAGE + \
     SC_VALUE_MASTERBOOT_FROM_INTERFACE_INNER_STEPS + SC_VALUE_MASTERBOOT_DOWNLOAD_INITIAL_IMAGE_INNER_STEPS)

#define SC_ASSERT_VALUE_RECOVERYBOOT_RUN (SC_ASSERT_VALUE_MASTERBOOT_RUN)

#define SC_ASSERT_VALUE_SERIALBOOTFAILREBOOT_RUN (SC_ASSERT_VALUE_MASTERBOOT_INIT)

#define SC_ASSERT_VALUE_GO_FAIL_THROUGH_MODE (SC_VALUE_GO_FAIL_THROUGH_MODE)

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

extern volatile secure_counter_t g_secure_counter;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

void secure_counter_init(void);

void secure_counter_assert_fail(void);

#if defined(__cplusplus)
}
#endif

/*!
 *@}
*/

#endif /* _BL_SECURE_COUNTER_H_ */
