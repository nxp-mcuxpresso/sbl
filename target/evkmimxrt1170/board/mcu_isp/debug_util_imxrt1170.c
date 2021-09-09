/*
 * Copyright 2018-2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*====================================================================================================
                                        INCLUDE FILES
==================================================================================================*/
#include <stdlib.h>
#include "bootloader/bl_cache.h"
#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "fusemap.h"

/*==================================================================================================
                                     MACROs
==================================================================================================*/
#define BOOTROM_PASS 0xaa
#define BOOTROM_FAIL 0xff

#define BOOTROM_ARG_ADDR 0x40c37dfc
#define BOOTROM_CMD_ADDR 0x40c37df8

#define BOOTROM_PRINTF32 0x91
#define BOOTROM_INFO 0x8B

#define BOOTROM_ASSIGN(address, data) *((volatile uint32_t *)(address)) = ((uint32_t)data)

/*==================================================================================================
                                     Prototypes
==================================================================================================*/
static void convert_digit_to_string(uint32_t digit, char *str, uint32_t *length);
static void convert_hexdigit_to_string(uint32_t digit, char *str, uint32_t *length);
static bool is_cm4_core(void);
extern status_t USDHC_MemorySpaceConvert(uint32_t *realAddress, uint32_t origAddress, uint32_t size);

/*==================================================================================================
                                     LOCAL FUNCTIONS
==================================================================================================*/
static bool is_cm4_core(void)
{
    enum
    {
        kPartNumber_CM4 = 0xC24,
        kPartNumber_CM7 = 0xC27,
    };

    uint32_t partNumber = (SCB->CPUID & SCB_CPUID_PARTNO_Msk) >> SCB_CPUID_PARTNO_Pos;
    if (partNumber == kPartNumber_CM4)
    {
        return true;
    }
    else
    {
        return false;
    }
}
#if defined(__ICCARM__)
#pragma no_stack_protect
#endif
static void convert_digit_to_string(uint32_t digit, char *str, uint32_t *length)
{
    char buffer[11] = { 0 };
    uint32_t index = 0;
    if (digit < 1)
    {
        index = 1;
        buffer[0] = '0';
    }
    else
    {
        while (digit > 0)
        {
            buffer[index++] = (digit % 10) + '0';
            digit /= 10;
        }
    }

    char *digitStr = &buffer[index - 1];
    for (uint32_t i = 0; i < index; i++)
    {
        *str++ = *digitStr--;
    }

    *length = index;
}

#if defined(__ICCARM__)
#pragma no_stack_protect
#endif
static void convert_hexdigit_to_string(uint32_t digit, char *str, uint32_t *length)
{
    char buffer[8];
    uint32_t index = 0;

    for (uint32_t i = 0; i < sizeof(buffer) / sizeof(buffer[0]); i++)
    {
        buffer[i] = '0';
    }

    while (digit > 0)
    {
        uint32_t temp = digit & 0x0F;
        digit /= 16;
        if (temp < 10)
        {
            temp += '0';
        }
        else
        {
            temp = (temp - 10) + 'a';
        }
        buffer[index++] = temp;
    }

    index = 8;

    *str++ = '0';
    *str++ = 'x';
    char *digitStr = &buffer[index - 1];
    for (uint32_t i = 0; i < index; i++)
    {
        *str++ = *digitStr--;
    }

    *length = index + 2;
}

#ifdef BL_TARGET_RTL
#define STRING_BUFFER_SIZE (0x1000)
BL_ALIGN(8) static char s_rtlStringBuffer[STRING_BUFFER_SIZE] BL_SECTION(".test_bss");
#if defined(__ICCARM__)
#pragma no_stack_protect
#endif
void debug_printf(const char *format, ...)
{
    static bool has_inited = false;
    if (!has_inited)
    {
        if (FUSE_MECC_ENABLE_VALUE)
        {
            extern void ram_region_clear(uint32_t start, uint32_t lengthInBytes);
            ram_region_clear((uint32_t)&s_rtlStringBuffer, sizeof(s_rtlStringBuffer));
        }
        has_inited = true;
    }

    char *stringBuffer = s_rtlStringBuffer;

    va_list arg;
    va_start(arg, format);

    const char *fmt = format;
    uint32_t digitLength = 0;
    char *printStr = stringBuffer;
    while (*fmt != '\0')
    {
        if (*fmt == '%')
        {
            if ((fmt[1] == 'd') || (fmt[1] == 'u') || (fmt[1] == 'l'))
            {
                uint32_t parameter = va_arg(arg, int);
                convert_digit_to_string(parameter, printStr, &digitLength);
                printStr += digitLength;
                fmt++;
            }
            else if (fmt[1] == 'x')
            {
                uint32_t parameter = va_arg(arg, int);
                convert_hexdigit_to_string(parameter, printStr, &digitLength);
                printStr += digitLength;
                fmt++;
            }
            else if (fmt[1] == 'c')
            {
                char parameter = (char)va_arg(arg, int);
                *printStr++ = parameter;
                fmt++;
            }
            else if (fmt[1] == 's')
            {
                char *str = va_arg(arg, char *);
                while (*str != '\0')
                {
                    *printStr++ = *str++;
                }
                fmt++;
            }
            else
            {
                *printStr++ = *fmt;
            }
        }
        else
        {
            *printStr++ = *fmt;
        }
        fmt++;
    }
    *printStr = '\0';

    printStr = stringBuffer;

    uint32_t arg_addr = BOOTROM_ARG_ADDR;
    if (is_cm4_core())
    {
        arg_addr += 0x100;
    }
    uint32_t cmd_addr = BOOTROM_CMD_ADDR;
    if (is_cm4_core())
    {
        cmd_addr += 0x100;
    }

    if (is_cm4_core())
    {
        USDHC_MemorySpaceConvert((uint32_t *)&printStr, (uint32_t)printStr, 1); // Convert M4 address to M7 address
    }

    bl_dcache_clean();

    BOOTROM_ASSIGN(arg_addr, printStr);
    BOOTROM_ASSIGN(cmd_addr, BOOTROM_INFO);

    __DSB();
    __ISB();
}

#if defined(__ICCARM__)
#pragma no_stack_protect
#endif
void trigger_pass(void)
{
    uint32_t cmd_addr = BOOTROM_CMD_ADDR;
    if (is_cm4_core())
    {
        cmd_addr += 0x100;
    }
    BOOTROM_ASSIGN(cmd_addr, BOOTROM_PASS);
}

#if defined(__ICCARM__)
#pragma no_stack_protect
#endif
void trigger_fail(void)
{
    uint32_t cmd_addr = BOOTROM_CMD_ADDR;
    if (is_cm4_core())
    {
        cmd_addr += 0x100;
    }
    BOOTROM_ASSIGN(cmd_addr, BOOTROM_FAIL);
}
#endif // BL_TARGET_RTL

#if defined(BL_TARGET_ZEBU) || defined(BL_TARGET_FPGA)
#include "lpuart/fsl_lpuart.h"
#pragma no_stack_protect
void debug_printf(const char *format, ...)
{
    char *stringBuffer = (char *)0x20000000;

    const char *fmt = format;
    uint32_t digitLength = 0;
    char *printStr = stringBuffer;
    va_list arg;
    va_start(arg, format);
    while (*fmt != '\0')
    {
        if (*fmt == '%')
        {
            if ((fmt[1] == 'd') || (fmt[1] == 'u') || (fmt[1] == 'l'))
            {
                uint32_t parameter = va_arg(arg, int);
                convert_digit_to_string(parameter, printStr, &digitLength);
                printStr += digitLength;
                fmt++;
            }
            else if (fmt[1] == 'x')
            {
                uint32_t parameter = va_arg(arg, int);
                convert_hexdigit_to_string(parameter, printStr, &digitLength);
                printStr += digitLength;
                fmt++;
            }
            else if (fmt[1] == 'c')
            {
                char parameter = (char)va_arg(arg, int);
                *printStr++ = parameter;
                fmt++;
            }
            else if (fmt[1] == 's')
            {
                char *str = va_arg(arg, char *);
                while (*str != '\0')
                {
                    *printStr++ = *str++;
                }
                fmt++;
            }
            else
            {
                *printStr++ = *fmt;
            }
        }
        else
        {
            *printStr++ = *fmt;
        }
        fmt++;
    }
    *printStr = '\0';

    LPUART_WriteBlocking(LPUART2, (const uint8_t *)stringBuffer, printStr - stringBuffer);
}

#endif
