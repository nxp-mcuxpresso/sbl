/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "bootloader/bl_log.h"
#include "crc/crc32.h"
#include <string.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
typedef struct _log_context
{
    uint32_t entryIndex;
    uint32_t logEntries[BL_LOG_MAX_ENTRY_NUM];
    uint32_t checkSum;
} log_context_t;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
static log_context_t s_logCtx BL_SECTION(".noinit");

// static void bl_log_checksum_refresh(void);
// static bool bl_log_context_valid(void);

////////////////////////////////////////////////////////////////////////////////
// Codes
////////////////////////////////////////////////////////////////////////////////

// static void bl_log_checksum_refresh(void)
//{
//    crc32_data_t crcConfig;
//    crc32_init(&crcConfig);
//    crc32_update(&crcConfig, (const uint8_t*)&s_logCtx, sizeof(s_logCtx) - sizeof(s_logCtx.checkSum));
//    crc32_finalize(&crcConfig, &s_logCtx.checkSum);
//}

void bl_log_init(void)
{
    memset(&s_logCtx, 0, sizeof(s_logCtx));
}

void bl_log_add_entry(uint32_t log_entry)
{
    // Once the buffer is full, ROM only retains the last error by copying it to the start of the log buffer), then adds
    // the new log after the last error if it presents.
    if (s_logCtx.entryIndex >= BL_LOG_MAX_ENTRY_NUM)
    {
        s_logCtx.entryIndex = 0;
        const void *lastErrorEntry = NULL;
        uint32_t entrySize = 0;

        bl_log_get_last_error(&lastErrorEntry, &entrySize);
        if (entrySize > 0)
        {
            memcpy(&s_logCtx.logEntries[0], lastErrorEntry, entrySize);
            s_logCtx.entryIndex += entrySize;
        }
    }
    s_logCtx.logEntries[s_logCtx.entryIndex] = log_entry;
    ++s_logCtx.entryIndex;
}

void bl_log_add_entry_data(uint32_t log_entry, uint32_t *log_data, uint32_t numberOfEntries)
{
    do
    {
        if (numberOfEntries > BL_LOG_MAX_ENTRY_NUM)
        {
            break;
        }
        // Once the buffer is full, ROM only retains the last error by copying it to the start of the log buffer), then
        // adds the new log after the last error if it presents.
        uint32_t totalEntries = MIN(1 + numberOfEntries, BL_LOG_MAX_ENTRY_NUM);
        if ((s_logCtx.entryIndex + totalEntries) >= BL_LOG_MAX_ENTRY_NUM)
        {
            s_logCtx.entryIndex = 0;
            const void *lastErrorEntry = NULL;
            uint32_t entrySize = 0;

            bl_log_get_last_error(&lastErrorEntry, &entrySize);
            if (entrySize > 0)
            {
                memcpy(&s_logCtx.logEntries[0], lastErrorEntry, entrySize);
                s_logCtx.entryIndex += entrySize;
            }
        }

        if ((s_logCtx.entryIndex + totalEntries) <= BL_LOG_MAX_ENTRY_NUM)
        {
            s_logCtx.logEntries[s_logCtx.entryIndex] = log_entry;
            ++s_logCtx.entryIndex;
            memcpy(&s_logCtx.logEntries[s_logCtx.entryIndex], log_data, sizeof(uint32_t) * numberOfEntries);
            s_logCtx.entryIndex += numberOfEntries;
        }

    } while (0);
}

void bl_log_tranverse(void)
{
#if defined(BL_TARGET_RTL) || defined(BL_TARGET_ZEBU)
    debug_printf("Traverse Bootloader Log:\n");
    for (uint32_t i = 0; i < s_logCtx.entryIndex; i++)
    {
        debug_printf("%x\n", s_logCtx.logEntries[i]);
    }
#endif
}

void bl_log_get_last_error(const void **buffer, uint32_t *count)
{
    if ((buffer == NULL) || (count == NULL))
    {
        return;
    }

    log_entry_t *pLastErrorLogEntry = NULL;
    uint32_t entryIndex = 0;
    while (entryIndex <= s_logCtx.entryIndex)
    {
        log_entry_t *pLogEntry = (log_entry_t *)&s_logCtx.logEntries[entryIndex];
        if ((pLogEntry->status == kLog_Status_Fail) || (pLogEntry->status == kLog_Status_Fatal))
        {
            pLastErrorLogEntry = pLogEntry;
        }
        entryIndex += (1 + pLogEntry->remainingEntries);
    }

    if (pLastErrorLogEntry == NULL)
    {
        *buffer = NULL;
        *count = 0;
    }
    else
    {
        *buffer = (uint32_t *)pLastErrorLogEntry;
        *count = (1 + pLastErrorLogEntry->remainingEntries) * sizeof(uint32_t);
    }
}
