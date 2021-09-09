/*
 * Copyright 2019-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "keystore_puf.h"
#include "memory/memory.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Define the key slot of the.specific hardware engine. */
enum
{
    kPUF_KeySlot_SNVS = kPUF_KeySlot0,
    kPUF_KeySlot_OTFAD = kPUF_KeySlot1,
};

/*! @brief Define the key range(in bytes) of the specific hardware engine. */
enum
{
    kKeySize_MAX_OTFAD = 16,
    kKeySize_MIN_OTFAD = 16,
};

/*! @brief Define the offset of the key store in the non-volatile memory. */
enum
{
    kKeyStore_Offset_FlexSpiNor = kFlexSpi_Key_Store_Offset,
};

/*******************************************************************************
 * Prototype
 ******************************************************************************/
extern status_t flexspi_get_amba_address(uint32_t offset, uint32_t *ambaAddr);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Codes
 ******************************************************************************/
/*! @brief Get the key store base address in the external memory. See keystore_puf.h for details. */
/*
 * Note: For XIP external memory, this function should return the XIP address, not the offset of the external memory.
 */
status_t get_key_store_nonvolatile_address(uint32_t memoryId, uint32_t *offset)
{
    assert(offset);

    status_t status = kStatus_Success;

    switch (memoryId)
    {
#if BL_FEATURE_FLEXSPI_NOR_MODULE
        case kMemoryFlexSpiNor:
            status = flexspi_get_amba_address(kKeyStore_Offset_FlexSpiNor, offset);
            break;
#endif
        default:
            status = kStatus_InvalidArgument;
    }
    return status;
}

/*! @brief Get the key slot for the specific hardware key. See keystore_puf.h for details. */
status_t key_store_get_hw_key_slot(uint32_t *slot, uint32_t type)
{
    assert(slot);

    status_t status = kStatus_Success;

    do
    {
        switch (type)
        {
            case kKeyType_OtfadKEK:
                *slot = kPUF_KeySlot_OTFAD;
                break;
            default:
                status = kStatus_InvalidArgument;
                break;
        }
    } while (0);

    return status;
}

/*! @brief Get the key size rang of the specifi key type. See keystore_puf.h for details. */
status_t key_store_get_hw_key_range(uint32_t type, uint32_t *maxKeySize, uint32_t *minKeySize)
{
    assert(maxKeySize);
    assert(minKeySize);

    status_t status = kStatus_Success;

    do
    {
        switch (type)
        {
            case kKeyType_OtfadKEK:
                *maxKeySize = kKeySize_MAX_OTFAD;
                *minKeySize = kKeySize_MIN_OTFAD;
                break;
            default:
                *maxKeySize = 0;
                *minKeySize = 0;
                status = kStatus_InvalidArgument;
                break;
        }
    } while (0);

    return status;
}

static inline void byte_switch(uint8_t *a, uint8_t *b)
{
    *a = *a ^ *b;
    *b = *a ^ *b;
    *a = *a ^ *b;
}

/*! @brief Change the key sequence to match the device-specific hardware. See keystore_puf.h for details. */
void key_store_switch_sequence(uint8_t *key, uint32_t size, uint32_t type)
{
    assert(key);
    assert(size);
    assert(!(((uint32_t)key) % 4));
    assert(!(size % 4));

    int i = 0;
    switch (type)
    {
        case kKeyType_OtfadKEK:
            /* OTFAD */
            /* Switch endian of the words. */
            while (i < (size - i))
            {
                byte_switch(&key[i], &key[size - i - 4]);
                byte_switch(&key[i + 1], &key[size - i - 3]);
                byte_switch(&key[i + 2], &key[size - i - 2]);
                byte_switch(&key[i + 3], &key[size - i - 1]);
                i += 4;
            }
            break;
        default:
            /* Do not change the key. */
            break;
    }
}
