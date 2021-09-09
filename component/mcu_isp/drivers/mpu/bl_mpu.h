/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _BL_MPU_H_
#define _BL_MPU_H_

#include "fsl_device_registers.h"
#include "fsl_common.h"

/*!
 * @addtogroup mpu
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//!@brief MPU region related defintions
enum
{
    kMinRegionSizeInBytes = 256,
    kMinRegionSizeInPower = 8,
    kRegionSize256Bytes = 8,
    kRegionSize512Bytes = 9,
    kRegionSize1KBytes = 10,
    kRegionSize2KBytes = 11,
    kRegionSize4KBytes = 12,
    kRegionSize8KBytes = 13,
    kRegionSize16KBytes = 14,
    kRegionSize32KBytes = 15,
    kRegionSize64KBytes = 16,
    kRegionSize128KBytes = 17,
    kRegionSize256KBytes = 18,
    kRegionSize512KBytes = 19,
    kRegionSize1MBytes = 20,
    kRegionSize2MBytes = 21,
    kRegionSize4MBytes = 22,
    kRegionSize8MBytes = 23,
    kRegionSize16MBytes = 24,
    kRegionSize32MBytes = 25,
    kRegionSize64MBytes = 26,
    kRegionSize128MBytes = 27,
    kRegionSize256MBytes = 28,
    kRegionSize512MBytes = 29,
    kRegionSize1GBytes = 30,
    kRegionSize2GBytes = 31,
    kRegionSize4GBytes = 32,
};

//!@brief Status codes for MPU operation
enum
{
    kStatus_MPU_Fail = 0x4d5055c3,            //!< MPU Initialization failure
    kStatus_MPU_InvalidArgument = 0x4d50555a, //!< Invalid Argument for MPU
    kStatus_MPU_Success = 0x4d5055f0,         //!< MPU operation succeeded
};

//!@brief Defintions for MPU attribute
typedef enum _mpu_access_attribute_type
{
    /*!< Full access. Normal memory, Outer and inner Non-cacheable, Execute Never */
    kMPU_RAMRegionAttributeWriteThroughXN = (1U << MPU_RASR_XN_Pos) | (3U << MPU_RASR_AP_Pos) | MPU_RASR_C_Msk,
    /*!< Full access. Normal memory, Outer and inner Non-cacheable */
    kMPU_RAMRegionAttributeWriteThrough = (3U << MPU_RASR_AP_Pos) | MPU_RASR_C_Msk,

    /*!< Full access. Normal memory, Non-Shareable, write-back */
    kMPU_RAMRegionAttributeWriteBack =
        (3U << MPU_RASR_AP_Pos) | MPU_RASR_C_Msk | MPU_RASR_B_Msk | (1U << MPU_RASR_TEX_Pos),

    /*!< Full access. Normal memory, Non-Shareable, write-back, Execute Never */
    kMPU_RAMRegionAttributeWriteBackXN =
        (3U << MPU_RASR_AP_Pos) | MPU_RASR_C_Msk | MPU_RASR_B_Msk | (1U << MPU_RASR_TEX_Pos) | (1U << MPU_RASR_XN_Pos),

    /*!< Full access. Normal memory, Non-Shareable, write-back no allocate, Execute Never */
    kMPU_RAMRegionAttributeWriteBackNoAllocateXN =
        (3U << MPU_RASR_AP_Pos) | MPU_RASR_C_Msk | MPU_RASR_B_Msk | (1U << MPU_RASR_XN_Pos),

    /*!< Full access. Read-only memory, Non-Shareable, write-through */
    kMPU_ROMRegionAttribute = (6U << MPU_RASR_AP_Pos) | MPU_RASR_C_Msk,

    /*!< Full access. Device memory, Non-Shareable, Execute Never*/
    kMPU_DeviceRegionAttribute = (1U << MPU_RASR_XN_Pos) | (3U << MPU_RASR_AP_Pos) | MPU_RASR_B_Msk,

    /*!<  RAM Non-cachable Executable */
    kMPU_RAMRegionAtrributeNonCacheable = (4u << MPU_RASR_TEX_Pos) | (3u << MPU_RASR_AP_Pos),

    /*!<  RAM Non-cachable Non-Executable */
    kMPU_RAMRegionAtrributeNonCacheableXN = (4u << MPU_RASR_TEX_Pos) | (3u << MPU_RASR_AP_Pos) | MPU_RASR_XN_Msk,

} mpu_access_attribute_type;

/*!
 * @brief MPU region configuration structure.
 *
 * This structure is used to configure the regionNum region.
 */
typedef struct _mpu_region_config
{
    uint32_t baseAddress; /*!< Memory region start address. Note: bit0 ~ bit4 must always be marked as 0. */
    uint32_t sizeInPower; /*!< Memory region Size. Note: Not size in bytes, it is the power of 2. */
    uint32_t attribute;   /* PMU region attribute. Note: bit0 ~ bit 15 must always be marked as 0 */
} mpu_region_config_t;

typedef struct _mpu_context
{
    mpu_region_config_t mpuMemoryMap[16];
    uint32_t validEntries;
    uint32_t crcChecksum;
} mpu_context_t;
/*******************************************************************************
 * Externs
 ******************************************************************************/
extern mpu_context_t g_mpuContext;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* _cplusplus */

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Initializes the MPU with the user configuration structure.
 *
 * This function configures the MPU module with the user-defined configuration.
 *
 */
status_t MPU_Init(void);

/*!
 * @brief Deinitializes the MPU regions.
 *
 */
status_t MPU_Deinit(void);

/* @}*/

/*!
 * @name Basic Control Operations
 * @{
 */

/*!
 * @brief Enables/disables the MPU during HardFault and NMI handlers globally.
 *
 * Call this API to enable or disable the MPU module during HardFault and NMI handlers.
 *
 * @param base     MPU peripheral base address.

 */
static inline uint32_t MPU_Region_Count(MPU_Type *base)
{
    return (base->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
}

/*!
 * @brief Enables/disables privileged software access to the default memory map.
 *
 * Call this API to enable or disable privileged software access to the default memory map.
 *
 * @param base     MPU peripheral base address.
 * @param enable   True enable access, false disable access.
 */
static inline void MPU_PRIVDEF_Enable(MPU_Type *base, bool enable)
{
    if (enable)
    {
        /* Enable the MPU globally. */
        base->CTRL |= MPU_CTRL_PRIVDEFENA_Msk;
    }
    else
    { /* Disable the MPU globally. */
        base->CTRL &= ~MPU_CTRL_PRIVDEFENA_Msk;
    }
}

/*!
 * @brief Enables/disables the MPU during HardFault and NMI handlers globally.
 *
 * Call this API to enable or disable the MPU module during HardFault and NMI handlers.
 *
 * @param base     MPU peripheral base address.
 * @param enable   True enable MPU, false disable MPU.
 */
static inline void MPU_HFNIM_Enable(MPU_Type *base, bool enable)
{
    if (enable)
    {
        /* Enable the MPU globally. */
        base->CTRL |= MPU_CTRL_HFNMIENA_Msk;
    }
    else
    { /* Disable the MPU globally. */
        base->CTRL &= ~MPU_CTRL_HFNMIENA_Msk;
    }
}

/*!
 * @brief Enables/disables the MPU globally.
 *
 * Call this API to enable or disable the MPU module.
 *
 * @param base     MPU peripheral base address.
 * @param enable   True enable MPU, false disable MPU.
 */
static inline void MPU_Enable(MPU_Type *base, bool enable)
{
    if (enable)
    {
        /* Enable the MPU globally. */
        base->CTRL |= MPU_CTRL_ENABLE_Msk;
    }
    else
    { /* Disable the MPU globally. */
        base->CTRL &= ~MPU_CTRL_ENABLE_Msk;
    }
}

//!@brief Update Checksum for MPU
status_t MPU_UpdateChecksum(void);

//!@brief Update MPU entry according to specified region
status_t MPU_UpdateEntryFromRegion(uint32_t start, uint32_t length);
/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _BL_MPU_H_ */
