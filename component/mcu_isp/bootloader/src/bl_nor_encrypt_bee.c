/*
 * Copyright 2017 - 2018 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "fsl_common.h"
#include "dcp/fsl_dcp.h"
#include "trng/fsl_trng.h"
#include "bootloader/bootloader.h"
#include "bootloader/bl_nor_encrypt.h"
#include "fusemap.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define KIB_ADDR(index) (FlexSPI_AMBA_BASE + 0x400 * (index + 1))
#define PRDB_ADDR(index) (FlexSPI_AMBA_BASE + 0x400 * (index + 1) + 0x80)
#define MIN_ENC_REGION_ADDR (FlexSPI_AMBA_BASE + 0x1000)

enum
{
    kAesMode_ECB = 0, // AES ECB mode is required
    kAesMode_CTR = 1, // AES CTR mode is required.
};

enum
{
    kBeeKeySel_OTPMK_SNVS_Low = 1,  //!< BEE Key from OTPMK/SNVS[127:0]
    kBeeKeySel_OTPMK_SNVS_High = 2, //!< BEE Key from OTPMK/SNVS[255:128]
    kBeeKeySel_SW_GP2 = 3,          //!< BEE Key from SW_GP2[127:0]
    kBeeKeySel_User_Key = 4,        //!< Key was provided by user
};

enum
{
    kBeeLockOption_NoLock = 0,      //!< BEE Region related bits are unlocked.
    kBeeLockOption_Region1 = 1,     //!< BEE Region1 related bits are locked.
    kBeeLockOption_Region0 = 2,     //!< BEE Region0 related bits are locked.
    kBeeLockOption_BothRegions = 3, //!< BEE regions related bits are locked.
};

enum
{
    kFacRegionLockOption_NoLock = 0, //!< FAC Region related bits are unlocked.
    kFacRegionLockOption_Lock = 1,   //!< FAC region related bits are locked.
};

typedef struct
{
    uint32_t start;       //!< Start address of the encrypted region, align at 1KB boundary
    uint32_t end;         //!< End address of the encrypted region, align at 1KB boundary
    uint32_t mode;        //!< AES mode: 0-CTR, 1-ECB
    uint32_t lock_option; //!< Lock options
    uint32_t counter[4];  //!< Counter for AES-CTR mode
    uint32_t reserved[8]; //!< Reserved for future use.
} encrypt_region_t;

//!@brief BEE Protection Region Descriptor Block related definitions
#define BEE_PROT_REGION_BLK_TAGL 0x5F474154 //"TAG_"
#define BEE_PROT_REGION_BLK_TAGH 0x52444845 //"EHDR"
#define BEE_PROT_REGION_HDR_VER 0x56010000  // Version 1.0.0

enum
{
    kFacMode_M7DebugAllowed = 0,
    kFacMode_M7DebugDisabled = 1,
    kFacMode_ExecuteOnlyDebugAllowed = 2,
    kFacMode_ExecuteOnly = 3,
};

typedef struct
{
    uint32_t start;       //!< Start address of one FAC region, align at 1KB boundary
    uint32_t end;         //!< End address of one FAC region, align at 1KB boundary
    uint32_t mode;        //!< Protected level: 0/1/2/3
    uint32_t reserved[5]; //!< Reserved for future use
} fac_region_t;

//!@brief Maximum supported BEE protection entries
#define MAX_BEE_PROT_ENTRIES 3

typedef struct
{
    uint32_t tagl;                   //!< Lower Half of tag, equal to BEE_PROT_REGION_BLK_TAGL
    uint32_t tagh;                   //!< Upper Half of tag, equal to BEE_PROT_REGION_BLK_TAGH
    uint32_t version;                //!< Version
    uint32_t fac_region_count;       //!< FAC region count, valid value: 1-4
    encrypt_region_t encrypt_region; //!< Encrypted region info
    fac_region_t fac_regions[4];     //!< FAC region info
    uint32_t reserved1[12];          //!< Reserved for future use.
} prot_region_desc_block_t;

#define PROT_REGION_ALIGN_SIZE (0x1000)
#define FAC_REGION_ALIGN_SIZE (0x400)

typedef struct
{
    uint8_t aes_key[16]; // AES KEY
    uint8_t iv[16];      // AES Initial Vector
} key_info_block_t;

typedef struct
{
    key_info_block_t kib;          // KIB info
    uint8_t unused[0x60];          // Padding data
    prot_region_desc_block_t prdb; // PRDB info
} prot_region_block_info_t;

typedef struct
{
    uint32_t lock_option : 4;      // Lock options, valid range: 0/1/2/3
    uint32_t region2_fac_mode : 4; // Region 2 Fac mode, valid range: 0/1/2/3
    uint32_t region1_fac_mode : 4; // Region 1 Fac mode, valid range: 0/1/2/3
    uint32_t region0_fac_mode : 4; // Region 0 Fac mode, valid range: 0/1/2/3
    uint32_t fac_region_count : 4; // Fac region count in selected BEE region, valid range: 1/2/3
    uint32_t aes_mode : 4;         // Fac region count in selected BEE region, valid range: 1/2/3
    uint32_t key_source : 4;       // Key source: valid range 0/1/2, 4/5/6
    uint32_t tag : 4;              // Fixed to 0x0E
} prot_region_option_t;

enum
{
    kOption_KeySel_Begion = 0,
    kOption_KeySel_OTPMK_SNVS_High = kOption_KeySel_Begion,
    kOption_KeySel_OTPMK_SNVS_Low,
    kOption_KeySel_SW_GP2,
    kOption_KeySel_End = kOption_KeySel_SW_GP2,
};

#define OPTION_BEE_REGION_KEY_SOURCE(idx, keysource) (((idx) << 2) | (keysource))
#define OPTION_BEE_REGION_KEY_SOURCE_MASK (0x3)

typedef struct
{
    uint32_t start;
    uint32_t end;
} bee_fac_region_t;

typedef struct
{
    prot_region_option_t option;
    struct
    {
        uint32_t start;
        uint32_t length;
    } fac_region[MAX_BEE_PROT_ENTRIES];
} prot_region_arg_t;

enum
{
    kProtRegionArg_Option_Tag = 0x0E,
};

typedef struct
{
    uint32_t fuse_key_sel[2];
    aes_key_sel_t bee_key_sel[2];
    bool encrypt_enabled[2];
    prot_region_block_info_t plain_block_info[2];
    prot_region_block_info_t enc_block_info[2];
} image_gen_ctx_t;

/*******************************************************************************
 * Local variables
 ******************************************************************************/
static image_gen_ctx_t s_img_gen_ctx;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
// Swap bytes within a block
static void aes_block_swap(uint8_t array[16]);

// Try to load and parse Protection region info
static status_t bl_nor_encrypt_region_info_load_default(void);

// Check whether the option is valid or not
static bool is_valid_options(prot_region_option_t option);

/*******************************************************************************
 * Codes
 ******************************************************************************/
status_t bl_nor_encrypt_region_info_load_default(void)
{
    /*
     * NOTE: This function supports load EKIB that is encrypted by the key derived from OTPMK only.
     */
    status_t status = kStatus_Fail;

    s_img_gen_ctx.fuse_key_sel[0] = kBeeKeySel_OTPMK_SNVS_High;
    s_img_gen_ctx.fuse_key_sel[1] = kBeeKeySel_OTPMK_SNVS_High;

    do
    {
        /* Flashloader will not load the PRDB if one of below conditions is met */
        if ((!ROM_OCOTP_ENCRYPT_XIP_VALAUE()) ||                      // Encrypted XIP feature is not enabled
            (!(ROM_OCOTP_BEE_KEY0_SEL_VALUE() > 0)) ||                // BEE_KEY0_SEL is 0
            (!(ROM_OCOTP_BEE_KEY1_SEL_VALUE() > 0)) ||                // BEE_KEY1_SEL is 0
            (!(get_primary_boot_device() == kBootDevice_FlexSpiNOR))) // Boot device is not FlexSPI NOR
        {
            break;
        }

        uint32_t boot_mode = (SRC->SBMR2 & SRC_SBMR2_BMOD_MASK) >> SRC_SBMR2_BMOD_SHIFT;

        // Boot from fuse mode or internal boot mode
        if ((boot_mode == 0x0) || (boot_mode == 0x02))
        {
            // Try to reset BEE FAC region registers
            bee_fac_region_t *fac_region = (bee_fac_region_t *)&IOMUXC_GPR->GPR18;
            for (uint32_t i = 0; i < MAX_BEE_PROT_ENTRIES; i++)
            {
                fac_region[i].start = 0;
                fac_region[i].end = 0;
            }
        }

        s_img_gen_ctx.fuse_key_sel[0] = ROM_OCOTP_BEE_KEY0_SEL_VALUE();
        s_img_gen_ctx.fuse_key_sel[1] = ROM_OCOTP_BEE_KEY1_SEL_VALUE();

        dcp_alg_ctx_t dcp_ctx;
        dcp_aes_init(&dcp_ctx);

        for (uint32_t i = 0; i < 2; i++)
        {
            switch (s_img_gen_ctx.fuse_key_sel[i])
            {
                case kBeeKeySel_OTPMK_SNVS_Low:
                    s_img_gen_ctx.bee_key_sel[i].option = OTPMK_SNVS_LOW_FLAG_BE;
                    break;
                case kBeeKeySel_OTPMK_SNVS_High:
                    s_img_gen_ctx.bee_key_sel[i].option = OTPMK_SNVS_HIGH_FLAG_BE;
                    break;
                case kBeeKeySel_SW_GP2:
                    s_img_gen_ctx.bee_key_sel[i].option = SW_GP2_FLAG_BE;
                    break;
                default:
                    s_img_gen_ctx.bee_key_sel[i].key = NULL;
                    break;
            }

            if (s_img_gen_ctx.bee_key_sel[i].key == NULL)
            {
                s_img_gen_ctx.encrypt_enabled[i] = false;
                continue;
            }
            else
            {
                s_img_gen_ctx.encrypt_enabled[i] = true;
            }

            prot_region_block_info_t plain_block_info;
            prot_region_block_info_t enc_block_info;
            dcp_aes_set_key(&dcp_ctx, s_img_gen_ctx.bee_key_sel[i], 128);
            uint32_t kib_addr = KIB_ADDR(i);
            uint32_t prdb_addr = PRDB_ADDR(i);

            // Read EKIB from Flash
            memcpy(&enc_block_info.kib, (void *)kib_addr, sizeof(key_info_block_t));
            // Read EPRDB from flash
            memcpy(&enc_block_info.prdb, (void *)prdb_addr, sizeof(prot_region_desc_block_t));

            // Decrypt EKIB
            dcp_aes_ecb_crypt(&dcp_ctx, kAesMode_Decrypt, (uint8_t *)&enc_block_info.kib,
                              (uint8_t *)&plain_block_info.kib, sizeof(key_info_block_t));

            // Decrypt PRDB
            aes_key_sel_t key_sel;
            key_sel.key = plain_block_info.kib.aes_key;
            dcp_aes_set_key(&dcp_ctx, key_sel, 128);
            dcp_aes_cbc_crypt(&dcp_ctx,                          // DCP context
                              kAesMode_Decrypt,                  // Decrypt
                              plain_block_info.kib.iv,           // Intial vector
                              (uint8_t *)&enc_block_info.prdb,   // Source, encrypted data
                              (uint8_t *)&plain_block_info.prdb, // Destination, decrypted data
                              sizeof(prot_region_desc_block_t)); // Size in bytes

            if ((plain_block_info.prdb.tagh != BEE_PROT_REGION_BLK_TAGH) ||
                (plain_block_info.prdb.tagl != BEE_PROT_REGION_BLK_TAGL))
            {
                s_img_gen_ctx.encrypt_enabled[i] = false;
                memset(&s_img_gen_ctx.plain_block_info[i], 0, sizeof(prot_region_block_info_t));
                continue;
            }
            else
            {
                s_img_gen_ctx.encrypt_enabled[i] = true;
                memcpy(&s_img_gen_ctx.plain_block_info[i], &plain_block_info, sizeof(prot_region_block_info_t));
                status = kStatus_Success;
            }
        }

    } while (0);

    return status;
}

bool bl_nor_encrypt_has_encrypted_region(void)
{
    bool has_encrypted_region = false;
    status_t status = bl_nor_encrypt_region_info_load_default();
    if (status == kStatus_Success)
    {
        has_encrypted_region = true;
    }

    return has_encrypted_region;
}

bool bl_nor_encrypt_region_info_valid(void *arg)
{
    bool is_valid = false;

    do
    {
        prot_region_arg_t *region_arg = (prot_region_arg_t *)arg;
        if ((arg == NULL) || (region_arg->option.tag != kProtRegionArg_Option_Tag))
        {
            break;
        }

        is_valid = true;

    } while (0);

    return is_valid;
}

bool is_valid_options(prot_region_option_t option)
{
    bool is_valid = false;
    do
    {
        uint32_t fac_region_count = option.fac_region_count;
        if ((fac_region_count < 1) || (fac_region_count > MAX_BEE_PROT_ENTRIES))
        {
            break;
        }

        if (option.aes_mode > kAesMode_CTR)
        {
            break;
        }

        if (option.region0_fac_mode > kFacMode_ExecuteOnly)
        {
            break;
        }

        if ((fac_region_count > 1) && (option.region1_fac_mode > kFacMode_ExecuteOnly))
        {
            break;
        }

        if ((fac_region_count > 2) && (option.region2_fac_mode > kFacMode_ExecuteOnly))
        {
            break;
        }

        uint32_t keysource = option.key_source;
        if ((keysource >= OPTION_BEE_REGION_KEY_SOURCE(0, kOption_KeySel_Begion)) &&
            (keysource <= OPTION_BEE_REGION_KEY_SOURCE(0, kOption_KeySel_End)))
        {
        }
        else if ((keysource >= OPTION_BEE_REGION_KEY_SOURCE(1, kOption_KeySel_Begion)) &&
                 (keysource <= OPTION_BEE_REGION_KEY_SOURCE(1, kOption_KeySel_End)))
        {
        }
        else
        {
            break;
        }
        is_valid = true;

    } while (0);

    return is_valid;
}

//! @brief Initialize Encrypt Region based on specified argument
status_t bl_nor_encrypt_init(void *arg)
{
    status_t status = kStatus_InvalidArgument;

    trng_config_t trng_config;

    do
    {
        // Check wether the EKIB and EPRDB are present before do initialization.
        bool has_prot_region = bl_nor_encrypt_has_encrypted_region();

        if (!bl_nor_encrypt_region_info_valid(arg))
        {
            break;
        }

        prot_region_arg_t *region_arg = (prot_region_arg_t *)arg;
        if (region_arg->option.tag == kProtRegionArg_Option_Tag)
        {
            if (!is_valid_options(region_arg->option))
            {
                break;
            }

            uint32_t bee_region_index = 0;
            uint32_t keysource = region_arg->option.key_source;
            if ((keysource >= OPTION_BEE_REGION_KEY_SOURCE(0, kOption_KeySel_Begion)) &&
                (keysource <= OPTION_BEE_REGION_KEY_SOURCE(0, kOption_KeySel_End)))
            {
                bee_region_index = 0;
            }
            else
            {
                bee_region_index = 1;
            }

            bool need_new_block_info = false;

            if (has_prot_region && s_img_gen_ctx.encrypt_enabled[bee_region_index])
            {
                need_new_block_info = false;
            }
            else
            {
                need_new_block_info = true;
            }

            if (need_new_block_info)
            {
                prot_region_block_info_t *block_info = &s_img_gen_ctx.plain_block_info[bee_region_index];

                switch (keysource & OPTION_BEE_REGION_KEY_SOURCE_MASK)
                {
                    case kOption_KeySel_OTPMK_SNVS_High:
                        s_img_gen_ctx.bee_key_sel[bee_region_index].option = OTPMK_SNVS_HIGH_FLAG_BE;
                        break;
                    case kOption_KeySel_OTPMK_SNVS_Low:
                        s_img_gen_ctx.bee_key_sel[bee_region_index].option = OTPMK_SNVS_LOW_FLAG_BE;
                        break;
                    case kOption_KeySel_SW_GP2:
                        s_img_gen_ctx.bee_key_sel[bee_region_index].option = SW_GP2_FLAG_BE;
                        break;
                }
                s_img_gen_ctx.encrypt_enabled[bee_region_index] = true;

                // Generate new Prot Block Info
                memset(block_info, 0, sizeof(prot_region_block_info_t));

                block_info->prdb.tagh = BEE_PROT_REGION_BLK_TAGH;
                block_info->prdb.tagl = BEE_PROT_REGION_BLK_TAGL;

                uint32_t fac_region_count = region_arg->option.fac_region_count;

                block_info->prdb.fac_region_count = fac_region_count;
                block_info->prdb.version = BEE_PROT_REGION_HDR_VER;
                block_info->prdb.encrypt_region.lock_option = region_arg->option.lock_option;
                block_info->prdb.encrypt_region.mode = region_arg->option.aes_mode;

                for (uint32_t i = 0; i < fac_region_count; i++)
                {
                    uint32_t fac_mode;

                    switch (i)
                    {
                        default:
                        case 0:
                            fac_mode = region_arg->option.region0_fac_mode;
                            break;
                        case 1:
                            fac_mode = region_arg->option.region1_fac_mode;
                            break;
                        case 2:
                            fac_mode = region_arg->option.region2_fac_mode;
                            break;
                    }

                    block_info->prdb.fac_regions[i].start = region_arg->fac_region[i].start;
                    block_info->prdb.fac_regions[i].end =
                        region_arg->fac_region[i].start + region_arg->fac_region[i].length;
                    block_info->prdb.fac_regions[i].mode = fac_mode;
                }

                // Find out Prot Region Start and End
                uint32_t prot_region_start = block_info->prdb.fac_regions[0].start;
                uint32_t prot_region_end = block_info->prdb.fac_regions[0].end;

                if (fac_region_count > 1)
                {
                    for (uint32_t i = 1; i < fac_region_count; i++)
                    {
                        if (prot_region_start > block_info->prdb.fac_regions[i].start)
                        {
                            prot_region_start = block_info->prdb.fac_regions[i].start;
                        }
                        if (prot_region_end < block_info->prdb.fac_regions[i].end)
                        {
                            prot_region_end = block_info->prdb.fac_regions[i].end;
                        }
                    }
                }

                // Ensure PROT REGION address is valid
                if (prot_region_start < MIN_ENC_REGION_ADDR)
                {
                    break;
                }

                block_info->prdb.encrypt_region.start = prot_region_start;
                block_info->prdb.encrypt_region.end = prot_region_end;

                // Initialize TRNG module before generating nonce and KIB
                status = TRNG_GetDefaultConfig(&trng_config);
                if (status != kStatus_Success)
                {
                    break;
                }
                // Set sample mode of the TRNG ring oscillator to Von Neumann, for better random data.
                trng_config.sampleMode = kTRNG_SampleModeVonNeumann;
                status = TRNG_Init(TRNG, &trng_config);
                if (status != kStatus_Success)
                {
                    break;
                }

                if (block_info->prdb.encrypt_region.mode == kAesMode_CTR)
                {
                    // Automatically generate random counter[127:32]
                    status = TRNG_GetRandomData(TRNG, &block_info->prdb.encrypt_region.counter[0], 16);
                    block_info->prdb.encrypt_region.counter[0] = 0;
                    if (status != kStatus_Success)
                    {
                        break;
                    }
                }

                // Generate random KIB info: aes_key
                status = TRNG_GetRandomData(TRNG, &block_info->kib.aes_key, 16);
                if (status != kStatus_Success)
                {
                    break;
                }

                // Generate random KIB info: initial vector
                status = TRNG_GetRandomData(TRNG, &block_info->kib.iv, 16);
                if (status != kStatus_Success)
                {
                    break;
                }

            } // if (need_new_block_info)
        }     // if (region_arg->option.tag == kProtRegionArg_Option_Tag)

    } while (0);

    return status;
}

//! @brief Check wether a specified region is in encrypted region or not
bool bl_nor_in_encrypted_region(uint32_t start, uint32_t bytes)
{
    bool is_in_encypted_region = false;
    for (uint32_t bee_region_index = 0; bee_region_index < 2; bee_region_index++)
    {
        prot_region_block_info_t *block_info = &s_img_gen_ctx.plain_block_info[bee_region_index];
        for (uint32_t i = 0; i < block_info->prdb.fac_region_count; i++)
        {
            if ((start > (block_info->prdb.fac_regions[i].end - 1)) ||
                ((start + bytes) <= block_info->prdb.fac_regions[i].start))
            {
                continue;
            }
            else
            {
                is_in_encypted_region = true;
                break;
            }
        }

        if (is_in_encypted_region)
        {
            break;
        }
    }

    return is_in_encypted_region;
}

//! @brief Get Encrypted Configuration block with EKIB and EPRDB
status_t bl_nor_encrypt_get_config_block(uint32_t index, uint32_t *start, uint32_t *bytes)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((start == NULL) || (bytes == NULL) || (index > 1) || (!s_img_gen_ctx.encrypt_enabled[index]))
        {
            break;
        }

        dcp_alg_ctx_t dcp_ctx;
        dcp_aes_init(&dcp_ctx);
        aes_key_sel_t key_sel;

        // Encrypt PRDB
        prot_region_block_info_t *plain_block_info = &s_img_gen_ctx.plain_block_info[index];
        prot_region_block_info_t *enc_block_info = &s_img_gen_ctx.enc_block_info[index];
        key_sel.key = &plain_block_info->kib.aes_key[0];
        dcp_aes_set_key(&dcp_ctx, key_sel, 128);
        dcp_aes_cbc_crypt(&dcp_ctx, kAesMode_Encrypt, &plain_block_info->kib.iv[0], (uint8_t *)&plain_block_info->prdb,
                          (uint8_t *)&enc_block_info->prdb, sizeof(prot_region_desc_block_t));

        // Enecrypt KIB
        key_sel.option = s_img_gen_ctx.bee_key_sel[index].option;
        dcp_aes_set_key(&dcp_ctx, key_sel, 128);
        dcp_aes_ecb_crypt(&dcp_ctx, kAesMode_Encrypt, (uint8_t *)&plain_block_info->kib,
                          (uint8_t *)&enc_block_info->kib, sizeof(key_info_block_t));

        *start = (uint32_t)enc_block_info;
        *bytes = sizeof(*enc_block_info);

        status = kStatus_Success;

    } while (0);

    return status;
}

void aes_block_swap(uint8_t array[16])
{
    uint32_t i;
    uint8_t tmp;

    for (i = 0; i < 8; i++)
    {
        tmp = array[i];
        array[i] = array[15 - i];
        array[15 - i] = tmp;
    }
}

//! @brief Encrypted data in specified region
status_t bl_nor_encrypt_data(uint32_t addr, uint32_t size, uint32_t *data_start)
{
    status_t status = kStatus_InvalidArgument;
    dcp_alg_ctx_t dcp_ctx;
    aes_key_sel_t key_sel;

    do
    {
        if ((size == 0) || (data_start == NULL))
        {
            break;
        }

        uint32_t bee_region_index = 0;

        while (bee_region_index < 2)
        {
            prot_region_block_info_t *plain_block_info = &s_img_gen_ctx.plain_block_info[bee_region_index];

            if ((addr > (plain_block_info->prdb.encrypt_region.end - 1)) ||
                ((addr + size) <= plain_block_info->prdb.encrypt_region.start))
            {
                bee_region_index++;
            }
            else
            {
                break;
            }
        }
        if (bee_region_index > 1)
        {
            break;
        }
        key_sel.option = s_img_gen_ctx.bee_key_sel[bee_region_index].option;

        dcp_aes_init(&dcp_ctx);

        dcp_aes_set_key(&dcp_ctx, key_sel, 128);

        prot_region_block_info_t *plain_block_info = &s_img_gen_ctx.plain_block_info[bee_region_index];
        if (plain_block_info->prdb.encrypt_region.mode == kAesMode_CTR)
        {
            uint32_t counter[4];
            memcpy(&counter, plain_block_info->prdb.encrypt_region.counter, 16);
            counter[0] = addr >> 4;
            // Convert to 128bit big-endian mode before doing AES-CTR encryption
            aes_block_swap((uint8_t *)&counter);
            // Do In-place encryption
            dcp_aes_ctr_crypt(&dcp_ctx, (uint8_t *)&counter, (uint8_t *)data_start, (uint8_t *)data_start, size);
        }
        else
        {
            // Do In-place encryption
            dcp_aes_ecb_crypt(&dcp_ctx, kAesMode_Encrypt, (uint8_t *)data_start, (uint8_t *)data_start, size);
        }

        status = kStatus_Success;

    } while (0);

    return status;
}

void bl_nor_encrypt_region_refresh(uint32_t start, uint32_t bytes)
{
    for (uint32_t i = 0; i < 2; i++)
    {
        prot_region_block_info_t *plain_block_info = &s_img_gen_ctx.plain_block_info[i];
        prot_region_block_info_t *enc_block_info = &s_img_gen_ctx.enc_block_info[i];
        if ((start <= KIB_ADDR(i)) && ((start + bytes) > KIB_ADDR(i)))
        {
            // Clear PRDB because it has been erased
            memset(plain_block_info, 0, sizeof(*plain_block_info));
            memset(enc_block_info, 0, sizeof(*enc_block_info));
        }
    }
}
