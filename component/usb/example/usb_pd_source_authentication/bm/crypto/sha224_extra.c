// ############################################################################
// #             Copyright (C), NXP Semiconductors                            #
// #                       (C), NXP B.V. of Eindhoven                         #
// #                                                                          #
// # All rights are reserved. Reproduction in whole or in part is prohibited  #
// # without the written consent of the copyright owner.                      #
// # NXP reserves the right to make changes without notice at any time.       #
// # NXP makes no warranty, expressed, implied or statutory, including but    #
// # not limited to any implied warranty of merchantability or fitness for    #
// # any particular purpose, or that the use will not infringe any third      #
// # party patent, copyright or trademark. NXP must not be liable for any     #
// # loss or damage arising from its use.                                     #
// ############################################################################

#include <stdint.h>
#include "sha224.h"
#include "sha224_extra.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static const uint8_t salt_zero[SHA224_HASH_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/*******************************************************************************
 * Code
 ******************************************************************************/

void sha224_hmac(sha224_t *context,
                 const uint8_t *key,
                 const uint32_t keysize,
                 const uint8_t *message,
                 const uint32_t messagesize,
                 uint8_t *mac)
{
    uint8_t mkey[SHA224_BLOCK_SIZE];
    uint8_t *d;
    uint8_t const *D;
    uint8_t const *p = 0;
    d = mkey;
    if (keysize > SHA224_BLOCK_SIZE)
    {
        sha224_init(context);
        sha224_update(context, key, keysize);
        sha224_final(context, d);
        d += SHA224_HASH_SIZE;
    }
    else
    {
        for (p = key, D = (key + keysize); p != D; d++, p++)
            *d = *p;
    }
    for (D = (mkey + SHA224_BLOCK_SIZE); d != D; d++)
        *d = 0x00;
    for (d = mkey; d != D; d++, p++)
        *d ^= 0x36;
    sha224_init(context);
    sha224_update(context, mkey, SHA224_BLOCK_SIZE);
    sha224_update(context, message, messagesize);
    sha224_final(context, mac);
    for (d = mkey; d != D; d++)
        *d ^= 0x6A;
    sha224_init(context);
    sha224_update(context, mkey, SHA224_BLOCK_SIZE);
    sha224_update(context, mac, SHA224_HASH_SIZE);
    sha224_final(context, mac);
}

void sha224_hkdf_extract(sha224_hkdf_t *context, const uint8_t *entropy, const uint32_t entropysize)
{
    context->count = 0;
    sha224_hmac(context->sha224_context, salt_zero, SHA224_HASH_SIZE, entropy, entropysize, context->prk);
}

void sha224_hkdf_expand(sha224_hkdf_t *context, const uint8_t *info, const uint32_t infosize)
{
    uint8_t *d;
    uint8_t const *D, *p;
    (context->count)++;
    d = (context->state + SHA224_HASH_SIZE);
    if (infosize > MAX_SHA224_HKDF_INFO_SIZE)
        D = (d + MAX_SHA224_HKDF_INFO_SIZE);
    else
        D = (d + infosize);
    for (p = info; d != D; d++, p++)
        *d = *p;
    *(d++) = (uint8_t)(((context->count) >> 24) & 0xFF);
    *(d++) = (uint8_t)(((context->count) >> 16) & 0xFF);
    *(d++) = (uint8_t)(((context->count) >> 8) & 0xFF);
    *(d++) = (uint8_t)((context->count) & 0xFF);
    if ((context->count) == 1)
        sha224_hmac(context->sha224_context, context->prk, SHA224_HASH_SIZE, (context->state + SHA224_HASH_SIZE),
                    (uint32_t)(d - context->state - SHA224_HASH_SIZE), context->state);
    else
        sha224_hmac(context->sha224_context, context->prk, SHA224_HASH_SIZE, context->state,
                    (uint32_t)(d - context->state), context->state);
}
