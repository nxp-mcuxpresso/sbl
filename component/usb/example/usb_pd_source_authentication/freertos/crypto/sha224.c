/*
 * Updated to C++, zedwood.com 2012
 * Based on Olivier Gay's version
 * See Modified BSD License below:
 *
 * FIPS 180-2 SHA-224/256/384/512 implementation
 * Issue date:  04/30/2005
 * http://www.ouah.org/ogay/sha2/
 *
 * Copyright (C) 2005, 2007 Olivier Gay <olivier.gay@a3.epfl.ch>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE PROJECT AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <stdint.h>
#include "sha224.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define SHA2_SHFR(x, n) (x >> n)
#define SHA2_ROTR(x, n) ((x >> n) | (x << ((sizeof(x) << 3) - n)))
#define SHA2_ROTL(x, n) ((x << n) | (x >> ((sizeof(x) << 3) - n)))
#define SHA2_CH(x, y, z) ((x & y) ^ (~x & z))
#define SHA2_MAJ(x, y, z) ((x & y) ^ (x & z) ^ (y & z))
#define SHA224_F1(x) (SHA2_ROTR(x, 2) ^ SHA2_ROTR(x, 13) ^ SHA2_ROTR(x, 22))
#define SHA224_F2(x) (SHA2_ROTR(x, 6) ^ SHA2_ROTR(x, 11) ^ SHA2_ROTR(x, 25))
#define SHA224_F3(x) (SHA2_ROTR(x, 7) ^ SHA2_ROTR(x, 18) ^ SHA2_SHFR(x, 3))
#define SHA224_F4(x) (SHA2_ROTR(x, 17) ^ SHA2_ROTR(x, 19) ^ SHA2_SHFR(x, 10))
#define SHA2_UNPACK32(x, str)                \
    {                                        \
        *((str) + 3) = (uint8_t)((x));       \
        *((str) + 2) = (uint8_t)((x) >> 8);  \
        *((str) + 1) = (uint8_t)((x) >> 16); \
        *((str) + 0) = (uint8_t)((x) >> 24); \
    }
#define SHA2_PACK32(str, x)                                                                                      \
    {                                                                                                            \
        *(x) = ((uint32_t) * ((str) + 3)) | ((uint32_t) * ((str) + 2) << 8) | ((uint32_t) * ((str) + 1) << 16) | \
               ((uint32_t) * ((str) + 0) << 24);                                                                 \
    }

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static const uint32_t SHA224_k[64] = {
    0x428A2F98, 0x71374491, 0xB5C0FBCF, 0xE9B5DBA5, 0x3956C25B, 0x59F111F1, 0x923F82A4, 0xAB1C5ED5,
    0xD807AA98, 0x12835B01, 0x243185BE, 0x550C7DC3, 0x72BE5D74, 0x80DEB1FE, 0x9BDC06A7, 0xC19BF174,
    0xE49B69C1, 0xEFBE4786, 0x0FC19DC6, 0x240CA1CC, 0x2DE92C6F, 0x4A7484AA, 0x5CB0A9DC, 0x76F988DA,
    0x983E5152, 0xA831C66D, 0xB00327C8, 0xBF597FC7, 0xC6E00BF3, 0xD5A79147, 0x06CA6351, 0x14292967,
    0x27B70A85, 0x2E1B2138, 0x4D2C6DFC, 0x53380D13, 0x650A7354, 0x766A0ABB, 0x81C2C92E, 0x92722C85,
    0xA2BFE8A1, 0xA81A664B, 0xC24B8B70, 0xC76C51A3, 0xD192E819, 0xD6990624, 0xF40E3585, 0x106AA070,
    0x19A4C116, 0x1E376C08, 0x2748774C, 0x34B0BCB5, 0x391C0CB3, 0x4ED8AA4A, 0x5B9CCA4F, 0x682E6FF3,
    0x748F82EE, 0x78A5636F, 0x84C87814, 0x8CC70208, 0x90BEFFFA, 0xA4506CEB, 0xBEF9A3F7, 0xC67178F2};

/*******************************************************************************
 * Code
 ******************************************************************************/

static void sha224_transform(sha224_t *context, const uint8_t *message, uint32_t block_nb)
{
    uint32_t w[64];
    uint32_t wv[8];
    uint32_t t1, t2;
    const uint8_t *sub_block;
    uint32_t i, j;
    for (i = 0; i < (uint32_t)block_nb; i++)
    {
        sub_block = message + (i << 6);
        for (j = 0; j < 16; j++)
        {
            SHA2_PACK32(&sub_block[j << 2], &w[j]);
        }
        for (j = 16; j < 64; j++)
            w[j] = SHA224_F4(w[j - 2]) + w[j - 7] + SHA224_F3(w[j - 15]) + w[j - 16];
        for (j = 0; j < 8; j++)
            wv[j] = (context->m_h)[j];
        for (j = 0; j < 64; j++)
        {
            t1 = wv[7] + SHA224_F2(wv[4]) + SHA2_CH(wv[4], wv[5], wv[6]) + SHA224_k[j] + w[j];
            t2 = SHA224_F1(wv[0]) + SHA2_MAJ(wv[0], wv[1], wv[2]);
            wv[7] = wv[6];
            wv[6] = wv[5];
            wv[5] = wv[4];
            wv[4] = wv[3] + t1;
            wv[3] = wv[2];
            wv[2] = wv[1];
            wv[1] = wv[0];
            wv[0] = t1 + t2;
        }
        for (j = 0; j < 8; j++)
            (context->m_h)[j] += wv[j];
    }
}

void sha224_init(sha224_t *context)
{
    (context->m_h)[0] = 0xc1059ed8;
    (context->m_h)[1] = 0x367cd507;
    (context->m_h)[2] = 0x3070dd17;
    (context->m_h)[3] = 0xf70e5939;
    (context->m_h)[4] = 0xffc00b31;
    (context->m_h)[5] = 0x68581511;
    (context->m_h)[6] = 0x64f98fa7;
    (context->m_h)[7] = 0xbefa4fa4;
    (context->m_len) = 0;
    (context->m_tot_len) = 0;
}

void sha224_update(sha224_t *context, const uint8_t *message, const uint32_t len)
{
    uint32_t block_nb;
    uint32_t new_len, rem_len, tmp_len;
    const uint8_t *shifted_message;
    uint32_t j;
    tmp_len = SHA224_BLOCK_SIZE - (context->m_len);
    rem_len = len < tmp_len ? len : tmp_len;
    // memcpy(&((context->m_block)[(context->m_len)]), message, rem_len);
    for (j = 0; j < rem_len; j++)
        (context->m_block)[(context->m_len) + j] = message[j];
    if ((context->m_len) + len < SHA224_BLOCK_SIZE)
    {
        (context->m_len) += len;
        return;
    }
    new_len = len - rem_len;
    block_nb = new_len / SHA224_BLOCK_SIZE;
    shifted_message = message + rem_len;
    sha224_transform(context, context->m_block, 1);
    sha224_transform(context, shifted_message, block_nb);
    rem_len = new_len % SHA224_BLOCK_SIZE;
    // memcpy(context->m_block, &shifted_message[block_nb << 6], rem_len);
    for (j = 0; j < rem_len; j++)
        (context->m_block)[j] = shifted_message[(block_nb << 6) + j];
    (context->m_len) = rem_len;
    (context->m_tot_len) += (block_nb + 1) << 6;
}

void sha224_final(sha224_t *context, uint8_t *digest)
{
    uint32_t block_nb;
    uint32_t pm_len;
    uint32_t len_b;
    uint32_t i;
    block_nb = (1 + ((SHA224_BLOCK_SIZE - 9) < ((context->m_len) % SHA224_BLOCK_SIZE)));
    len_b = ((context->m_tot_len) + (context->m_len)) << 3;
    pm_len = block_nb << 6;
    // memset((context->m_block) + (context->m_len), 0, pm_len-(context->m_len));
    for (i = 0; i < (pm_len - (context->m_len)); i++)
        ((context->m_block) + (context->m_len))[i] = 0;
    (context->m_block)[(context->m_len)] = 0x80;
    SHA2_UNPACK32(len_b, (context->m_block) + pm_len - 4);
    sha224_transform(context, (context->m_block), block_nb);
    for (i = 0; i < 7; i++)
        SHA2_UNPACK32((context->m_h)[i], &digest[i << 2]);
}
