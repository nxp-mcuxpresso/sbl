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

#ifndef _SHA224_H_
#define _SHA224_H_

/*!
 * @addtogroup sha224
 * @{
 * @brief SHA224 algorithms. Implements SHA224 hash generation, HMAC and HKDF.
 */

/*! @file */

/*!
 * @name Module version
 * @{
 * @brief Version 1.0.0
 */
#define SHA224_VERSION (((SHA224_VERSION_MAJOR) << 16) | ((SHA224_VERSION_MINOR) << 8) | (SHA224_VERSION_REVISION))
#define SHA224_VERSION_MAJOR 1
#define SHA224_VERSION_MINOR 0
#define SHA224_VERSION_REVISION 0
/*! @} */

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Size of SHA224 hash (in byte) */
#define SHA224_HASH_SIZE (224 / 8)
/*! @brief Size of SHA224 block (in byte) */
#define SHA224_BLOCK_SIZE (512 / 8)

/*! @brief SHA224 context */
typedef struct
{
    uint32_t m_tot_len;
    uint32_t m_len;
    uint8_t m_block[2 * SHA224_BLOCK_SIZE];
    uint32_t m_h[8];
} sha224_t;

/*!
 * @brief Initialize SHA224 context.
 *
 * @param context   Pointer to SHA224 context to initialize
 */
void sha224_init(sha224_t *context);

/*!
 * @brief Update SHA224 context with specified message.
 *
 * @param context   Pointer to SHA224 context to update
 * @param message   Pointer to input message
 * @param len       Length of message
 */
void sha224_update(sha224_t *context, const uint8_t *message, const uint32_t len);

/*!
 * @brief Finalize SHA224 context and return digest.
 *
 * @param context   Pointer to SHA224 context to finalize
 * @param digest    Pointer to store final digest
 */
void sha224_final(sha224_t *context, uint8_t *digest);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif // _SHA224_H_
