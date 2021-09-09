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

#ifndef _SHA224_EXTRA_H_
#define _SHA224_EXTRA_H_

/*!
 * @addtogroup sha224
 * @{
 */

/*! @file */

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_SHA224_HKDF_INFO_SIZE 16

/*! @brief SHA224 HKDF context */
typedef struct
{
    sha224_t *sha224_context;
    uint8_t prk[SHA224_HASH_SIZE];
    uint8_t state[SHA224_HASH_SIZE + MAX_SHA224_HKDF_INFO_SIZE + 4];
    uint32_t count;
} sha224_hkdf_t;

/*!
 * @brief Compute SHA224-HMAC MAC tag of specified message with specified key.
 *
 * @param context       Pointer to SHA224 context
 * @param key           Secret key used for HMAC
 * @param keysize       Size of secret key
 * @param message       Message to be authenticated
 * @param messagesize   Size of message to be authenticated
 * @param mac           Pointer where to store the MAC
 */
void sha224_hmac(sha224_t *context,
                 const uint8_t *key,
                 const uint32_t keysize,
                 const uint8_t *message,
                 const uint32_t messagesize,
                 uint8_t *mac);

/*!
 * @brief Initialize SHA224-HKDF based PRNG, seeded with specified entropy.
 *
 * @param context       Pointer to SHA224 HKDF context to initialize
 * @param entropy       Entropy used for seeding
 * @param entropysize   Size of entropy
 */
void sha224_hkdf_extract(sha224_hkdf_t *context, const uint8_t *entropy, const uint32_t entropysize);

/*!
 * @brief Generate a single SHA224-HKDF PRNG iteration of 28 bytes, using specified HKDF info.
 *
 * Derived key available in first 28 bytes of context state.
 *
 * @param context   Pointer to SHA224 HKDF context used for generation
 * @param info      Pointer to info for key derivation
 * @param infosize  Size of info
 */
void sha224_hkdf_expand(sha224_hkdf_t *context, const uint8_t *info, const uint32_t infosize);

/*! @brief Macro to reference SHA224-HKDF PRNG iteration block (first 28 bytes) */
#define sha224_hkdf_block(C) ((C)->state)

#ifdef __cplusplus
}
#endif

/*! @} */

#endif // _SHA224_EXTRA_H_
