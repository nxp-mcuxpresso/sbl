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

#ifndef _A100X_CRYPTO_H_
#define _A100X_CRYPTO_H_

#include "sha224.h"

/*!
 * @addtogroup a100x_crypto
 * @{
 * @brief Cryptographic helper functions for A100x. Includes function to decompress and validate A1006 certificates, as
 * well as to generate A1006 authentication challenges and validate responses.
 */

/*! @file */

#ifdef __cplusplus
extern "C" {
#endif

#define CERT_SIZE 305

/*!
 * @name A100x certificate offsets and sizes
 * @{ @brief Offsets and sizes of fields in A100x certificate
 */
#define CERT_TBS_OFF 0x004
#define CERT_TBS_SIZE 222
#define CERT_SERIAL_OFF 0x00F
#define CERT_SERIAL_SIZE 8
#define CERT_ISSORG_OFF 0x030
#define CERT_ISSORG_SIZE 10
#define CERT_SUBUID_OFF 0x06C
#define CERT_SUBUID_SIZE 8
#define CERT_SUBORG_OFF 0x07F
#define CERT_SUBORG_SIZE 10
#define CERT_SUBCN_OFF 0x094
#define CERT_SUBCN_SIZE 12
#define CERT_PUBKEY_OFF 0x0B8
#define CERT_PUBKEY_SIZE 42
#define CERT_SIG_OFF 0x0F1
#define CERT_SIG_SIZE 64

#define CERTZ_SUBORG_OFF 0x000
#define CERTZ_SUBCN_OFF 0x00A
#define CERTZ_PUBKEY_OFF 0x016
#define CERTZ_SIG_OFF 0x040
/*! @} */

/*!
 * @name Error return codes
 * @{ @brief Return codes on error
 */
#define ERROR_ECDSA_DER_SEQ_TAG 1
#define ERROR_ECDSA_DER_SEQ_LEN 2
#define ERROR_ECDSA_DER_R_INT_TAG 3
#define ERROR_ECDSA_DER_R_INT_LEN 4
#define ERROR_ECDSA_DER_S_INT_TAG 5
#define ERROR_ECDSA_DER_S_INT_LEN 6
#define ERROR_ECDSA_VERIFY_FAIL 7
/*! @} */

/*!
 * @brief Decompress 8 byte UID & 128 byte compressed certificate into 305 octet (fixed size with right zero padding)
 * X509v3 DER encoded certificate using predefined certificate template.
 *
 * @param cert  Buffer to store uncompressed certificate
 * @param uid   Pointer to UID
 * @param certz Pointer to compressed certificate
 */
void a100x_decompressCert(uint8_t *cert, const uint8_t *uid, const uint8_t *certz);

/*!
 * @brief Verifies the DER encoded signature with NIST-p224 (secp224r1) curve, with the specified public key,
 * uncompressed (x,y) coordinated with little endian and zero padded, against against the specified data.
 *
 * @param pubKey    Pointer to public key used for verification
 * @param derSig    Pointer to DER encoded ECDSA+SHA224 signature
 * @param databuf   Pointer to data to verify
 * @param datalen   Length of data to verify
 * @param context   Context object for sha224
 *
 * @return 0 on verified OK\n
 *         ERROR_ECDSA_VERIFY_FAIL on verify fail
 */
uint32_t a100x_verifyDERSig(
    const uint32_t *pubKey, const uint8_t *derSig, const uint8_t *databuf, const uint32_t datalen, sha224_t *context);

/*!
 * @brief Verifies the DER encoded X509v3 certificate conforming to the certificate template.
 *
 * @param PUBKEY    Pointer to public key used for verification
 * @param CERT      Pointer to DER encoded X509v3 certificate
 * @param CONTEXT   Context object for sha224
 *
 * @return 0 on verified OK\n
 *         ERROR_ECDSA_VERIFY_FAIL on verify fail
 */
#define a100x_verifyCert(PUBKEY, CERT, CONTEXT)                                              \
    a100x_verifyDERSig((const uint32_t *)(PUBKEY), (const uint8_t *)((CERT) + CERT_SIG_OFF), \
                       (const uint8_t *)((CERT) + CERT_TBS_OFF), (uint32_t)CERT_TBS_SIZE, (sha224_t *)(CONTEXT))

/* generates the A100x challenge from 21 bytes of random, storing */
/* ephemerial private key internally, while outputing into buffer */
/* the A100x challenge message, returning that message byte size  */

/*!
 * @brief Generates the A100x challenge from random data, stores the ephemerial private key internally.
 *
 * @param buf       Buffer to store the challenge in
 * @param randBuf   Pointer to 21 random bytes
 *
 * @return Length of challenge
 */
uint32_t a100x_generateChallenge(uint8_t *buf, const uint8_t *randBuf);

/*!
 * @brief Precomputes the expected challenge response, computed using previously stored ephemeral private key and
 * specified A1006 public key from previously parsed certificate, and stores this precomputed affine x-coordinate result
 * internally.
 *
 * @param pubKey    Pointer to public key used to precompute the reponse
 */
void a100x_precomputeResponse(const uint8_t *pubKey);

/*!
 * @brief Compares A100x response to challenge with precomputed reponse.
 * Before comparing, the precomputed reponse is converted from affine to the randomized projective coordinate.
 *
 * @param buf       Pointer to A100x reponse to challenge
 * @param bufSize   Size of reponse
 *
 * @return 0 if validation passed\n
           1 if validation failed
 */
uint32_t a100x_verifyResponse(const uint8_t *buf, const uint32_t bufSize);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif // _A100X_CRYPTO_H_
