/* Copyright 2014, Kenneth MacKay. Licensed under the BSD 2-clause license. */

/* open-source light-weight elliptic short Weierstrass prime field */
/* curve implementation                                            */

#ifndef _UECC_H_
#define _UECC_H_

/*!
 * @addtogroup uecc
 * @{
 * @brief Elliptic Curve Cryptography ECDH and ECDSA implementation.
 */

/*! @file */

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @name Module version
 * @{
 * @brief Version 1.0.0
 */
#define UECC_VERSION (((UECC_VERSION_MAJOR) << 16) | ((UECC_VERSION_MINOR) << 8) | (UECC_VERSION_REVISION))
#define UECC_VERSION_MAJOR 1
#define UECC_VERSION_MINOR 0
#define UECC_VERSION_REVISION 0
/*! @} */

/*!
 * @name Platform selection
 * @{ @brief Define uECC_PLATFORM to any of these values to configure the platform uECC will run on. If uECC_PLATFORM is
 * not defined, the code will try to guess it based on compiler macros.
 */
#define uECC_arch_other 0
#define uECC_x86 1
#define uECC_x86_64 2
#define uECC_arm 3
#define uECC_arm_thumb 4
#define uECC_arm_thumb2 5
#define uECC_arm64 6
#define uECC_avr 7
/*! @} */
#if 0
#ifndef uECC_PLATFORM
#if defined(__thumb2__) || defined(_M_ARMT)
#define uECC_PLATFORM uECC_arm_thumb2
#elif defined(__thumb__)
#define uECC_PLATFORM uECC_arm_thumb
#elif defined(__arm__) || defined(_M_ARM)
#define uECC_PLATFORM uECC_arm
#elif defined(__aarch64__)
#define uECC_PLATFORM uECC_arm64
#elif defined(__i386__) || defined(_M_IX86) || defined(_X86_) || defined(__I86__)
#define uECC_PLATFORM uECC_x86
#elif defined(__amd64__) || defined(_M_X64)
#define uECC_PLATFORM uECC_x86_64
#else
#define uECC_PLATFORM uECC_arch_other
#endif
#endif
#endif

#define uECC_PLATFORM uECC_arch_other

typedef int32_t wordcount_t;
typedef int32_t bitcount_t;
typedef int32_t cmpresult_t;

typedef uint32_t uECC_word_t;
typedef uint64_t uECC_dword_t;

#define HIGH_BIT_SET 0x80000000
#define uECC_WORD_BITS 32
#define uECC_WORD_BITS_SHIFT 5
#define uECC_WORD_BITS_MASK 0x01F

#ifndef uECC_OPTIMIZATION_LEVEL
/*!
 * @name uECC optimization level
 * @{ @brief Trade speed for code size, larger values produce code that is faster but larger. Currently supported values
 * are 0 - 3; 0 is unusably slow for most applications. If not defined, it will default to optimization level 2.
 */
#define uECC_OPTIMIZATION_LEVEL 2
/*! @} */
#endif

#ifndef uECC_SQUARE_FUNC
/*!
 * @name Fast scalar squaring
 * @{ @brief If enabled (defined as nonzero), this will cause a specific function to be used for (scalar) squaring
 * instead of the generic multiplication function. This can make things faster, but increases the code size.
 */
#define uECC_SQUARE_FUNC 0
/*! @} */
#endif

/*!
 * @name Curve support selection
 * @{ @brief Set to 0 to remove specified curve.
 */
#define uECC_SUPPORTS_secp224r1 1
/*! @} */

struct uECC_Curve_t;
/*! @brief ECC curve structure. */
typedef const struct uECC_Curve_t *uECC_Curve;

/*!
 * @brief Returns NIST P-224 curve.
 *
 * @return NIST P-224 curve as uECC_Curve
 */
uECC_Curve uECC_secp224r1(void);

/*!
 * @brief Returns the size of a private key (in bytes) for the curve.
 *
 * @param curve Curve to get size of private key of
 *
 * @return Size of private key in bytes
 */
uint32_t uECC_curve_private_key_size(uECC_Curve curve);

/*!
 * @brief Returns the size of a public key (in bytes) for the curve.
 *
 * @param curve Curve to get size of public key of
 *
 * @return Size of public key in bytes
 */
uint32_t uECC_curve_public_key_size(uECC_Curve curve);

/*!
 * @brief Check to see if a public key is valid.
 *
 * Note that you are not required to check for a valid public key before using any other uECC functions. However, you
 * may wish to avoid spending CPU time computing a shared secret or verifying a signature using an invalid public key.
 *
 * @param public_key    The public key to check
 * @param curve         Curve to use
 *
 * @return 1 if valid\n
 *         0 if invalid
 */
uint32_t uECC_valid_public_key(const uint8_t *public_key, uECC_Curve curve);

/*!
 * @brief Compute the corresponding public key for a private key.
 *
 * @param private_key   Private key to compute public key for
 * @param public_key    Pointer to where to store computed public key
 * @param curve         Curve to use
 *
 * @return 1 if the key was computed successfully\n
 *         0 if an error occured
 */
uint32_t uECC_compute_public_key(const uint8_t *private_key, uint8_t *public_key, uECC_Curve curve);

/*!
 * @brief Verify an ECDSA signature.
 *
 * Usage: Compute the hash of the signed data using the same hash as the signer and pass it to this function along with
 * the signer's public key and the signature values (r and s).
 *
 * @param public_key    Signer's public key
 * @param message_hash  Hash of the signed data
 * @param hash_size     Size of message_hash in bytes
 * @param signature     Signature value
 * @param curve         Curve to use
 *
 * @return 1 if signature is valid\n
 *         0 if signature is invalid
 */
uint32_t uECC_verify(const uint8_t *public_key,
                     const uint8_t *message_hash,
                     const uint32_t hash_size,
                     const uint8_t *signature,
                     uECC_Curve curve);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* _UECC_H_ */
