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

#ifndef _BECC_H_
#define _BECC_H_

/*!
 * @addtogroup becc
 * @{
 * @brief Elliptic Curve Cryptography challenge/response generation and verification.
 */

/*! @file */

/*!
 * @name Module version
 * @{
 * @brief Version 1.0.0
 */
#define BECC_VERSION (((BECC_VERSION_MAJOR) << 16) | ((BECC_VERSION_MINOR) << 8) | (BECC_VERSION_REVISION))
#define BECC_VERSION_MAJOR 1
#define BECC_VERSION_MINOR 0
#define BECC_VERSION_REVISION 0
/*! @} */

#ifdef __cplusplus
extern "C" {
#endif

#define DEGREE 163 /* the degree of the field polynomial */
#define MARGIN 3   /* don't touch this */
/*! @brief Size of coordinate point in 32-bit words */
#define NUMWORDS ((DEGREE + MARGIN + 31) / 32)

typedef uint32_t bitstr_t[NUMWORDS];
/*! @brief Type for coordinate point in EPIF format */
typedef bitstr_t epif_t;
/*! @brief Type for field elements */
typedef bitstr_t elem_t;

/*!
 * @brief Converts point coordinate to EPIF format from normal representation
 *
 * @param destination   Destination where to store converted coordinate in EPIF format
 * @param x_value       Coordinate in standard representation
 */
void bECC_toEPIF(epif_t destination, const elem_t x_value);

/*!
 * @brief Converts point coordinate to normal representation from EPIF format
 *
 * @param destination   Destination where to store converted coordinate in normal representation
 * @param source_xepif  Coordinate in EPIF representation
 */
void bECC_fromEPIF(epif_t destination, const epif_t source_xepif);

/*!
 * @brief Generates challenge.
 *
 * @param x     Location where to store x-coordinate of challenge
 * @param r2    Location where to store check value R2 of challenge
 * @param r     Random number used to generate challenge
 * @param tmp   Temporarily storage
 */
void bECC_generateChallenge(elem_t x, elem_t r2, elem_t r, elem_t tmp);

/*!
 * @brief Computes reponse to challenge.
 *
 * @param resp  Location where to store computed response
 * @param r     Random number used during challenge
 * @param Qx    X-coordinate of public key
 * @param Qy    Y-coordinate of public key
 * @param tmp   Temporarily storage
 */
void bECC_precomputeResponse(elem_t resp, elem_t r, elem_t Qx, elem_t Qy, elem_t tmp);

/*!
 * @brief Verify reponse to challenge.
 *
 * @param result    Verification result. 0 if the response was correct, else the difference between x_P1 and x_P2
 * @param r2        precomputed response
 * @param X_P1      X-coordinate of response
 * @param Z_P1      Z-coordinate of response
 */
void bECC_verifyResponse(elem_t result, elem_t r2, elem_t X_P1, elem_t Z_P1);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* _BECC_H_ */
