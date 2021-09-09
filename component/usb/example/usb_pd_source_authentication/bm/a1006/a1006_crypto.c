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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sha224.h"
#include "uECC.h"
#include "bECC.h"
#include "a1006_crypto.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static const uint8_t cert_template[CERT_SIZE] = {
    0x30, 0x82, 0x01, 0x00, 0x30, 0x81, 0xDB, 0xA0, 0x03, 0x02, 0x01, 0x02, 0x02, 0x09, 0x01, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x30, 0x0A, 0x06, 0x08, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x04, 0x03, 0x01, 0x30,
    0x15, 0x31, 0x13, 0x30, 0x11, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x0C, 0x0A, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0xCC, 0xCC, 0x30, 0x22, 0x18, 0x0F, 0x31, 0x39, 0x37, 0x30, 0x30, 0x31, 0x30, 0x31, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x5A, 0x18, 0x0F, 0x39, 0x39, 0x39, 0x39, 0x31, 0x32, 0x33, 0x31, 0x32, 0x33, 0x35,
    0x39, 0x35, 0x39, 0x5A, 0x30, 0x40, 0x31, 0x12, 0x30, 0x10, 0x06, 0x03, 0x55, 0x04, 0x2D, 0x03, 0x09, 0x00,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x31, 0x13, 0x30, 0x11, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x0C,
    0x0A, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x31, 0x15, 0x30, 0x13, 0x06, 0x03, 0x55,
    0x04, 0x03, 0x0C, 0x0C, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x30, 0x40,
    0x30, 0x10, 0x06, 0x07, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x02, 0x01, 0x06, 0x05, 0x2B, 0x81, 0x04, 0x00, 0x0F,
    0x03, 0x2C, 0x00, 0x04, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x30, 0x0A, 0x06, 0x08, 0x2A, 0x86, 0x48, 0xCE,
    0x3D, 0x04, 0x03, 0x01, 0x03, 0x14, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static elem_t ECC_Qx, ECC_Qy, ECC_r, ECC_rGx, ECC_r2, ECC_X_P1, ECC_Z_P1;

/*******************************************************************************
 * Code
 ******************************************************************************/

/* CONVERT TO/FROM BINARY & ECC SCALAR */
static void bin2elem(elem_t E, const uint8_t *buf, const uint32_t bufsize)
{
    uint32_t j;
    uint32_t X;
    uint8_t const *p, *P;
    for (j = 0, p = (buf + bufsize - 1), P = (buf - 1); j < 6; j++)
    {
        X = 0;
        if (p != P)
            X = ((uint32_t) * (p--));
        if (p != P)
            X |= (((uint32_t) * (p--)) << 8);
        if (p != P)
            X |= (((uint32_t) * (p--)) << 16);
        if (p != P)
            X |= (((uint32_t) * (p--)) << 24);
        E[j] = X;
    }
}

static void elem2bin(uint8_t *buf, const uint32_t bufsize, const elem_t E)
{
    uint32_t j;
    uint32_t X;
    uint8_t *d;
    uint8_t const *D;
    for (j = 0, d = (buf + bufsize - 1), D = (buf - 1); j < 6; j++)
    {
        X = E[j];
        if (d != D)
            *(d--) = ((uint8_t)(X & 0xFF));
        X >>= 8;
        if (d != D)
            *(d--) = ((uint8_t)(X & 0xFF));
        X >>= 8;
        if (d != D)
            *(d--) = ((uint8_t)(X & 0xFF));
        X >>= 8;
        if (d != D)
            *(d--) = ((uint8_t)(X & 0xFF));
    }
    while (d != D)
        *(d--) = 0x00;
}

void a100x_decompressCert(uint8_t *cert, const uint8_t *uid, const uint8_t *certz)
{
    uint32_t j;
    memcpy(cert, cert_template, CERT_SIZE);
    memcpy((cert + CERT_SERIAL_OFF), uid, CERT_SERIAL_SIZE);
    memcpy((cert + CERT_SUBUID_OFF), uid, CERT_SERIAL_SIZE);
    memcpy((cert + CERT_ISSORG_OFF), (certz + CERTZ_SUBORG_OFF), CERT_ISSORG_SIZE);
    memcpy((cert + CERT_SUBORG_OFF), (certz + CERTZ_SUBORG_OFF), CERT_SUBORG_SIZE);
    memcpy((cert + CERT_SUBCN_OFF), (certz + CERTZ_SUBCN_OFF), CERT_SUBCN_SIZE);
    memcpy((cert + CERT_PUBKEY_OFF), (certz + CERTZ_PUBKEY_OFF), CERT_PUBKEY_SIZE);
    memcpy((cert + CERT_SIG_OFF), (certz + CERTZ_SIG_OFF), CERT_SIG_SIZE);
    /* adjust certificate DER lengths */
    j = 2 + (uint32_t)cert[CERT_SIG_OFF + 1];
    cert[CERT_SIG_OFF - 2] = (uint8_t)(j + 1);
    j += CERT_SIG_OFF - CERT_TBS_OFF;
    cert[CERT_TBS_OFF - 2] = (uint8_t)((j >> 8) & 0xFF);
    cert[CERT_TBS_OFF - 1] = (uint8_t)(j & 0xFF);
}

uint32_t a100x_verifyDERSig(
    const uint32_t *pubKey, const uint8_t *derSig, const uint8_t *databuf, const uint32_t datalen, sha224_t *context)
{
    static uint8_t tmpBuf[84];
    uint32_t j, k, n, n0;
    uint8_t c;
    /* parse DER encoded ECDSA into little endian (r, s) padded values */
    k = 0;
    if (derSig[k++] != 0x30)
        return ERROR_ECDSA_DER_SEQ_TAG;
    n0 = (uint32_t)derSig[k++];
    if (n0 < 17 || n0 > 62)
        return ERROR_ECDSA_DER_SEQ_LEN;
    if (derSig[k++] != 0x02)
        return ERROR_ECDSA_DER_R_INT_TAG;
    n = (uint32_t)derSig[k++];
    if (derSig[k] == 0x00)
    {
        k++;
        n--;
    }
    if (n < 8 || n > 28)
        return ERROR_ECDSA_DER_R_INT_LEN;
    for (j = 0; j < (28 - n); j++)
        tmpBuf[27 - j] = 0x00;
    for (; j < 28; j++)
        tmpBuf[27 - j] = derSig[k++];
    if (derSig[k++] != 0x02)
        return ERROR_ECDSA_DER_S_INT_TAG;
    n = (uint32_t)derSig[k++];
    if (derSig[k] == 0x00)
    {
        k++;
        n--;
    }
    if (n < 8 || n > 28 || (k + n) != (2 + n0))
        return ERROR_ECDSA_DER_S_INT_LEN;
    for (j = 0; j < (28 - n); j++)
        tmpBuf[55 - j] = 0x00;
    for (; j < 28; j++)
        tmpBuf[55 - j] = derSig[k++];
    /* hash data */
    sha224_init(context);
    sha224_update(context, databuf, datalen);
    sha224_final(context, (tmpBuf + 56));
    /* convert hash to little endian */
    for (j = 0; j < 14; j++)
    {
        c = tmpBuf[56 + j];
        tmpBuf[56 + j] = tmpBuf[83 - j];
        tmpBuf[83 - j] = c;
    }
    /* verify ECDSA signature with NIST-p224 curve */
    j = uECC_verify((uint8_t *)pubKey, (tmpBuf + 56), 28, tmpBuf, uECC_secp224r1());
    return (j == 1) ? 0 : ERROR_ECDSA_VERIFY_FAIL;
}

uint32_t a100x_generateChallenge(uint8_t *buf, const uint8_t *randBuf)
{
    bin2elem(ECC_r, randBuf, 21);
    (ECC_r[5]) &= 3;
    bECC_generateChallenge(ECC_rGx, ECC_r2, ECC_r, ECC_Z_P1);
    bECC_toEPIF(ECC_Qx, ECC_rGx);
    bECC_toEPIF(ECC_Qy, ECC_r2);
    elem2bin((buf), 22, ECC_Qx);
    elem2bin((buf + 22), 22, ECC_Qy);
    return 44;
}

void a100x_precomputeResponse(const uint8_t *pubKey)
{
    uint32_t j;
    /* copy certificate public key */
    j = CERT_PUBKEY_SIZE >> 1;
    bin2elem(ECC_Qx, (pubKey), j);
    bin2elem(ECC_Qy, (pubKey + j), j);
    bECC_precomputeResponse(ECC_r2, ECC_r, ECC_Qx, ECC_Qy, ECC_Z_P1);
}

uint32_t a100x_verifyResponse(const uint8_t *buf, const uint32_t bufSize)
{
    uint32_t j;
    uint32_t v;
    if (bufSize != 46 || buf[0] != 0x2D || buf[1] != 0x00)
        return (uint32_t)-1;
    bin2elem(ECC_Qx, (buf + 2), 22);
    bin2elem(ECC_Qy, (buf + 24), 22);
    bECC_fromEPIF(ECC_X_P1, ECC_Qx);
    bECC_fromEPIF(ECC_Z_P1, ECC_Qy);
    bECC_verifyResponse(ECC_r2, ECC_r2, ECC_X_P1, ECC_Z_P1);
    for (j = 0, v = 0; j < NUMWORDS; j++)
        v |= (ECC_r2[j]);
    return (v) ? 1 : 0;
}
