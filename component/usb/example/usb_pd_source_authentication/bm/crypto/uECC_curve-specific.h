/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _UECC_CURVE_SPECIFIC_H_
#define _UECC_CURVE_SPECIFIC_H_

#define num_words_secp224r1 7
#define num_bytes_secp224r1 28

#define BYTES_TO_WORDS_8(a, b, c, d, e, f, g, h) 0x##d##c##b##a, 0x##h##g##f##e
#define BYTES_TO_WORDS_4(a, b, c, d) 0x##d##c##b##a

static void double_jacobian_default(uECC_word_t *X1, uECC_word_t *Y1, uECC_word_t *Z1, uECC_Curve curve)
{
    /* t1 = X, t2 = Y, t3 = Z */
    uECC_word_t t4[uECC_MAX_WORDS];
    uECC_word_t t5[uECC_MAX_WORDS];
    wordcount_t num_words = curve->num_words;

    if (uECC_vli_isZero(Z1, num_words))
    {
        return;
    }

    uECC_vli_modSquare_fast(t4, Y1, curve);   /* t4 = y1^2 */
    uECC_vli_modMult_fast(t5, X1, t4, curve); /* t5 = x1*y1^2 = A */
    uECC_vli_modSquare_fast(t4, t4, curve);   /* t4 = y1^4 */
    uECC_vli_modMult_fast(Y1, Y1, Z1, curve); /* t2 = y1*z1 = z3 */
    uECC_vli_modSquare_fast(Z1, Z1, curve);   /* t3 = z1^2 */

    uECC_vli_modAdd(X1, X1, Z1, curve->p, num_words); /* t1 = x1 + z1^2 */
    uECC_vli_modAdd(Z1, Z1, Z1, curve->p, num_words); /* t3 = 2*z1^2 */
    uECC_vli_modSub(Z1, X1, Z1, curve->p, num_words); /* t3 = x1 - z1^2 */
    uECC_vli_modMult_fast(X1, X1, Z1, curve);         /* t1 = x1^2 - z1^4 */

    uECC_vli_modAdd(Z1, X1, X1, curve->p, num_words); /* t3 = 2*(x1^2 - z1^4) */
    uECC_vli_modAdd(X1, X1, Z1, curve->p, num_words); /* t1 = 3*(x1^2 - z1^4) */
    if (uECC_vli_testBit(X1, 0))
    {
        uECC_word_t l_carry = uECC_vli_add(X1, X1, curve->p, num_words);
        uECC_vli_rshift1(X1, num_words);
        X1[num_words - 1] |= l_carry << (uECC_WORD_BITS - 1);
    }
    else
    {
        uECC_vli_rshift1(X1, num_words);
    }
    /* t1 = 3/2*(x1^2 - z1^4) = B */

    uECC_vli_modSquare_fast(Z1, X1, curve);           /* t3 = B^2 */
    uECC_vli_modSub(Z1, Z1, t5, curve->p, num_words); /* t3 = B^2 - A */
    uECC_vli_modSub(Z1, Z1, t5, curve->p, num_words); /* t3 = B^2 - 2A = x3 */
    uECC_vli_modSub(t5, t5, Z1, curve->p, num_words); /* t5 = A - x3 */
    uECC_vli_modMult_fast(X1, X1, t5, curve);         /* t1 = B * (A - x3) */
    uECC_vli_modSub(t4, X1, t4, curve->p, num_words); /* t4 = B * (A - x3) - y1^4 = y3 */

    uECC_vli_set(X1, Z1, num_words);
    uECC_vli_set(Z1, Y1, num_words);
    uECC_vli_set(Y1, t4, num_words);
}

/* Computes result = x^3 + ax + b. result must not overlap x. */
static void x_side_default(uECC_word_t *result, const uECC_word_t *x, uECC_Curve curve)
{
    uECC_word_t _3[uECC_MAX_WORDS] = {3}; /* -a = 3 */
    wordcount_t num_words = curve->num_words;

    uECC_vli_modSquare_fast(result, x, curve);                      /* r = x^2 */
    uECC_vli_modSub(result, result, _3, curve->p, num_words);       /* r = x^2 - 3 */
    uECC_vli_modMult_fast(result, result, x, curve);                /* r = x^3 - 3x */
    uECC_vli_modAdd(result, result, curve->b, curve->p, num_words); /* r = x^3 - 3x + b */
}

// if uECC_SUPPORTS_secp224r1

#if (uECC_OPTIMIZATION_LEVEL > 0)
static void vli_mmod_fast_secp224r1(uECC_word_t *result, uECC_word_t *product);
#endif

static const struct uECC_Curve_t curve_secp224r1 = {
    num_words_secp224r1,
    num_bytes_secp224r1,
    224, /* num_n_bits */
    {BYTES_TO_WORDS_8(01, 00, 00, 00, 00, 00, 00, 00), BYTES_TO_WORDS_8(00, 00, 00, 00, FF, FF, FF, FF),
     BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF), BYTES_TO_WORDS_4(FF, FF, FF, FF)},
    {BYTES_TO_WORDS_8(3D, 2A, 5C, 5C, 45, 29, DD, 13), BYTES_TO_WORDS_8(3E, F0, B8, E0, A2, 16, FF, FF),
     BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF), BYTES_TO_WORDS_4(FF, FF, FF, FF)},
    {BYTES_TO_WORDS_8(21, 1D, 5C, 11, D6, 80, 32, 34), BYTES_TO_WORDS_8(22, 11, C2, 56, D3, C1, 03, 4A),
     BYTES_TO_WORDS_8(B9, 90, 13, 32, 7F, BF, B4, 6B), BYTES_TO_WORDS_4(BD, 0C, 0E, B7),

     BYTES_TO_WORDS_8(34, 7E, 00, 85, 99, 81, D5, 44), BYTES_TO_WORDS_8(64, 47, 07, 5A, A0, 75, 43, CD),
     BYTES_TO_WORDS_8(E6, DF, 22, 4C, FB, 23, F7, B5), BYTES_TO_WORDS_4(88, 63, 37, BD)},
    {BYTES_TO_WORDS_8(B4, FF, 55, 23, 43, 39, 0B, 27), BYTES_TO_WORDS_8(BA, D8, BF, D7, B7, B0, 44, 50),
     BYTES_TO_WORDS_8(56, 32, 41, F5, AB, B3, 04, 0C), BYTES_TO_WORDS_4(85, 0A, 05, B4)},
    &double_jacobian_default,
    &x_side_default,
#if (uECC_OPTIMIZATION_LEVEL > 0)
    &vli_mmod_fast_secp224r1
#endif
};

uECC_Curve uECC_secp224r1(void)
{
    return &curve_secp224r1;
}

#if (uECC_OPTIMIZATION_LEVEL > 0)
/* Computes result = product % curve_p
   from http://www.nsa.gov/ia/_files/nist-routines.pdf */

static void vli_mmod_fast_secp224r1(uint32_t *result, uint32_t *product)
{
    uint32_t tmp[num_words_secp224r1];
    int carry;

    /* t */
    uECC_vli_set(result, product, num_words_secp224r1);

    /* s1 */
    tmp[0] = tmp[1] = tmp[2] = 0;
    tmp[3] = product[7];
    tmp[4] = product[8];
    tmp[5] = product[9];
    tmp[6] = product[10];
    carry = uECC_vli_add(result, result, tmp, num_words_secp224r1);

    /* s2 */
    tmp[3] = product[11];
    tmp[4] = product[12];
    tmp[5] = product[13];
    tmp[6] = 0;
    carry += uECC_vli_add(result, result, tmp, num_words_secp224r1);

    /* d1 */
    tmp[0] = product[7];
    tmp[1] = product[8];
    tmp[2] = product[9];
    tmp[3] = product[10];
    tmp[4] = product[11];
    tmp[5] = product[12];
    tmp[6] = product[13];
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp224r1);

    /* d2 */
    tmp[0] = product[11];
    tmp[1] = product[12];
    tmp[2] = product[13];
    tmp[3] = tmp[4] = tmp[5] = tmp[6] = 0;
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp224r1);

    if (carry < 0)
    {
        do
        {
            carry += uECC_vli_add(result, result, curve_secp224r1.p, num_words_secp224r1);
        } while (carry < 0);
    }
    else
    {
        while (carry || uECC_vli_cmp_unsafe(curve_secp224r1.p, result, num_words_secp224r1) != 1)
        {
            carry -= uECC_vli_sub(result, result, curve_secp224r1.p, num_words_secp224r1);
        }
    }
}
#endif /* (uECC_OPTIMIZATION_LEVEL > 0) */

#endif /* _UECC_CURVE_SPECIFIC_H_ */
