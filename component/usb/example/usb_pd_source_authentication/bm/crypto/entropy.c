// ############################################################################
// #             Copyright (C), NXP Semiconductors                            #
// #                       (C), NXP B.V. of Eindhoven                         #
// #                                                                          #
// # All rights are reserved. Reproduction in whole or in part is prohibited  #
// # without the written consent of the copyright owner.                      #
// # NXP reserves the right to make changes without notice at any time.       #
// # NXP makes no warranty, expressed, implied or statutory, including but    #
// # not limited to any implied warranty of merchantibility or fitness for    #
// # any particular purpose, or that the use will not infringe any third      #
// # party patent, copyright or trademark. NXP must not be liable for any     #
// # loss or damage arising from its use.                                     #
// ############################################################################

#include <stdint.h>
#include "entropy.h"
#include "entropy_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define LOG2_TABLE_NUM_VALUES 128

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* hamming weight values for first 16 numbers */
static const uint32_t hammingWeight4[16] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
static const uint32_t hammingWeight4diff2sqr[16] = {4, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 4};

/* log2 values left shifted 13 bits for first LOG2_TABLE_NUM_VALUES numbers */
static const uint16_t log2_lshift13[LOG2_TABLE_NUM_VALUES + 1] = {
    0,     0,     8192,  12984, 16384, 19021, 21176, 22998, 24576, 25968, 27213, 28340, 29368, 30314, 31190,
    32005, 32768, 33484, 34160, 34799, 35405, 35982, 36532, 37057, 37560, 38042, 38506, 38952, 39382, 39797,
    40197, 40585, 40960, 41324, 41676, 42019, 42352, 42676, 42991, 43298, 43597, 43889, 44174, 44452, 44724,
    44989, 45249, 45503, 45752, 45996, 46234, 46469, 46698, 46923, 47144, 47361, 47574, 47783, 47989, 48191,
    48389, 48585, 48777, 48966, 49152, 49335, 49516, 49693, 49868, 50041, 50211, 50379, 50544, 50707, 50868,
    51026, 51183, 51338, 51490, 51641, 51789, 51936, 52081, 52224, 52366, 52506, 52644, 52781, 52916, 53049,
    53181, 53312, 53441, 53569, 53695, 53820, 53944, 54066, 54188, 54308, 54426, 54544, 54661, 54776, 54890,
    55003, 55115, 55226, 55336, 55445, 55553, 55660, 55766, 55871, 55975, 56078, 56181, 56282, 56383, 56482,
    56581, 56679, 56777, 56873, 56969, 57064, 57158, 57251, 57344};

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Calculate log2 of a number that is a power of 2 */
static uint8_t log2OfPowerOf2Int(uint32_t powerOf2Int)
{
    uint8_t log2 = 0;

    while (powerOf2Int >>= 1)
    {
        log2++;
    }

    return log2;
}

uint32_t entropy_init(void)
{
    return entropy_hal_init();
}

uint32_t entropy_deinit(void)
{
    return entropy_hal_deinit();
}

uint32_t entropy_harvest(uint8_t *buf, const uint32_t bufsize)
{
    return entropy_hal_harvest(buf, bufsize);
}

uint32_t entropy_calculate(uint8_t *buf, uint16_t bufSize, uint32_t *shannon, uint32_t *chisqr, uint32_t *bits)
{
    uint16_t count[16];
    uint32_t c, j;
    *shannon = 0;
    *chisqr = 0;
    *bits = 0;

    if (1 << log2OfPowerOf2Int(bufSize) != bufSize)
    {
        /* bufSize is not a power of 2 */
        return 1;
    }

    for (j = 0; j < 16; j++)
    {
        count[j] = 0;
    }
    for (j = 0; j < bufSize; j++)
    {
        c = (uint32_t)buf[j];
        /* increment nibble counts for entropy calc */
        (count[(c >> 4)])++;
        (count[(c & 15)])++;
        if ((count[(c >> 4)] > LOG2_TABLE_NUM_VALUES) || (count[(c & 15)] > LOG2_TABLE_NUM_VALUES))
        {
            /* Count too high for precalculated log2 table */
            return 1;
        }
        /* add up set bits (hamming-weight) per nibble */
        *bits += ((uint32_t)hammingWeight4[c >> 4]) + ((uint32_t)hammingWeight4[c & 15]);
        *chisqr += ((uint32_t)hammingWeight4diff2sqr[c >> 4]) + ((uint32_t)hammingWeight4diff2sqr[c & 15]);
    }
    /* calculate Shannon entropy from 4bit counts */
    for (j = 0; j < 16; j++)
        *shannon += ((uint32_t)count[j]) * ((uint32_t)log2_lshift13[(uint32_t)count[j]]);
    *shannon = (((log2OfPowerOf2Int(bufSize * 2) * bufSize * 2 * 8192) - *shannon) * 1000 + (bufSize * 2 * 8192 / 2)) /
               (bufSize * 2 * 8192);
    /* shannon is from 0-4, rounded to 3 decimal places */
    return 0;
}
