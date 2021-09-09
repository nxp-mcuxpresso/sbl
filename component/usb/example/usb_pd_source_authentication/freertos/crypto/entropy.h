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

#ifndef _ENTROPY_H_
#define _ENTROPY_H_

#define ENTROPY_VERSION (((ENTROPY_VERSION_MAJOR) << 16) | ((ENTROPY_VERSION_MINOR) << 8) | (ENTROPY_VERSION_REVISION))
#define ENTROPY_VERSION_MAJOR 1
#define ENTROPY_VERSION_MINOR 0
#define ENTROPY_VERSION_REVISION 1

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Initialize entropy module.
 *
 * @note This API should be called at the beginning of the application before using the driver.
 *
 * @return 0 on succes\n
 *         != 0 on failure
 */
uint32_t entropy_init(void);

/*!
 * @brief Deinitialize entropy module.
 *
 * @return 0 on succes\n
 *         != 0 on failure
 */
uint32_t entropy_deinit(void);

/*!
 * @brief Harvest specified number of bytes of hardware random entropy so that it can be used to seed a PRNG.
 *
 * @param buf       Buffer to store gathered entropy in
 * @param bufsize   Number of bytes of entropy to gather and store in buf
 *
 * @return 0 on succes\n
 *         != 0 on failure
 */
uint32_t entropy_harvest(uint8_t *buf, const uint32_t bufsize);

/*!
 * @brief Calculate entropy of given buffer.
 *
 * To prevent having to use floating-point calculations, log2 calculations are performed using a lookup table.
 * If the local counters exceed the size of the lookup tables, the function will return with an error code.
 *
 * @param buf       Buffer to calcualte entropy over
 * @param bufSize   Number of bytes in buf to calculate entropy over. Must be a power of 2
 * @param shannon   Pointer where to store shannon result (as shannon * 1000)
 * @param chisq     Pointer where to store Chi-Square result
 * @param bitCount  Pointer where to store bit-count (number of set bits) result
 *
 * @return 0 on succes\n
 *         != 0 on failure
 */
uint32_t entropy_calculate(uint8_t *buf, uint16_t bufSize, uint32_t *shannon, uint32_t *chisq, uint32_t *bitCount);

#ifdef __cplusplus
}
#endif

#endif // _ENTROPY_H_
