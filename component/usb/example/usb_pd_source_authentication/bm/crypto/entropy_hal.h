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

#ifndef _ENTROPY_HAL_H_
#define _ENTROPY_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Initialize entropy hal.
 *
 * @note This API should be called at the beginning of the application before using the driver.
 *
 * @return 0 on succes\n
 *         != 0 on failure
 */
uint32_t entropy_hal_init(void);

/*!
 * @brief Deinitialize entropy hal.
 *
 * @return 0 on succes\n
 *         != 0 on failure
 */
uint32_t entropy_hal_deinit(void);

/*!
 * @brief Harvest specified number of bytes of hardware random entropy so that it can be used to seed a PRNG.
 *
 * @param buf       Buffer to store gathered entropy in
 * @param bufsize   Number of bytes of entropy to gather and store in buf
 *
 * @return 0 on succes\n
 *         != 0 on failure
 */
uint32_t entropy_hal_harvest(uint8_t *buf, const uint32_t bufsize);

#ifdef __cplusplus
}
#endif

#endif // _ENTROPY_HAL_H_
