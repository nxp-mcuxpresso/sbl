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
#include <string.h>
#include "board.h"
#include "pd_board_config.h"
#include "entropy_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#if (defined BOARD_SUPPORT_HARDWARE_RAND) && (BOARD_SUPPORT_HARDWARE_RAND)
extern void BOARD_RandInit(void);
extern void BOARD_RandDeinit(void);
extern void BOARD_RandGetData(uint8_t *data, uint32_t count);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

uint32_t entropy_hal_init(void)
{
#if (defined BOARD_SUPPORT_HARDWARE_RAND) && (BOARD_SUPPORT_HARDWARE_RAND)
    BOARD_RandInit();
#endif
    return 0;
}

uint32_t entropy_hal_deinit(void)
{
#if (defined BOARD_SUPPORT_HARDWARE_RAND) && (BOARD_SUPPORT_HARDWARE_RAND)
    BOARD_RandDeinit();
#endif
    return 0;
}

uint32_t entropy_hal_harvest(uint8_t *buf, const uint32_t bufsize)
{
#if (defined BOARD_SUPPORT_HARDWARE_RAND) && (BOARD_SUPPORT_HARDWARE_RAND)
    BOARD_RandGetData(buf, bufsize);
#endif

    return 0;
}
