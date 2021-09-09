/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __EMVL1_INTERFACE_H__
#define __EMVL1_INTERFACE_H__

#if defined(USING_PHY_TDA8035)
#include "fsl_smartcard_phy.h"
#endif
#if defined(USING_PHY_EMVSIM)
#include "fsl_smartcard_phy_emvsim.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef void (*emv_callback_t)(uint8_t evnet, void *buffer, uint32_t size);

#define EMV_SMART_CARD_PROTOCOL_T0 (0x01U)
#define EMV_SMART_CARD_PROTOCOL_T1 (0x02U)

typedef enum _emv_smart_card_convention
{
    kEmvSmartCardConventionDirect = (0x00U),
    kEmvSmartCardConventionInverse = (0x01U),
} emv_smart_card_convention_t;

typedef enum _emv_smart_card_checksum_type
{
    kEmvSmartCardChecksumTypeLRC = (0x00U),
    kEmvSmartCardChecksumTypeCRC = (0x01U),
} emv_smart_card_checksum_type_t;

enum
{
    kEmvSmartCardEventCardInserted = 1U,
    kEmvSmartCardEventCardRemoved,
};

enum
{
    kStatus_CCID_EMV_Success = 0x00U,
    kStatus_CCID_EMV_CardRemoved = 0x80U,
    kStatus_CCID_EMV_InvalidParameter = 0x81U,
    kStatus_CCID_EMV_Unsupported = 0x82U,

    kStatus_CCID_EMV_Error = 0xFFU,
};

typedef struct _emv_card_data_information_struct
{
    uint8_t FI;
    uint8_t DI;
    uint8_t GTN;
    emv_smart_card_convention_t convention;
    union
    {
        struct
        {
            uint8_t WI;
        } t0;
        struct
        {
            emv_smart_card_checksum_type_t checksumType;
            uint8_t BWI;
            uint8_t CWI;
            uint8_t IFSC;
        } t1;
    } paramUnion;
} emv_card_data_information_struct_t;

#if defined(__cplusplus)
extern "C" {
#endif

void EMVL1_SmartCardInit(emv_callback_t callback);
uint8_t EMVL1_SmartCardPowerOn(uint8_t *atrBuffer, uint32_t *length);
uint8_t EMVL1_SmartCardPowerOff(void);
uint8_t EMVL1_SmartCardGetProtocol(void);
uint8_t EMVL1_SmartCardPresence(void);
uint8_t EMVL1_SmartCardGetInformation(emv_card_data_information_struct_t *information);
uint8_t EMVL1_SendApduCommand(uint8_t *commandApdu,
                              uint32_t commandApduLength,
                              uint8_t *ResponseApdu,
                              uint32_t *ResponseApduLength);

#if defined(__cplusplus)
}
#endif

#endif /* __EMVL1_INTERFACE_H__ */
