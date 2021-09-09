/*
 * The Clear BSD License
 * Copyright 2015 - 2017 NXP
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

#include "usb_pd_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "usb_pd_ptn5110.h"
#include "usb_pd_ptn5110_register.h"
#include "string.h"
#include "usb_pd_timer.h"
#include "usb_pd_spec.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TCPC_RECEIVE_DETECT_ENABLE_RESET_MSG_MASK (TCPC_RECEIVE_DETECT_ENABLE_HARD_RESET_MASK)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void PDPTN5110_MsgHalSetRxBufDoneFreeExtBuf(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t force);
void PDPTN5110_IntcIrqEnable(pd_phy_ptn5110_instance_t *ptn5110Instance, uint16_t mask);
void PDPTN5110_IntcIrqDisable(pd_phy_ptn5110_instance_t *ptn5110Instance, uint16_t mask);
void PD_WaitUsec(uint32_t us);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void PDPTN5110_MsgResetAllMsgId(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    memset(ptn5110Instance->msgId, 0, sizeof(ptn5110Instance->msgId));
    memset(ptn5110Instance->rcvMsgId, 0, sizeof(ptn5110Instance->rcvMsgId));
    memset(ptn5110Instance->firstMsgAfterReset, true, sizeof(ptn5110Instance->firstMsgAfterReset));
}

static inline void PDPTN5110_MsgIncrMsgId(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sop)
{
    ptn5110Instance->msgId[sop] = (uint8_t)(ptn5110Instance->msgId[sop] + 1) & (uint8_t)0x07u;
}

static inline void PDPTN5110_MsgResetMsgId(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sop)
{
    ptn5110Instance->msgId[sop] = 0;
    ptn5110Instance->rcvMsgId[sop] = 0;
    ptn5110Instance->firstMsgAfterReset[sop] = true;
}

static inline uint8_t PDPTN5110_MsgHalRxPktSopBuf(uint8_t SOP)
{
    return (((SOP) == kPD_MsgSOP) ? 0 : 1);
}

#define MSG_TX_BUF_RELEASE                                                 \
    {                                                                      \
        do                                                                 \
        {                                                                  \
            if (ptn5110Instance->msgTxBufState == TCPC_TX_BUF_UNAVAILABLE) \
            {                                                              \
                ptn5110Instance->msgTxBufState = TCPC_TX_BUF_AVAILABLE;    \
            }                                                              \
        } while (0);                                                       \
    }

/* TCPM must writes the content and configuration of the message to be transmitted into the TRANSMIT_BUFFER before start
 */
/* transmitting */
#define MSG_TX_START_TRANSMIT \
    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->tcpcRegCache.MSG_TX.transmit, 1, transmit)

#if 0
static uint8_t PDPTN5110_MsgHalTxStartTransmit(void)
{
    const uint8_t rxBuf = (MSG_DATA(msg_tx_sop) == msgSop) ? 0 : 1;
    bool retVal = false;
    INTC_DEV_LOCK_CS();
    /* Perform the check for pending message in a critical section
     * with the TX write - other wise the INTC may come and clear the
     * RX pendining in the TCPC after the check but before the
     * TRANSMIT write. */
    if (MsgHalGetRxPending(rxBuf))
    {
        if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
        {
            /* Reset the transmit buffer to discard any message that may have been loaded. */
            REG_SET_REG(CLP_PORT, command, TCPC_RESETTRASMITBUFFER);
        }
        MSG_DATA(msg_tx_buf_state) = TCPC_TX_BUF_AVAILABLE;
    }
    else
    {
        Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->tcpcRegCache.MSG_TX.transmit, 1, transmit)
        retVal = true;
    }
    return retVal;
}
#endif

/*! **************************************************************************
    \brief Reset the protcol layer hardware
******************************************************************************/
void PDPTN5110_MsgHalProtocolLayerResetAndPowerUp(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* reset config to POR */
    ptn5110Instance->msgRxSopMask = 0;
    PDPTN5110_MsgHalSetReceiveDetect(ptn5110Instance, POR_receive_detect);

    /* Disable Bist Test Mode */
    ptn5110Instance->tcpcRegCache.CONTROL.tcpc_control &= (uint8_t) ~((uint8_t)TCPC_TCPC_CONTROL_BIST_TEST_MODE_MASK);
    Reg_BusClrBit(ptn5110Instance, tcpc_control, TCPC_TCPC_CONTROL_BIST_TEST_MODE_MASK);

    PDPTN5110_MsgResetAllMsgId(ptn5110Instance);

    /* Restore message reception enable */
    /* PDPTN5110_MsgEnableRxAfterReset(ptn5110Instance); */
}

/*! ***************************************************************************
    \brief  After any reset we need to re-enable the message RX
    \note Only called from CLP task context
******************************************************************************/
static void PDPTN5110_MsgEnableRxAfterReset(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    ptn5110Instance->msgRxSopMask = kPD_MsgSOPMask;
    PDPTN5110_MsgHalSetReceiveDetect(ptn5110Instance,
                                     ptn5110Instance->msgRxSopMask | TCPC_RECEIVE_DETECT_ENABLE_RESET_MSG_MASK);
}

/*! ***************************************************************************
    \brief common function to clear pending and abort
******************************************************************************/
void PDPTN5110_MsgHalClearPendingAndAbort(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    PDPTN5110_MsgHalSetRxBufDoneFreeExtBuf(ptn5110Instance, 1);
    ptn5110Instance->msgRxSopMask = 0;
    ptn5110Instance->msgTxSop = kPD_MsgSOP;
    ptn5110Instance->txAbort = false;

    ptn5110Instance->msgTxBufState = TCPC_TX_BUF_AVAILABLE;
}

/*! ***************************************************************************
    \brief Setup passive PD monitoring (unused, low power related)
******************************************************************************/
void PDPTN5110_MsgHalEnablePassiveDetection(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    PDPTN5110_MsgHalSetReceiveDetect(ptn5110Instance,
                                     ptn5110Instance->msgRxSopMask | TCPC_RECEIVE_DETECT_ENABLE_RESET_MSG_MASK);
}

/*! ***************************************************************************
    \brief Disable message RX and reset the protocol layer (for SOURCE_DISABLE state) (unused, low power related)
******************************************************************************/
void PDPTN5110_MsgHalDisableMessageRx(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Disable message reception */
    ptn5110Instance->msgRxSopMask = 0;
    PDPTN5110_MsgHalSetReceiveDetect(ptn5110Instance, TCPC_RECEIVE_DETECT_ENABLE_RESET_MSG_MASK);
}

/*! ***************************************************************************
    \brief Power down the USBPD TX hardware
******************************************************************************/
void PDPTN5110_MsgHalPowerDownTx(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
}

/*! ***************************************************************************
    \brief Power down the USBPD RX hardware
******************************************************************************/
void PDPTN5110_MsgHalPowerDownRx(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
}

/*! ***************************************************************************
    \brief Set/Change the port role, must be called before any messages are sent.
    \param  role        The current role for this device.
    \note Called from task context
******************************************************************************/
void PDPTN5110_MsgHalSetPortRole(pd_phy_ptn5110_instance_t *ptn5110Instance,
                                 uint8_t revision,
                                 uint8_t powerRole,
                                 uint8_t dataRole)
{
    /* Set the role for each */
    /* As we are not a cable plug, set upper bits to 0 */
    ptn5110Instance->revision = revision;
    Reg_BusWriteByte(
        ptn5110Instance, message_header_info,
        (((uint32_t)dataRole << TCPC_MESSAGE_HEADER_INFO_DATA_ROLE_LSB) & TCPC_MESSAGE_HEADER_INFO_DATA_ROLE_MASK) |
            (((uint32_t)revision << TCPC_MESSAGE_HEADER_INFO_USB_PD_SPECIFICATION_REVISION_LSB) &
             TCPC_MESSAGE_HEADER_INFO_USB_PD_SPECIFICATION_REVISION_MASK) |
            (((uint32_t)powerRole << TCPC_MESSAGE_HEADER_INFO_POWER_ROLE_LSB) &
             TCPC_MESSAGE_HEADER_INFO_POWER_ROLE_MASK));
}

/*! ***************************************************************************
    \brief Change the SOP used for transmit
    \param sop Type of sop used for next packet transmit
******************************************************************************/
void PDPTN5110_MsgHalSetTxSopMode(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sop)
{
    ptn5110Instance->msgTxSop = sop;
    Reg_CacheWriteField(ptn5110Instance, MSG_TX, transmit, TCPC_TRANSMIT_TRANSMIT_SOP_MESSAGE_MASK, sop);
}

/*! ***************************************************************************
    \brief Change the SOP mask used for receive
    \param sop_mask mask where each bit selects the type of SOP to be received
******************************************************************************/
void PDPTN5110_MsgHalSetRxSopEnable(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sop_mask)
{
    if (sop_mask != ptn5110Instance->msgRxSopMask)
    {
        ptn5110Instance->msgRxSopMask = sop_mask;
        /* always enable Hard Reset reception */
        PDPTN5110_MsgHalSetReceiveDetect(ptn5110Instance, sop_mask | TCPC_RECEIVE_DETECT_ENABLE_RESET_MSG_MASK);
    }
}

/*! ***************************************************************************
    \brief Common prefix for all send data and control functions
    \note Only called from CLP task context
******************************************************************************/
static uint8_t PDPTN5110_MsgHalSendCommonPrefix(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sop)
{
    /* If Buffer is using by previous transmitted msg or received msg, current Transmit should fail. */
    if (ptn5110Instance->msgTxBufState != TCPC_TX_BUF_AVAILABLE)
    {
        return true;
    }

    /* Buffer will now be used for TX */
    ptn5110Instance->msgTxBufState = TCPC_TX_BUF_UNAVAILABLE;

    PDPTN5110_MsgHalSetTxSopMode(ptn5110Instance, sop);
    sop = (1 << sop);
    if (!(ptn5110Instance->msgRxSopMask & sop))
    {
        /* We need to control TX and RX sop together - otherwise we cannot receive good crc response */
        PDPTN5110_MsgHalSetRxSopEnable(ptn5110Instance, sop);
    }

    return false;
}

/*! ***************************************************************************
    \brief Common prefix for all send data functions
    \param  type    Type of data message to send
    \param  log     Enable logging
    \note Only called from CLP task context
******************************************************************************/
static uint8_t PDPTN5110_MsgHalSendDataPrefix(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sop)
{
    if (PDPTN5110_MsgHalSendCommonPrefix(ptn5110Instance, sop))
    {
        return true;
    }

    return false;
}

#define MSG_SEND_DATA_PREFIX(ptn5110Instance, SOP, TYPE)          \
    do                                                            \
    {                                                             \
        if (PDPTN5110_MsgHalSendDataPrefix(ptn5110Instance, SOP)) \
        {                                                         \
            return false;                                         \
        }                                                         \
    } while (0)

#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
/*! ***************************************************************************
    \brief Common prefix for all send extended functions
    \param  type    Type of extended message to send
    \param  log     Enable logging
    \note Only called from CLP task context
******************************************************************************/
static uint8_t PDPTN5110_MsgHalSendExtendedPrefix(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sop, uint8_t type)
{
    if (PDPTN5110_MsgHalSendCommonPrefix(ptn5110Instance, sop))
    {
        return true;
    }

    return false;
}

#define MSG_SEND_EXTENDED_PREFIX(ptn5110Instance, SOP, TYPE)                \
    do                                                                      \
    {                                                                       \
        if (PDPTN5110_MsgHalSendExtendedPrefix(ptn5110Instance, SOP, TYPE)) \
        {                                                                   \
            return false;                                                   \
        }                                                                   \
    } while (0)
#endif

/*! ***************************************************************************
    \brief Common register access code for all send data functions to set the request
    \note Only called from CLP task context
******************************************************************************/
static void PDPTN5110_MsgHalSetTxDataReq(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sop, uint8_t count)
{
    /* Allow automatic retry when sending data message */
    Reg_CacheWriteField(ptn5110Instance, MSG_TX, transmit, TCPC_TRANSMIT_RETRY_COUNTER_MASK,
                        AUTOMATICALLY_RETRY_MESSAGE_TRANSMISSION_THREE_TIMES);

    /* transmit_byte_count is the number of bytes in the TX_BUFFER_DATA_OBJECTS plus two (for the TX_BUF_HEADER) */
    /* count is word (4 bytes), convert it to bytes by shifting left 2 bits */
    ptn5110Instance->msgTxBuf[TCPC_TRANSMIT_BYTE_COUNT_POS] = (count << 2) + TCPC_TX_BUF_HEADER_LEN;

    /* set msg header msg id */
    ptn5110Instance->msgTxBuf[TCPC_TX_BUF_HEADER_BYTE_POS + 1] &=
        (uint8_t)(~(TCPC_TX_BUF_HEADER_TX_MESSAGE_ID_MASK >> 8));
    ptn5110Instance->msgTxBuf[TCPC_TX_BUF_HEADER_BYTE_POS + 1] |=
        (uint8_t)((ptn5110Instance->msgId[sop]) << (TCPC_TX_BUF_HEADER_TX_MESSAGE_ID_LSB - 8));
}

#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
/* (unused) */
/*
static uint8_t PDPTN5110_MsgHalAmsReadyToSend(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    switch (ptn5110Instance->msgAMSState)
    {
    case PRL_Tx_Src_Pending:
        return PDPTN5110_MsgHalWaitSendReady(ptn5110Instance, PR_SOURCE);
    case PRL_Tx_Sink_Pending:
        return PDPTN5110_MsgHalWaitSendReady(ptn5110Instance, PR_SINK);
    default:
        break;
    }
    return true;
}
*/
#endif

/*! ***************************************************************************
    \brief Send a control message, specify sop
******************************************************************************/
uint8_t PDPTN5110_MsgHalSendControl(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sop, uint8_t type)
{
    uint8_t retValue = false;

    {
        if (!PDPTN5110_MsgHalSendCommonPrefix(ptn5110Instance, sop))
        {
            if (type == kPD_MsgSoftReset)
            {
                PDPTN5110_MsgResetMsgId(ptn5110Instance, sop);
            }
            PDPTN5110_MsgHalSetTxDataReq(ptn5110Instance, sop, 0);

            /* Update tx_byte_count/tx_buf_header before sending */
            Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->msgTxBuf[TCPC_TRANSMIT_BYTE_COUNT_POS],
                              TCPC_TRANSMIT_BYTE_COUNT_LEN, transmit_byte_count);
            Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->msgTxBuf[TCPC_TX_BUF_HEADER_BYTE_POS],
                              TCPC_TX_BUF_HEADER_LEN, tx_buf_header);

            /* If we're still using Transmit Buf, start Transmitting. Otherwise the transmition has been interrupted by
             */
            /* RX, stop Transmitting */
            if (ptn5110Instance->msgTxBufState == TCPC_TX_BUF_UNAVAILABLE)
            {
                /* TCPM must writes the content and configuration of the message to be transmitted into the */
                /* TRANSMIT_BUFFER before start transmitting */
                MSG_TX_START_TRANSMIT;
                retValue = true;
            }
        }
    }
    return retValue;
}

/*! ***************************************************************************
    \brief Send a data message. The data object must be loaded to the PDPTN5110_MsgHalGetTxPayloadPtr() buffer before
 calling this
 function.
    \param  count   Number of power data objects
    \retval true    We were able to initiate sending a source capabilities message
    \retval false   We were not able to initiate sending a source capabilities message
    \note Only called from CLP task context
    \note The data object contents of the message must be written to the buffer returned by
 PDPTN5110_MsgHalGetTxPayloadPtr()
 ******************************************************************************/
uint8_t PDPTN5110_MsgHalSendData(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sop, uint8_t type, uint8_t count)
{
    /* TODO: fill the data MsgHalGetTxPayloadPtr MsgHalGetRxPayloadPtr */
    uint8_t retValue = false;

    {
        MSG_SEND_DATA_PREFIX(ptn5110Instance, sop, type);

        PDPTN5110_MsgHalSetTxDataReq(ptn5110Instance, sop, count);

        /* Write byte_count/tx_buf_header/tx_buf_obj_byte in one transaction, as per USB TCPC specification 4.3.5 */
        /* Writing the TRANSMIT_BUFFER */
        Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->msgTxBuf[TCPC_TRANSMIT_BYTE_COUNT_POS],
                          (((uint32_t)count << 2) + TCPC_TX_BUF_HEADER_LEN + TCPC_TRANSMIT_BYTE_COUNT_LEN),
                          transmit_byte_count);

        /* If we're still using Transmit Buf, start Transmitting. Otherwise the transmition has been interrupted by RX,
         */
        /* stop Transmitting */
        if (ptn5110Instance->msgTxBufState == TCPC_TX_BUF_UNAVAILABLE)
        {
            /* TCPM must writes the content and configuration of the message to be transmitted into the TRANSMIT_BUFFER
             */
            /* before start transmitting */
            MSG_TX_START_TRANSMIT;
            retValue = true;
        }
    }
    return retValue;
}

#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
/*! ***************************************************************************
    \brief  Send an extended data message unchunked.
    \param  dataSize - of extended message in bytes, doesn't contain the extend header
    \param  raw - pointer to outgoing fifo
    \retval true    We were able to initiate sending a
    \retval false   We were not able to initiate
    \note Only called from CLP task context
    \note
    \note
 ******************************************************************************/
uint8_t PDPTN5110_MsgHalSendUnchunked(pd_phy_ptn5110_instance_t *ptn5110Instance,
                                      uint8_t sop,
                                      uint8_t type,
                                      uint16_t dataSize)
{
    uint8_t retValue = false;
    uint8_t i;
    int16_t dataRemain;
    uint16_t txCount;
    uint8_t *bufPtr;

    if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
    {
        /* Reg_BusWriteByte(ptn5110Instance, command, TCPC_RESETTRASMITBUFFER); */
        MSG_SEND_EXTENDED_PREFIX(ptn5110Instance, sop, type);
        PDPTN5110_MsgHalSetTxDataReq(ptn5110Instance, sop, 0);
        /* Standard TCPC 2.0 interface */
        uint8_t saved_byte; /* Used saved_byte to make sure we don't have any side effects on *raw */
        uint8_t *i2c_write_buf_ptr = (uint8_t *)&(ptn5110Instance->msgTxBuf[TCPC_TRANSMIT_BYTE_COUNT_POS]);

#if 0
        uint8_t hdr_buf[3];
        /* Header */
        hdr_buf[0] = 2;
        hdr_buf[1] = ptn5110Instance->msgTxBuf[TCPC_TX_BUF_HEADER_BYTE_POS];
        hdr_buf[2] = ptn5110Instance->msgTxBuf[TCPC_TX_BUF_HEADER_BYTE_POS + 1] | 0x80; /* Bit 15 extended message */
        Reg_BusWriteBlock(ptn5110Instance, hdr_buf, 3, transmit_byte_count);
#endif

        /* 1st half of data */
        dataSize += 2; /* the normal header */
        saved_byte = i2c_write_buf_ptr[0];
        i2c_write_buf_ptr[0] =
            (dataSize > MSG_TCPC_MAX_TX_WINDOW_WRITE_BURST) ? MSG_TCPC_MAX_TX_WINDOW_WRITE_BURST : dataSize;
        Reg_BusWriteBlock(ptn5110Instance, i2c_write_buf_ptr, i2c_write_buf_ptr[0] + 1, transmit_byte_count);
        i2c_write_buf_ptr[0] = saved_byte;

        /* Second half of data */
        if (dataSize > MSG_TCPC_MAX_TX_WINDOW_WRITE_BURST)
        {
            i2c_write_buf_ptr += MSG_TCPC_MAX_TX_WINDOW_WRITE_BURST;
            saved_byte = i2c_write_buf_ptr[0];
            i2c_write_buf_ptr[0] = dataSize - MSG_TCPC_MAX_TX_WINDOW_WRITE_BURST;
            Reg_BusWriteBlock(ptn5110Instance, i2c_write_buf_ptr, i2c_write_buf_ptr[0] + 1, transmit_byte_count);
            i2c_write_buf_ptr[0] = saved_byte;
        }
    }
    else /* Legacy vendor defined interface */
    {
        MSG_SEND_EXTENDED_PREFIX(ptn5110Instance, sop, type);

        if (dataSize < 4)
        {
            dataSize = 4;
        }
        dataSize += 2; /* the normal header */

        txCount = dataRemain = dataSize;
        if (txCount > 30)
        {
            txCount = 30;
        }

        PDPTN5110_MsgHalSetTxDataReq(ptn5110Instance, sop, (txCount - 2) >> 2);
        /* Set extended message bit on message header */
        ptn5110Instance->msgTxBuf[TCPC_TX_BUF_HEADER_BYTE_POS + 1] |= 0x80;

        /* Write byte_count/tx_buf_header/tx_buf_obj_byte in one transaction, as per USB TCPC specification 4.3.5 */
        /* Writing the TRANSMIT_BUFFER */
        Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->msgTxBuf[TCPC_TRANSMIT_BYTE_COUNT_POS],
                          (txCount + TCPC_TRANSMIT_BYTE_COUNT_LEN), transmit_byte_count);

        dataRemain -= txCount;
        /* Need to use TCPC extended tx buffer */
        if (dataRemain > 0)
        {
            bufPtr = (uint8_t *)&ptn5110Instance->msgTxBuf[TCPC_TX_BUF_HEADER_BYTE_POS] + 30;
            for (i = 1; dataRemain > 0; i++)
            {
                txCount = dataRemain;
                if (txCount > 30)
                {
                    txCount = 30;
                }

                /* set ext_msg_index */
                Reg_BusWriteByte(ptn5110Instance, ptn5110_ext_msg_index, i);
                /* copy data (30 bytes) to tcpc tx buffer */
                Reg_BusWriteBlock(ptn5110Instance, bufPtr, txCount, tx_buf_header);
                dataRemain -= txCount;
                bufPtr += txCount;
            }
        }

        /* Msg header + ext_msg header + ext_data */
        Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&dataSize, 2, ptn5110_ext_transmit_byte_count);
    }

    /* If we're still using Transmit Buf, start Transmitting. Otherwise the transmition has been interrupted by RX,
     */
    /* stop Transmitting */
    if (ptn5110Instance->msgTxBufState == TCPC_TX_BUF_UNAVAILABLE)
    {
        MSG_TX_START_TRANSMIT;
        retValue = true;
    }
    return retValue;
}

/*! ***************************************************************************
    \brief Save the unchunked extended message in the receive fifo
    \note Only called from CLP task context
    \note format of dst is
    {pad0, frame type, header byte 0, header byte 1, data 0, data 1.. data_n)
******************************************************************************/
void PDPTN5110_MsgHalRcvdExtMsgUnChunked(pd_phy_ptn5110_instance_t *ptn5110Instance,
                                         uint8_t *padded_dst,
                                         pd_phy_rx_result_t *rxResult)
{
    if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
    {
        /* Allow for MSG_HAL_RCVD_EXT_MSG_UNCHUNKED_PAD_BYTES to not match the TCPC2 requirements if
         * another platform is added.*/
        uint8_t *dst = padded_dst - 2; /* 2 bytes receive_byte_count and frame_type */
        uint16_t total_bytes_received = 0;
        uint8_t rx_buf_count;
        uint8_t tmp_buf[2];

        /* Disable the alert until message buffer has been read. */
        /* PDPTN5110_IntcIrqDisable(ptn5110Instance, TCPC_ALERT_RECEIVE_SOP_MESSAGE_STATUS_MASK |
                                                  TCPC_ALERT_BEGINNING_SOP_MESSAGE_STATUS_MASK); */
        /* Move to start of buffer */
        Reg_BusWriteByte(ptn5110Instance, command, TCPC_RESETRECEIVEBUFFER);
        /* Read just the readable byte count. */
        Reg_BusReadBlock(ptn5110Instance, receive_byte_count, 1, &rx_buf_count);

        /* Returned data format is {readable_byte_count, frame type, data0, data1 ... dataN} */
        /* Need to store data0 at dst[0], so read buffer from */
        Reg_BusReadBlock(ptn5110Instance, receive_byte_count, rx_buf_count + 1, dst);
        total_bytes_received += (rx_buf_count - 1);

        if (Reg_CacheRead(ptn5110Instance, INTC, alert) & TCPC_ALERT_BEGINNING_SOP_MESSAGE_STATUS_MASK)
        {
            static const uint16_t ClrValue = TCPC_ALERT_BEGINNING_SOP_MESSAGE_STATUS_MASK;

            /* Move pointer to next location */
            dst += (rx_buf_count - 1); /* Number of data bytes, remove the 1 byte count for frame type */

            /* Save the bytes that are overwritten by {readable byte count, frame type} */
            tmp_buf[0] = dst[0]; /* save last two bytes of previous buffer read */
            tmp_buf[1] = dst[1]; /* which will get overwritten by readable_byte_count and frame_type of next read */

            /* Move TCPC window to next region in buffer */
            Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ClrValue, 2, alert);

            /* Re-read count */
            Reg_BusReadBlock(ptn5110Instance, receive_byte_count, 1, &rx_buf_count);
            total_bytes_received += (rx_buf_count - 1);

            /* Note: I am assuming that there is no need to limit
             * read length to 132 bytes */
            /* Should have finished reading ext message, but just in case
             * Assume that receive_byte_count will be zero once ext message is completely read */
            Reg_BusReadBlock(ptn5110Instance, receive_byte_count, rx_buf_count + 1, dst);

            dst[0] = tmp_buf[0]; /* restore overwritten bytes */
            dst[1] = tmp_buf[1];
        }

        PDPTN5110_MsgHalSetRxBufDoneFreeExtBuf(ptn5110Instance, 0);
        rxResult->rxLength = total_bytes_received;
        rxResult->rxResultStatus = kStatus_PD_Success;
    }
    else
    {
        /* MEAD vendor defined method does not use padding. */
        uint8_t *dst = padded_dst;
        uint8_t i;
        int16_t data_received;
        int16_t data_remain;

#if 0
        const uint8_t *buf_ptr = (const uint8_t *)ptn5110Instance->rxDataBuffer + 2;
        /* Copy header to output buffer */
        *dst++ = *msg_hdr_ptr++;
        *dst++ = *msg_hdr_ptr++;

        /* copy the first block of data objects to output buffer. */
        for (i = 0; i < 28; i++)
        {
            *dst++ = *buf_ptr++;
        }
#endif
        data_received = (dst[2] & PD_MSG_EXT_HEADER_DATA_SIZE_MASK);
        rxResult->rxLength = (data_received + 2 + 2); /* add ext header and normal header */
        dst += (2 + 28);                              /* head and first packet has been buffer */

        /* %%% FIXME: These registers are different per-chip, needs Hal layer %%% */
        if (data_received > 26)
        {
            /* need to use TCPC extended rx buffer */
            data_remain = data_received - 26;
            for (i = 1; data_remain > 0; i++)
            {
                /* set ext_msg_index */
                Reg_BusWriteByte(ptn5110Instance, ptn5110_ext_msg_index, i);
                /* copy data (30 bytes) from tcpc rx buffer */
                Reg_BusReadBlock(ptn5110Instance, rx_buf_header, 30, dst);
                data_remain -= 30;
                dst += 30;
            }
            Reg_BusWriteByte(ptn5110Instance, ptn5110_ext_msg_index, 0);
        }
        rxResult->rxResultStatus = kStatus_PD_Success;
    }
}

/*! ***************************************************************************
    \brief  Send an extended data message chunked.
    \param  count - number of data objects in message
    \retval true    We were able to initiate sending a
    \retval false   We were not able to initiate
    \note Only called from CLP task context
    \note
    \note
 ******************************************************************************/

uint8_t PDPTN5110_MsgHalSendChunked(pd_phy_ptn5110_instance_t *ptn5110Instance,
                                    uint8_t sop,
                                    uint8_t type,
                                    uint8_t count)
{
    uint8_t retValue = false;

    {
        MSG_SEND_EXTENDED_PREFIX(ptn5110Instance, sop, type);

        PDPTN5110_MsgHalSetTxDataReq(ptn5110Instance, sop, count);
        /* Set extended message bit on message header */
        ptn5110Instance->msgTxBuf[TCPC_TX_BUF_HEADER_BYTE_POS + 1] |= 0x80;

        /* Write byte_count/tx_buf_header/tx_buf_obj_byte in one transaction, as per USB TCPC specification 4.3.5 */
        /* Writing the TRANSMIT_BUFFER */
        Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->msgTxBuf[TCPC_TRANSMIT_BYTE_COUNT_POS],
                          ((count << 2) + TCPC_TX_BUF_HEADER_LEN + TCPC_TRANSMIT_BYTE_COUNT_LEN), transmit_byte_count);

        /* If we're still using Transmit Buf, start Transmitting. Otherwise the transmition has been interrupted by RX,
         */
        /* stop Transmitting */
        if (ptn5110Instance->msgTxBufState == TCPC_TX_BUF_UNAVAILABLE)
        {
            /* TCPM must writes the content and configuration of the message to be transmitted into the TRANSMIT_BUFFER
             */
            /* before start transmitting */
            MSG_TX_START_TRANSMIT;
            retValue = true;
        }
    }
    return retValue;
}

#endif

#if defined(PD_CONFIG_DEBUG_ACCESSORY_SUPPORT) && (PD_CONFIG_DEBUG_ACCESSORY_SUPPORT)
/*! ***************************************************************************
    \brief Send a custom data message. The data object must be loaded to the PDPTN5110_MsgHalGetTxPayloadPtr() buffer
 before
 calling this function.
    \param  count   Number of power data objects
    \param  numObjs   Number of power data objects to write in the header
    \retval true    We were able to initiate sending a source capabilities message
    \retval false   We were not able to initiate sending a source capabilities message
    \note Only called from CLP task context
    \note The data object contents of the message must be written to the buffer returned by
 PDPTN5110_MsgHalGetTxPayloadPtr()
 ******************************************************************************/
uint8_t PDPTN5110_MsgHalSendCustomData(
    pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sop, uint8_t type, uint8_t count, uint8_t numObjs)
{
    uint8_t retValue = false;

    {
        MSG_SEND_DATA_PREFIX(ptn5110Instance, sop, type);

        PDPTN5110_MsgHalSetTxDataReq(ptn5110Instance, sop, count);

        /* Write byte_count/tx_buf_header/tx_buf_obj_byte in one transaction, as per USB TCPC specification 4.3.5 */
        /* Writing the TRANSMIT_BUFFER */
        Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->msgTxBuf[TCPC_TRANSMIT_BYTE_COUNT_POS],
                          ((count << 2) + TCPC_TX_BUF_HEADER_LEN + TCPC_TRANSMIT_BYTE_COUNT_LEN), transmit_byte_count);

        /* If we're still using Transmit Buf, start Transmitting. Otherwise the transmition has been interrupted by RX,
         */
        /* stop Transmitting */
        if (ptn5110Instance->msgTxBufState == TCPC_TX_BUF_UNAVAILABLE)
        {
            /* TCPM must writes the content and configuration of the message to be transmitted into the TRANSMIT_BUFFER
             */
            /* before start transmitting */
            MSG_TX_START_TRANSMIT;
            retValue = true;
        }
    }
    return retValue;
}
#endif

/*! ***************************************************************************
    \brief Wait the result of the send after the tx_done interrupt has been received.
    \note  Once this is complete (unused only for cable reset)
******************************************************************************/
uint8_t PDPTN5110_MsgHalWaitSendResult(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    uint16_t txStatus;

    /* Poll the interrupt controller for a result */
    /* Need to read the status first, so that it doesn't stay set on a txAbort. */
    do
    {
        txStatus = PDPTN5110_IntcTxDoneSeen(ptn5110Instance);
        if (txStatus & TCPC_ALERT_TRANSMIT_SOP_MESSAGE_SUCCESSFUL_MASK)
        {
            PDPTN5110_MsgIncrMsgId(ptn5110Instance, ptn5110Instance->msgTxSop);

            MSG_TX_BUF_RELEASE;
            return true;
        }
        if (ptn5110Instance->txAbort)
        {
            ptn5110Instance->txAbort = false;
            PDPTN5110_MsgIncrMsgId(ptn5110Instance, ptn5110Instance->msgTxSop);
            if (txStatus & TCPC_ALERT_TRANSMIT_SOP_MESSAGE_FAILED_MASK)
            {
            }
            else
            {
            }
            MSG_TX_BUF_RELEASE;
            return false;
        }
    } while (1);
}

#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
#if 0
/*! ***************************************************************************
    \brief Wait for conditions to be right for sending the first message of AMS (unused)
******************************************************************************/
uint8_t PDPTN5110_MsgHalWaitSendReady(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t powerRole)
{
    while (1)
    {
        if ((PD_CONFIG_VERSION < 3) ||
            (powerRole == kPD_PowerRoleSource && PD_TimerCheckInvalidOrTimeOut(ptn5110Instance, tSinkTxTimer)) ||
            ((powerRole == kPD_PowerRoleSink) && ptn5110Instance->amsSinkTxOK))
        {
            ptn5110Instance->msgAMSState = PRL_Tx_AMS_In_Progress;
            return true;
        }
        if (ptn5110Instance->txAbort)
        {
            ptn5110Instance->txAbort = false;
            return false;
        }
    }
}
#endif
#endif

/* ------------------------------------------------------------------------ */
/* MSG RX */

/*! ***************************************************************************
  \brief Free the extended message buffer
  \param force - if true free up any pending ALERTS and clear assuming buffer will not be read.
******************************************************************************/
static void PDPTN5110_MsgHalSetRxBufDoneFreeExtBuf(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t force)
{
    const uint16_t ClrValue =
        force ? TCPC_ALERT_BEGINNING_SOP_MESSAGE_STATUS_MASK | TCPC_ALERT_RECEIVE_SOP_MESSAGE_STATUS_MASK :
                TCPC_ALERT_RECEIVE_SOP_MESSAGE_STATUS_MASK;
    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ClrValue, 2, alert);
    if (force)
    {
        /* Clear again - we may have 2nd region pending */
        Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ClrValue, 2, alert);
        /* Clear again - we may have 2nd message pending */
        Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ClrValue, 2, alert);
    }
    /* Enable receive interrupt */
    /* PDPTN5110_IntcIrqEnable(ptn5110Instance, TCPC_ALERT_RECEIVE_SOP_MESSAGE_STATUS_MASK); */
}

#if 0
/*! ***************************************************************************
    \brief Normal control for signalling the rx buffer that we have the packet.
    \note Will also clear the message pending status for a message that has not been read.
    \note Only called from CLP task context
******************************************************************************/
void PDPTN5110_MsgHalSetRxBufDone(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t rxBuf)
{
    /* Flag the message buffer as invalid */
    /* Critical section as this function can pre-empt in ISR */
    USB_OSA_SR_ALLOC();
    USB_OSA_ENTER_CRITICAL();

    if (ptn5110Instance->rx_process_index[rxBuf] == ptn5110Instance->msg_rx_buf_state[rxBuf])
    {
        /* The other buffer does not have data, so both are now available */
        ptn5110Instance->msg_rx_buf_state[rxBuf] = TCPC_RX_BUF_AVAILABLE;
        /* We have now completed processing all buffers */
        ptn5110Instance->rx_process_index[rxBuf] = ptn5110Instance->rx_update_index[rxBuf] = 0;
    }
    else
    {
        /* Swap to the other filled buffer and keep message pending */
        TCPC_UPDATE_RX_BUF_INDEX((ptn5110Instance->rx_process_index[rxBuf]));
    }
    USB_OSA_EXIT_CRITICAL();
}
#endif
/* ------------------------------------------------------------------------ */
/* MSG RX Header/SOP accessors */

/*! **************************************************************************
    \brief Read the device receive buffer to our intermediate buffer
    \note Only called from ISR
******************************************************************************/
void PDPTN5110_MsgHalRxReceiveBuffer(pd_phy_ptn5110_instance_t *ptn5110Instance,
                                     uint8_t *dest,
                                     pd_phy_rx_result_t *rxResult)
{
    /* Reading the RECEIVE_BUFFER from TCPC will include 0-RECEIVE_BYTE_COUNT, 1-RX_BUF_FRAME_TYPE, */
    /* 2-RX_BUF_HEADER_BYTE_0, 3-RX_BUF_HEADER_BYTE_1 */
    int bytes = Reg_CacheRead(ptn5110Instance, MSG_RX, receive_byte_count);
    if (bytes == 0)
    {
        return;
    }
    /* Ensure the buffer does not overflow */
    if (bytes > 31)
    {
        bytes = 31;
    }

    rxResult->rxSop = Reg_CacheRead(ptn5110Instance, MSG_RX, rx_buf_frame_type);
    *(uint8_t *)dest = (Reg_CacheRead(ptn5110Instance, MSG_RX, rx_buf_header) & 0xFF);
    dest++;
    *(uint8_t *)dest = (Reg_CacheRead(ptn5110Instance, MSG_RX, rx_buf_header) >> 8);
    dest++;

    /* Ensure the buffer does not overflow */
    if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
    {
        if (bytes > 0)
        {
            /* Reset and re-read from start - TODO - avoid this by doing byte read to READABLE_BYTE_COUNT */
            Reg_BusWriteByte(ptn5110Instance, command, TCPC_RESETRECEIVEBUFFER);
            PD_WaitUsec(10);
            Reg_BusReadBlock(ptn5110Instance, rx_buf_obj1, bytes - 1, dest);
        }
    }
    else
    {
        if (bytes > 0)
        {
            Reg_BusReadBlock(ptn5110Instance, rx_buf_obj1, bytes - 1, dest);
        }
    }

#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    if (Reg_CacheRead(ptn5110Instance, MSG_RX, rx_buf_header) & PD_MSG_HEADER_EXTENDED_MASK)
    {
        PDPTN5110_MsgHalRcvdExtMsgUnChunked(ptn5110Instance, dest - 2, rxResult);
    }
    else
#endif
    {
        rxResult->rxLength = bytes - 1;
        rxResult->rxResultStatus = kStatus_PD_Success;
    }
}

/*! ***************************************************************************
    \brief Set RECEIVE_DETECT register directly
    \note Called from task context and ISR
******************************************************************************/
void PDPTN5110_MsgHalSetReceiveDetect(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t value)
{
    Reg_BusWriteByte(ptn5110Instance, receive_detect, value);
}

/* ------------------------------------------------------------------------ */
/* MSG RESETS */

/*! ***************************************************************************
    \brief Send a hard reset or cable reset, and perform the associated reset of our state.
    hardResetOrCableReset: 0 - hard reset, 1 - cable reset.
******************************************************************************/
void PDPTN5110_MsgHalSendReset(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t hardResetOrCableReset)
{
    /* Buffer will now be used for TX, Hard Reset is allowed to be sent while the TCPC is still processing a previous */
    /* transmission */
    ptn5110Instance->msgTxBufState = TCPC_TX_BUF_UNAVAILABLE;

    /* Clear TX done cached status before sending */
    PDPTN5110_IntcTxDoneSeen(ptn5110Instance);

    /* Need to update Transmit SOP* message field to Hard Reset/Cable Reset type and Retry Counter should be set to 0 */
    Reg_CacheWrite(ptn5110Instance, MSG_TX, transmit, 5 + hardResetOrCableReset);

    /* TCPM must writes the content and configuration of the message to be transmitted into the TRANSMIT_BUFFER before
     */
    /* start transmitting */
    MSG_TX_START_TRANSMIT;

    if (hardResetOrCableReset == 0)
    {
        PD_TimerStart(ptn5110Instance->pdHandle, tMsgHardResetCompleteTimer, T_HARD_RESET_COMPLETE);

        /* Wait until Reset msg has been sent successafully */
        /* Need to read the status first, so that it doesn't stay set on a txAbort. */
        do
        {
            /* Wait for hard reset to be sent */
            uint16_t txStatus = PDPTN5110_IntcTxDoneSeen(ptn5110Instance);
            if ((txStatus & TX_HARDRESET_MASK) ||
                (PD_TimerCheckInvalidOrTimeOut(ptn5110Instance->pdHandle, tMsgHardResetCompleteTimer)))
            {
                ptn5110Instance->txAbort = false;
                MSG_TX_BUF_RELEASE;
                break;
            }
        } while (1);

        PD_TimerClear(ptn5110Instance->pdHandle, tMsgHardResetCompleteTimer);

        PDPTN5110_MsgResetAllMsgId(ptn5110Instance);

        /* Restore message reception enable */
        PDPTN5110_MsgEnableRxAfterReset(ptn5110Instance);
    }
    else
    {
        PDPTN5110_MsgHalWaitSendResult(ptn5110Instance);
        /* Reset all sop' sop'' msgid  after sending cable reset */
        PDPTN5110_MsgResetMsgId(ptn5110Instance, kPD_MsgSOPp);
        PDPTN5110_MsgResetMsgId(ptn5110Instance, kPD_MsgSOPpp);
        PDPTN5110_MsgResetMsgId(ptn5110Instance, kPD_MsgSOPDbg);
        PDPTN5110_MsgResetMsgId(ptn5110Instance, kPD_MsgSOPpDbg);
    }
}

/*! ***************************************************************************
    \brief Start the hard reset process after confirming receptioin of hard reset (unused)
******************************************************************************/
void PDPTN5110_MsgHalRcvdHardResetStart(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
}

/*! ***************************************************************************
    \brief After all S/W processing of a hard reset is complete, flag the hardware (unused)
******************************************************************************/
void PDPTN5110_MsgHalRcvdHardResetComplete(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    PDPTN5110_MsgResetAllMsgId(ptn5110Instance);
    PDPTN5110_MsgEnableRxAfterReset(ptn5110Instance);
}

/* ------------------------------------------------------------------------ */
/* MSG BIST */

/*! ***************************************************************************
    \brief Reset the HW bist mode PRBS generator
******************************************************************************/
void PDPTN5110_MsgHalResetBist(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
}

/*! ***************************************************************************
    \brief Control the HW bist mode
    This will send a test frame, continuous pattern or setup the hardware to receive bist
    \note This will also enable the tx_done interrupt for TX mode
******************************************************************************/
void PDPTN5110_MsgHalSetupBist(pd_phy_ptn5110_instance_t *ptn5110Instance, pd_bist_mst_t mode)
{
    /*    TCPC supports only two mode: 	kBIST_CarrierMode2, kBIST_TestData */
    /*    // Power up the load driver for TX - or disable for RX */
    /*    if (mode==kBIST_TransmitMode) { */
    /*        // Enable the interrupt for reception of the returned counters */
    /*        MSG_DATA(tx_bist_state) = BIST_TX_SENT; */
    /*    } */
    if (mode == kBIST_TestData)
    {
        ptn5110Instance->tcpcRegCache.CONTROL.tcpc_control |= (TCPC_TCPC_CONTROL_BIST_TEST_MODE_MASK);
        Reg_BusSetBit(ptn5110Instance, tcpc_control, TCPC_TCPC_CONTROL_BIST_TEST_MODE_MASK);
    }
    else if (mode == kBIST_CarrierMode2)
    {
        Reg_BusWriteByte(ptn5110Instance, transmit,
                         ((uint8_t)NO_MESSAGE_RETRY_IS_REQUIRED |
                          (uint8_t)TRANSMIT_TRANSMIT_SOP_MESSAGE_TRANSMIT_BIST_CARRIER_MODE_2));
    }
    else
    {
    }
}

/*! ***************************************************************************
    \brief Get the bist data from the HW registers (unused)
******************************************************************************/
/*void PDPTN5110_MsgHalGetBistErrCnt(pd_phy_ptn5110_instance_t *ptn5110Instance, msgBistDataObject_t *bistData)
{

}
*/

/* ------------------------------------------------------------------------ */
/* Interrupt callback */

/*! ***************************************************************************
    \brief This is called by the interrupt controller when we receive a hard reset.
    \note  Called only in interrupt context.

******************************************************************************/
void PDPTN5110_MsgInterruptCallbackHardReset(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Abort any pending tx */
    ptn5110Instance->txAbort = true;

    PDPTN5110_MsgResetAllMsgId(ptn5110Instance);
    PD_Notify(ptn5110Instance->pdHandle, PD_PHY_EVENT_HARD_RESET_RECEIVED, NULL);
}

/*! ***************************************************************************
    \brief This is called by the interrupt controller when we receive a message.
    \note  Called only in interrupt context.

******************************************************************************/
void PDPTN5110_MsgInterruptCallbackMsgRcvd(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    uint8_t frameType = (uint8_t)Reg_CacheRead(ptn5110Instance, MSG_RX, rx_buf_frame_type);
    uint8_t rxBuf = PDPTN5110_MsgHalRxPktSopBuf(frameType);
    uint16_t rxBufHeader = Reg_CacheRead(ptn5110Instance, MSG_RX, rx_buf_header);
    uint8_t msgType = (uint8_t)(((uint32_t)rxBufHeader & TCPC_RX_BUF_HEADER_RX_MESSAGE_TYPE_MASK) >>
                                TCPC_RX_BUF_HEADER_RX_MESSAGE_TYPE_LSB);
    uint8_t msgId = (uint8_t)(((uint32_t)rxBufHeader & TCPC_RX_BUF_HEADER_RX_MESSAGE_ID_MASK) >>
                              TCPC_RX_BUF_HEADER_RX_MESSAGE_ID_LSB);

    if ((msgType == kPD_MsgSoftReset) && (!(rxBufHeader & PD_MSG_HEADER_EXTENDED_MASK)) &&
        (!(rxBufHeader & PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_MASK)))
    {
        PDPTN5110_MsgResetMsgId(ptn5110Instance, rxBuf);
        /* Always receive SoftReset since SoftReset Message always has a MessageID value of zero */
    }
    else if ((msgId != ptn5110Instance->rcvMsgId[frameType]) || (ptn5110Instance->firstMsgAfterReset[frameType]))
    {
        /* Drop the duplicate messages */
        /* 1.When the first good packet is received after a reset, the receiver shall store a copy of the received */
        /* MessageID value */
        /* 2.If MessageID value in the received Message is different than the stored value, the receiver shall return a
         */
        /* GoodCRC */
        /* Message with the new MessageID value, store a copy of the new MessageID value and process the Message. */
        ptn5110Instance->firstMsgAfterReset[frameType] = false;
        ptn5110Instance->rcvMsgId[frameType] = msgId;
    }
    else
    {
        /* For subsequent Messages, if MessageID value in a received Message is the same as the stored value, */
        /* the receiver shall return a GoodCRC Message with that MessageID value and drop the Message */
        /* (this is a retry of an already received Message) */

        return;
    }

    /* The TCPM reads the RECEIVE_BUFFER.RECEIVE_BYTE_COUNT and RECEIVE_BUFFER.RX_BUF_FRAME_TYPE. */
    /* If the TCPC received an SOP* message, the TCPM reads as many bytes as defined in the */
    /* RECEIVE_BUFFER.RECEIVE_BYTE_COUNT */
    if (frameType <= RX_BUF_FRAME_TYPE_RECEIVED_SOP_MESSAGE_RECEIVED_SOP_PP)
    {
        PDPTN5110_MsgReceiveCompLete(ptn5110Instance, kStatus_PD_Success);
    }
}

void PDPTN5110_MsgHalSendAbortIsrProcess(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    PDPTN5110_MsgIncrMsgId(ptn5110Instance, ptn5110Instance->msgTxSop);
    MSG_TX_BUF_RELEASE;
    PDPTN5110_MsgSendCompLete(ptn5110Instance, kStatus_PD_Abort);
}

void PDPTN5110_MsgHalSendSuccessIsrProcess(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    PDPTN5110_MsgIncrMsgId(ptn5110Instance, ptn5110Instance->msgTxSop);
    MSG_TX_BUF_RELEASE;
    PDPTN5110_MsgSendCompLete(ptn5110Instance, kStatus_PD_Success);
}

#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
/*! **************************************************************************
  \brief Start AMS
  \note   Only called from CLP task context (unused)
******************************************************************************/
void PDPTN5110_MsgHalStartAMS(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t usedCC)
{
    if (usedCC == kPD_CC2)
    {
        Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, roleControl,
                                       TCPC_ROLE_CONTROL_RP_VALUE_MASK | TCPC_ROLE_CONTROL_CC2_MASK,
                                       (uint8_t)(ROLE_CONTROL_RP_VALUE_RP_1 | ROLE_CONTROL_CC2_RP));
    }
    else
    {
        Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, roleControl,
                                       TCPC_ROLE_CONTROL_RP_VALUE_MASK | TCPC_ROLE_CONTROL_CC1_MASK,
                                       (uint8_t)(ROLE_CONTROL_RP_VALUE_RP_1 | ROLE_CONTROL_CC1_RP));
    }
}

/*! **************************************************************************
  \brief End AMS
  \note   Only called from CLP task context (unused)
******************************************************************************/
void PDPTN5110_MsgHalEndAMS(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t usedCC)
{
    if (usedCC == kPD_CC2)
    {
        Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, roleControl,
                                       TCPC_ROLE_CONTROL_RP_VALUE_MASK | TCPC_ROLE_CONTROL_CC2_MASK,
                                       (uint8_t)(ROLE_CONTROL_RP_VALUE_RP_3 | ROLE_CONTROL_CC2_RP));
    }
    else
    {
        Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, roleControl,
                                       TCPC_ROLE_CONTROL_RP_VALUE_MASK | TCPC_ROLE_CONTROL_CC1_MASK,
                                       (uint8_t)(ROLE_CONTROL_RP_VALUE_RP_3 | ROLE_CONTROL_CC1_RP));
    }
}

#endif

/* tx_done interrupt || soft_reset || hard_reset || PD_PHY_CANCEL_MSG_TX */
void PDPTN5110_MsgSendCompLete(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t sendResult)
{
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    if (ptn5110Instance->txHave)
    {
        ptn5110Instance->txHave = 0u;
        USB_OSA_EXIT_CRITICAL();

        if (!(ptn5110Instance->rxHave))
        {
            PDPTN5110_MsgHalSetRxSopEnable(ptn5110Instance, 0);
        }

        /* don't need the callback */
        if (sendResult == kStatus_PD_Cancel)
        {
            /*_PDphy_MsgSwAbortTx(ptn5110Instance); */
            /* TODO: need abort */
            return;
        }
        if (sendResult == kStatus_PD_Abort)
        {
            /* TODO: need abort */
        }

        /* notify pd stack */
        PD_Notify(ptn5110Instance->pdHandle, PD_PHY_EVENT_SEND_COMPLETE, &sendResult);
    }
    else
    {
        USB_OSA_EXIT_CRITICAL();
    }
}

/* receive interrupt */
void PDPTN5110_MsgReceiveCompLete(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t rxState)
{
    pd_phy_rx_result_t rxResult;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    if (ptn5110Instance->rxHave)
    {
        ptn5110Instance->rxHave = 0u;
        USB_OSA_EXIT_CRITICAL();

        PDPTN5110_MsgHalSetRxSopEnable(ptn5110Instance, 0);
        if (rxState == kStatus_PD_Cancel)
        {
            return;
        }

        if (rxState == kStatus_PD_Success)
        {
            PDPTN5110_MsgHalRxReceiveBuffer(ptn5110Instance, ptn5110Instance->rxDataBuffer, &rxResult);
        }
        else
        {
            rxResult.rxLength = 0;
            rxResult.rxSop = kPD_MsgSOPInvalid;
            rxResult.rxResultStatus = rxState;
        }
        PD_Notify(ptn5110Instance->pdHandle, PD_PHY_EVENT_RECEIVE_COMPLETE, &rxResult);
    }
    else
    {
        if (rxState == kStatus_PD_Success)
        {
            ptn5110Instance->cacheRxValid = 1;
        }
        USB_OSA_EXIT_CRITICAL();
        if (rxState == kStatus_PD_Success)
        {
            /* cache the received message */
            PDPTN5110_MsgHalRxReceiveBuffer(ptn5110Instance, (uint8_t *)&(ptn5110Instance->msgRxCacheBuf[0]),
                                            (pd_phy_rx_result_t *)&(ptn5110Instance->cacheRxResult));
        }
    }
}
