/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "usb_pd_config.h"
#include "usb_pd.h"
#include "usb_osa.h"
#include "usb_cmsis_wrapper.h"
#include "pd_i2c_over_vdm.h"
#include "pd_command_interface.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define NXP_VENDOR_ID 0x1FC9u
#define NXP_VDM_SENDER_RESPONSE (25)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

i2c_over_vdm_state_t s_i2cOverVdmState[1];

/*******************************************************************************
 * Code
 ******************************************************************************/

static void PD_I2cOverVdmTimer_Start(i2c_over_vdm_state_t *i2cOvdmState, uint32_t time)
{
    uint32_t sr = DisableGlobalIRQ();
    __ASM("CPSID I");
    i2cOvdmState->timerValue = time;
    EnableGlobalIRQ(sr);
}

void PD_I2cOverVdmTimer1msIsr(pd_i2c_over_vdm_handle i2cOvdmHandle)
{
    i2c_over_vdm_state_t *i2cOvdmState = (i2c_over_vdm_state_t *)i2cOvdmHandle;

    if (i2cOvdmHandle == NULL)
    {
        return;
    }

    if (i2cOvdmState->timerValue > 0)
    {
        i2cOvdmState->timerValue--;
    }
    else
    {
        i2cOvdmState->waitTimerOut = 1u;
    }
}

static void I2cVdmInit(i2c_over_vdm_state_t *i2cOvdmState)
{
    i2cOvdmState->I2CoVDMLastTxRxResult = 1u;
    i2cOvdmState->I2CoVDMState = I2CoVDM_Idle;
    i2cOvdmState->I2CoVDMCurrIndex = 0;
    i2cOvdmState->I2CoVDMTxBufEnd = 0;
    i2cOvdmState->I2CoVDMRxBufEnd = 0;
    i2cOvdmState->i2covdmhdr.curr_seq = 0;
    i2cOvdmState->i2covdmhdr.total_seq = 0;
    i2cOvdmState->callbackFn = NULL;
    i2cOvdmState->callbackParam = NULL;
    i2cOvdmState->cmsisHandle = NULL;
}

static void I2cVdmSetupPDO(i2c_over_vdm_state_t *i2cOvdmState)
{
    pd_unstructured_vdm_header_t unstructuredVDMHeader;

    unstructuredVDMHeader.bitFields.SVID = NXP_VENDOR_ID;
    unstructuredVDMHeader.bitFields.vdmType = 0;
    unstructuredVDMHeader.bitFields.vendorUse = I2C_OVER_VDM_TAG;
    i2cOvdmState->newSendMsg[0] = unstructuredVDMHeader.unstructuredVdmHeaderVal;
}

static void I2cVdmSetupPDONormalPayload(i2c_over_vdm_state_t *i2cOvdmState)
{
    I2CoVDM_cmd_hdr_t *i2covdmhdr = &i2cOvdmState->i2covdmhdr;
    uint32_t remaining_bytes = i2cOvdmState->I2CoVDMTxBufEnd - i2cOvdmState->I2CoVDMCurrIndex;

    i2covdmhdr->curr_seq = (i2cOvdmState->I2CoVDMCurrIndex / I2C_VDM_MAX_PAYLOAD) + 1;
    if (remaining_bytes > I2C_VDM_MAX_PAYLOAD)
    {
        i2covdmhdr->length = I2C_VDM_MAX_PAYLOAD;
    }
    else /* the last packet */
    {
        i2covdmhdr->length = remaining_bytes;
        /* update cmd to show its the last packet */
        if (i2covdmhdr->cmd == I2COVDM_CMD_WRITE_REQ)
        {
            i2covdmhdr->cmd = I2COVDM_CMD_WRITE_COMPLETE;
        }
        else if (i2covdmhdr->cmd == I2COVDM_CMD_RESP_ACK_START)
        {
            i2covdmhdr->cmd = I2COVDM_CMD_RESP_ACK_COMPLETE;
        }
        else
        {
        }
    }

    /* Copy the header into the first 2 bytes of the first VDO */
    memcpy((void *)(&i2cOvdmState->newSendMsg[1]), i2covdmhdr, 2);
    /* Copy the rest of the payload (max 22 bytes) into the rest of the VDOs */
    memcpy((void *)((uint8_t *)(&i2cOvdmState->newSendMsg[1]) + 2),
           &i2cOvdmState->i2cOvdmCommonBuf[i2cOvdmState->I2CoVDMCurrIndex], i2covdmhdr->length);
    /* Zero out the rest of the buffer so we don't send garbage */
    if (i2covdmhdr->length < I2C_VDM_MAX_PAYLOAD)
    {
        memset((void *)((uint8_t *)(&i2cOvdmState->newSendMsg[1]) + 2 + i2covdmhdr->length), 0,
               I2C_VDM_MAX_PAYLOAD - i2covdmhdr->length);
    }
}

void PD_I2cOverVdmTask(void *arg)
{
    i2c_over_vdm_state_t *i2cOvdmState = (i2c_over_vdm_state_t *)arg;
    uint8_t length;
    uint16_t localcmd;
    uint8_t *I2CoVDMPayLoad;
    uint8_t needTX = 0;

    if (i2cOvdmState->newReceived)
    {
        I2CoVDM_cmd_hdr_t *pkgi2cOverVdmHdr = (I2CoVDM_cmd_hdr_t *)&(i2cOvdmState->newReceivedMsg[1]);
        I2CoVDMPayLoad = (uint8_t *)&(i2cOvdmState->newReceivedMsg[1]);
        I2CoVDMPayLoad += 2;
        i2cOvdmState->newReceived = 0u;

        switch (i2cOvdmState->I2CoVDMState)
        {
            case I2CoVDM_SlaveSendRespWaitAck: /* slave */
            case I2CoVDM_MasterWaitAck:        /* master */
                if ((pkgi2cOverVdmHdr->cmd == I2COVDM_CMD_STATUS) && (I2CoVDMPayLoad[0] == I2COVDM_STAT_ACK))
                {
                    /* ack received successfully, send next packets later in transmit logic below
                     * change internal seq counter, NOT the i2covmhdr in the rxbuf */
                    if (i2cOvdmState->I2CoVDMState == I2CoVDM_SlaveSendRespWaitAck)
                    {
                        /* slave behaviour */
                        i2cOvdmState->I2CoVDMState = I2CoVDM_SlaveSendResp;
                    }
                    else
                    {
                        /* host behaviour */
                        i2cOvdmState->I2CoVDMState = I2CoVDM_MasterAckReceived;
                    }
                }
                else
                {
                    /* received something out of order from slave, reset the statemachine */
                    I2cVdmInit(i2cOvdmState);
                    if (i2cOvdmState->callbackFn != NULL)
                    {
                        i2cOvdmState->callbackFn(i2cOvdmState->callbackParam, i2cOvdmState->I2CoVDMLastTxRxResult);
                    }
                }
                break;

            case I2CoVDM_MasterWaitResp: /* master */
                memcpy(&i2cOvdmState->i2cOvdmCommonBuf[(pkgi2cOverVdmHdr->curr_seq - 1) * I2C_VDM_MAX_PAYLOAD],
                       I2CoVDMPayLoad, pkgi2cOverVdmHdr->length);
                if (pkgi2cOverVdmHdr->curr_seq == pkgi2cOverVdmHdr->total_seq)
                {
                    i2cOvdmState->I2CoVDMRxBufEnd =
                        (pkgi2cOverVdmHdr->curr_seq - 1) * I2C_VDM_MAX_PAYLOAD + pkgi2cOverVdmHdr->length;
                    /* no more VDMs for this i2c transaction
                     * set back to idle state, transaction finished for this slave
                     * when USB driver in context, will see that i2covdm state is idle and check rxbufend counter
                     * and read out the contents accordingly */
                    if ((pkgi2cOverVdmHdr->cmd == I2COVDM_CMD_RESP_NACK) ||
                        ((pkgi2cOverVdmHdr->cmd == I2COVDM_CMD_RESP_ACK_COMPLETE) && (pkgi2cOverVdmHdr->length == 1) &&
                         (I2CoVDMPayLoad[0] == I2COVDM_STAT_ERR)))
                    {
                        i2cOvdmState->I2CoVDMLastTxRxResult = 1u;
                        if (pkgi2cOverVdmHdr->cmd == I2COVDM_CMD_RESP_NACK)
                        {
                            i2cOvdmState->I2CoVDMRxBufEnd = 0;
                        }
                    }
                    else
                    {
                        i2cOvdmState->I2CoVDMLastTxRxResult = 0u;
                        i2cOvdmState->I2CoVDMState = I2CoVDM_Idle;
                    }
                    memcpy(i2cOvdmState->readDataBuffer, &i2cOvdmState->i2cOvdmCommonBuf[0],
                           (pkgi2cOverVdmHdr->curr_seq - 1) * I2C_VDM_MAX_PAYLOAD + pkgi2cOverVdmHdr->length);
                    i2cOvdmState->I2CoVDMState = I2CoVDM_Idle;
                    i2cOvdmState->I2CoVDMCurrIndex = 0;
                    i2cOvdmState->I2CoVDMTxBufEnd = 0;
                    if (i2cOvdmState->callbackFn != NULL)
                    {
                        i2cOvdmState->callbackFn(i2cOvdmState->callbackParam, i2cOvdmState->I2CoVDMLastTxRxResult);
                    }
                }
                else
                {
                    /* send ack and wait for more packets */
                    i2cOvdmState->I2CoVDMState = I2CoVDM_MasterWaitRspSendAck;
                }
                break;

            case I2CoVDM_WaitVDMSendCallback: /* reset to process new package */
            case I2CoVDM_Idle:
                memcpy(&i2cOvdmState->i2cOvdmCommonBuf[(pkgi2cOverVdmHdr->curr_seq - 1) * I2C_VDM_MAX_PAYLOAD],
                       I2CoVDMPayLoad, pkgi2cOverVdmHdr->length);

                if (pkgi2cOverVdmHdr->curr_seq == pkgi2cOverVdmHdr->total_seq)
                {
                    /* i2cOvdmState->I2CoVDMRxLen = (i2cOverVdmHdr->curr_seq - 1) * I2C_VDM_MAX_PAYLOAD +
                     * i2cOverVdmHdr->length; */

                    switch (pkgi2cOverVdmHdr->cmd)
                    {
                        case I2COVDM_CMD_READ_REQ:
                            /* retrieve the length from the first packet */
                            length = i2cOvdmState->i2cOvdmCommonBuf[I2C_VDM_LENGTH_OFFSET];
                            localcmd = i2cOvdmState->i2cOvdmCommonBuf[I2C_VDM_CMDADDR_HB_OFFSET];
                            localcmd = localcmd << 8;
                            localcmd = localcmd | i2cOvdmState->i2cOvdmCommonBuf[I2C_VDM_CMDADDR_LB_OFFSET];
                            if (!CMSIS_PortControlInterfaceReadRegister(
                                    i2cOvdmState->cmsisHandle, i2cOvdmState->i2cOvdmCommonBuf[I2C_VDM_SLAVEADDR_OFFSET],
                                    i2cOvdmState->i2cOvdmCommonBuf[I2C_VDM_CMDADDR_HB_OFFSET] |
                                        (i2cOvdmState->i2cOvdmCommonBuf[I2C_VDM_CMDADDR_LB_OFFSET] << 8),
                                    2, i2cOvdmState->i2cOvdmCommonBuf, length))
                            {
                                /* send i2c result back to i2c master over vdm */
                                if (localcmd == 0x900)
                                {
                                    i2cOvdmState->i2cOvdmCommonBuf[2] = i2cOvdmState->i2cOvdmCommonBuf[1];
                                    i2cOvdmState->i2cOvdmCommonBuf[1] = 0;
                                }
                                /* init i2covdm header */
                                i2cOvdmState->i2covdmhdr.total_seq =
                                    (length + I2C_VDM_MAX_PAYLOAD - 1) / I2C_VDM_MAX_PAYLOAD;
                                i2cOvdmState->i2covdmhdr.curr_seq = 1;
                                if (i2cOvdmState->i2covdmhdr.total_seq > 1)
                                {
                                    i2cOvdmState->i2covdmhdr.cmd = I2COVDM_CMD_RESP_ACK_START;
                                }
                                else
                                {
                                    i2cOvdmState->i2covdmhdr.cmd = I2COVDM_CMD_RESP_ACK_COMPLETE;
                                }
                                /* total length */
                                i2cOvdmState->I2CoVDMTxBufEnd = length;
                                i2cOvdmState->I2CoVDMCurrIndex = 0;
                            }
                            else /* fail */
                            {
                                /* prepare data */
                                i2cOvdmState->i2cOvdmCommonBuf[0] = I2COVDM_STAT_ERR;
                                /* init i2covdm header */
                                i2cOvdmState->i2covdmhdr.total_seq = 1;
                                i2cOvdmState->i2covdmhdr.curr_seq = 1;
                                i2cOvdmState->i2covdmhdr.cmd = I2COVDM_CMD_RESP_NACK;
                                /* total length */
                                i2cOvdmState->I2CoVDMTxBufEnd = 1;
                                i2cOvdmState->I2CoVDMCurrIndex = 0;
                            }
                            i2cOvdmState->I2CoVDMState = I2CoVDM_SlaveSendResp;
                            break;

                        case I2COVDM_CMD_WRITE_REQ:
                        case I2COVDM_CMD_WRITE_COMPLETE:
                        case I2COVDM_CMD_EXTENDED:
                            length =
                                i2cOvdmState->i2cOvdmCommonBuf[I2C_VDM_WRITE_LENGTH_OFFSET] + I2C_VDM_WRITE_HEADER_SIZE;
                            if (pkgi2cOverVdmHdr->cmd == I2COVDM_CMD_EXTENDED)
                            {
                                /* for special cmds, no 2 byte offsetaaddr, single byte payload */
                                uint8_t extCmd = i2cOvdmState->i2cOvdmCommonBuf[I2C_VDM_WRITE_SLAVEADDR_OFFSET];
                                if (extCmd == I2COVDM_EXT_CMD_AUTH_STATUS)
                                {
                                    /* prepare data */
                                    i2cOvdmState->i2cOvdmCommonBuf[0] = I2COVDM_STAT_ACK;
                                    /* init i2covdm header */
                                    i2cOvdmState->i2covdmhdr.total_seq = 1;
                                    i2cOvdmState->i2covdmhdr.curr_seq = 1;
                                    i2cOvdmState->i2covdmhdr.cmd = I2COVDM_CMD_RESP_ACK_COMPLETE;
                                    /* total length */
                                    i2cOvdmState->I2CoVDMTxBufEnd = 1;
                                    i2cOvdmState->I2CoVDMCurrIndex = 0;

                                    i2cOvdmState->I2CoVDMState = I2CoVDM_SlaveSendResp;
                                    break;
                                }
                                else if (extCmd == I2COVDM_EXT_CMD_SPECIAL)
                                {
                                    length = 1;
                                }
                            }

                            /* prepare data */
                            if (!CMSIS_PortControlInterfaceWriteRegister(
                                    i2cOvdmState->cmsisHandle,
                                    i2cOvdmState->i2cOvdmCommonBuf[I2C_VDM_WRITE_SLAVEADDR_OFFSET], 0, 0,
                                    &i2cOvdmState->i2cOvdmCommonBuf[I2C_VDM_WRITE_PAYLOAD_OFFSET], length))
                            {
                                /* transmit success */
                                i2cOvdmState->i2cOvdmCommonBuf[0] = I2COVDM_STAT_ACK;
                            }
                            else
                            {
                                i2cOvdmState->i2cOvdmCommonBuf[0] = I2COVDM_STAT_ERR;
                            }
                            /* init i2covdm header */
                            i2cOvdmState->i2covdmhdr.total_seq = 1;
                            i2cOvdmState->i2covdmhdr.curr_seq = 1;
                            i2cOvdmState->i2covdmhdr.cmd = I2COVDM_CMD_RESP_ACK_COMPLETE;
                            /* total length */
                            i2cOvdmState->I2CoVDMTxBufEnd = 1;
                            i2cOvdmState->I2CoVDMCurrIndex = 0;
                            i2cOvdmState->I2CoVDMState = I2CoVDM_SlaveSendResp;
                            break;

                        default:
                            break;
                    }
                }
                else
                {
                    /* send ack and wait for more packets
                     * offset at next position */
                    i2cOvdmState->I2CoVDMState = I2CoVDM_SlaveSendAck;
                }
                break;

            default:
                break;
        }
    }

    switch (i2cOvdmState->I2CoVDMState)
    {
        case I2CoVDM_Idle:
            break;

        case I2CoVDM_MasterStart:
        case I2CoVDM_MasterAckReceived:
        case I2CoVDM_SlaveSendResp:
            if (i2cOvdmState->I2CoVDMTxBufEnd > i2cOvdmState->I2CoVDMCurrIndex)
            {
                /* send next command if available */
                I2cVdmSetupPDO(i2cOvdmState);
                I2cVdmSetupPDONormalPayload(i2cOvdmState);
                needTX = 1;
            }
            else
            {
                i2cOvdmState->I2CoVDMState = I2CoVDM_Idle;
            }
            break;

        case I2CoVDM_MasterWaitRspSendAck:
        case I2CoVDM_SlaveSendAck:
            /* init i2covdm header */
            i2cOvdmState->i2covdmhdr.total_seq = 1;
            i2cOvdmState->i2covdmhdr.curr_seq = 1;
            i2cOvdmState->i2covdmhdr.length = 1;
            i2cOvdmState->i2covdmhdr.cmd = I2COVDM_CMD_STATUS;
            /* total length */
            i2cOvdmState->I2CoVDMTxBufEnd = 1;
            i2cOvdmState->I2CoVDMCurrIndex = 0;
            /* prepare data */
            I2cVdmSetupPDO(i2cOvdmState);
            ((uint8_t *)&(i2cOvdmState->newSendMsg[1]))[0] = ((uint8_t *)&(i2cOvdmState->i2covdmhdr))[0];
            ((uint8_t *)&(i2cOvdmState->newSendMsg[1]))[1] = ((uint8_t *)&(i2cOvdmState->i2covdmhdr))[1];
            ((uint8_t *)&(i2cOvdmState->newSendMsg[1]))[2] = I2COVDM_STAT_ACK;
            ((uint8_t *)&(i2cOvdmState->newSendMsg[1]))[3] = 0u;
            needTX = 1;
            break;

        case I2CoVDM_WaitVDMSendCallback:
            if (!(i2cOvdmState->sending))
            {
                if (i2cOvdmState->sendResult)
                {
                    /* update the transfered size, below will use this to judge for state machine */
                    i2cOvdmState->I2CoVDMCurrIndex = i2cOvdmState->I2CoVDMCurrIndex + i2cOvdmState->i2covdmhdr.length;
                    /* reply */
                    if (i2cOvdmState->I2CoVDMWaitSendPreState == I2CoVDM_SlaveSendAck)
                    {
                        /* wait for more incoming packets */
                        i2cOvdmState->I2CoVDMState = I2CoVDM_Idle;
                    }
                    else if (i2cOvdmState->I2CoVDMWaitSendPreState == I2CoVDM_MasterWaitRspSendAck)
                    {
                        /* for host, wait for more incoming packets in WaitResp state */
                        i2cOvdmState->I2CoVDMState = I2CoVDM_MasterWaitResp;
                        i2cOvdmState->waitTimerOut = 0;
                        PD_I2cOverVdmTimer_Start(i2cOvdmState, NXP_VDM_SENDER_RESPONSE);
                    }
                    else /* normal data */
                    {
                        if (i2cOvdmState->I2CoVDMTxBufEnd <= i2cOvdmState->I2CoVDMCurrIndex)
                        {
                            if (i2cOvdmState->I2CoVDMWaitSendPreState == I2CoVDM_SlaveSendResp)
                            {
                                /* slave behaviour */
                                i2cOvdmState->I2CoVDMState = I2CoVDM_Idle;
                            }
                            else
                            {
                                /* host behavoiur */
                                i2cOvdmState->I2CoVDMState = I2CoVDM_MasterWaitResp;
                                i2cOvdmState->waitTimerOut = 0;
                                PD_I2cOverVdmTimer_Start(i2cOvdmState, NXP_VDM_SENDER_RESPONSE);
                            }
                        }
                        else
                        {
                            /* more to send, increase packet count, and wait for ack */
                            if (i2cOvdmState->I2CoVDMWaitSendPreState == I2CoVDM_SlaveSendResp)
                            {
                                /* slave behaviour */
                                i2cOvdmState->I2CoVDMState = I2CoVDM_SlaveSendRespWaitAck;
                            }
                            else
                            {
                                /* host behaviour */
                                i2cOvdmState->I2CoVDMState = I2CoVDM_MasterWaitAck;
                            }
                            i2cOvdmState->waitTimerOut = 0;
                            PD_I2cOverVdmTimer_Start(i2cOvdmState, NXP_VDM_SENDER_RESPONSE);
                        }
                    }
                }
                else
                {
                    i2cOvdmState->I2CoVDMState = I2CoVDM_Idle;
                    if (i2cOvdmState->callbackFn != NULL)
                    {
                        i2cOvdmState->callbackFn(i2cOvdmState->callbackParam, i2cOvdmState->I2CoVDMLastTxRxResult);
                    }
                }
            }
            break;

        default:
            needTX = 0;
            break;
    }

    if (needTX)
    {
        pd_unstructured_vdm_command_param_t unstructuredVDMCommandParam;

        unstructuredVDMCommandParam.vdmSop = kPD_MsgSOP;
        unstructuredVDMCommandParam.vdmHeaderAndVDOsCount = (i2cOvdmState->i2covdmhdr.length + 2 + 4 + 3) / 4;
        unstructuredVDMCommandParam.vdmHeaderAndVDOsData = (uint32_t *)&i2cOvdmState->newSendMsg[0];

        i2cOvdmState->sending = 1u;
        i2cOvdmState->sendResult = 0u;
        if (PD_Command(i2cOvdmState->pdHandle, PD_DPM_SEND_UNSTRUCTURED_VDM, &unstructuredVDMCommandParam) !=
            kStatus_PD_Success)
        {
            i2cOvdmState->sending = 0u;
            PRINTF("command fail\r\n");
            I2cVdmInit(i2cOvdmState);
            if (i2cOvdmState->callbackFn != NULL)
            {
                i2cOvdmState->callbackFn(i2cOvdmState->callbackParam, i2cOvdmState->I2CoVDMLastTxRxResult);
            }
        }
        else
        {
            i2cOvdmState->I2CoVDMWaitSendPreState = i2cOvdmState->I2CoVDMState;
            i2cOvdmState->I2CoVDMState = I2CoVDM_WaitVDMSendCallback;
            i2cOvdmState->waitTimerOut = 0;
            PD_I2cOverVdmTimer_Start(i2cOvdmState, NXP_VDM_SENDER_RESPONSE);
        }
    }

    /* Handle a timeout while waiting for a response */
    switch (i2cOvdmState->I2CoVDMState)
    {
        case I2CoVDM_WaitVDMSendCallback:
        case I2CoVDM_SlaveSendRespWaitAck:
        case I2CoVDM_MasterWaitAck:  // wait for ack to send further packets, both i2cmaster and slave uses this
        case I2CoVDM_MasterWaitResp: // i2cmaster only
            if (i2cOvdmState->waitTimerOut)
            {
                i2cOvdmState->I2CoVDMLastTxRxResult = 1u;
                i2cOvdmState->I2CoVDMRxBufEnd = 0;
                i2cOvdmState->I2CoVDMState = I2CoVDM_Idle;
                i2cOvdmState->I2CoVDMCurrIndex = 0;
                i2cOvdmState->I2CoVDMTxBufEnd = 0;
                if (i2cOvdmState->callbackFn != NULL)
                {
                    i2cOvdmState->callbackFn(i2cOvdmState->callbackParam, i2cOvdmState->I2CoVDMLastTxRxResult);
                }
            }
            break;
    }
}

static void PD_I2cOverVdmReceivedMsg(pd_i2c_over_vdm_handle i2cOvdmHandle,
                                     pd_unstructured_vdm_command_param_t *unstructuredVDMParam)
{
    i2c_over_vdm_state_t *i2cOvdmState = (i2c_over_vdm_state_t *)i2cOvdmHandle;
    pd_unstructured_vdm_header_t unstructuredVDMHeader;

    if ((unstructuredVDMParam->vdmHeaderAndVDOsData == NULL) || (unstructuredVDMParam->vdmHeaderAndVDOsCount == 0))
    {
        return;
    }
    unstructuredVDMHeader.unstructuredVdmHeaderVal = unstructuredVDMParam->vdmHeaderAndVDOsData[0];

    /* I2C over VDM package */
    if ((unstructuredVDMHeader.bitFields.SVID == NXP_VENDOR_ID) && (!(unstructuredVDMHeader.bitFields.vdmType)) &&
        (I2C_OVER_VDM_TAG == unstructuredVDMHeader.bitFields.vendorUse))
    {
        for (uint8_t index = 0; (index < unstructuredVDMParam->vdmHeaderAndVDOsCount) && index < 7; ++index)
        {
            i2cOvdmState->newReceivedMsg[index] = unstructuredVDMParam->vdmHeaderAndVDOsData[index];
        }
        i2cOvdmState->newReceived = 1;
    }
}

static void PD_I2cOverVdmSendDone(pd_i2c_over_vdm_handle i2cOvdmHandle, uint8_t success)
{
    i2c_over_vdm_state_t *i2cOvdmState = (i2c_over_vdm_state_t *)i2cOvdmHandle;

    i2cOvdmState->sendResult = success;
    i2cOvdmState->sending = 0;
}

static pd_status_t PD_I2cOverVdmCallback(void *callbackParam, uint32_t event, void *param)
{
    if (event == PD_DPM_UNSTRUCTURED_VDM_RECEIVED)
    {
        PD_I2cOverVdmReceivedMsg((pd_i2c_over_vdm_handle)callbackParam, (pd_unstructured_vdm_command_param_t *)param);
    }
    else if (event == PD_DPM_SEND_UNSTRUCTURED_VDM_SUCCESS)
    {
        PD_I2cOverVdmSendDone((pd_i2c_over_vdm_handle)callbackParam, 1);
    }
    else if (event == PD_DPM_SEND_UNSTRUCTURED_VDM_FAIL)
    {
        PD_I2cOverVdmSendDone((pd_i2c_over_vdm_handle)callbackParam, 0);
    }
    else
    {
    }

    return kStatus_PD_Success;
}

uint8_t PD_I2cOverVdmInit(pd_i2c_over_vdm_handle *i2cOvdmHandle, pd_i2c_over_vdm_config_t *config)
{
    uint8_t index;
    for (index = 0; index < sizeof(s_i2cOverVdmState) / sizeof(i2c_over_vdm_state_t); ++index)
    {
        if (s_i2cOverVdmState[index].occupied == 0)
        {
            break;
        }
    }
    if (index >= sizeof(s_i2cOverVdmState) / sizeof(i2c_over_vdm_state_t))
    {
        return 1;
    }

    I2cVdmInit(&s_i2cOverVdmState[index]);
    s_i2cOverVdmState[index].pdHandle = config->pdHandle;
    s_i2cOverVdmState[index].callbackFn = config->callback;
    s_i2cOverVdmState[index].callbackParam = config->callbackParam;
    s_i2cOverVdmState[index].cmsisHandle = config->cmsisHandle;
    s_i2cOverVdmState[index].i2cSlaveAddress = config->i2cSlaveAddress;

    PD_UnstructuredVDMSetCallback(config->pdHandle, PD_I2cOverVdmCallback, &s_i2cOverVdmState[index]);

    *i2cOvdmHandle = &s_i2cOverVdmState[index];
    return 0;
}

uint8_t PD_I2cOverVdmDeinit(pd_i2c_over_vdm_handle i2cOvdmHandle)
{
    i2c_over_vdm_state_t *i2cOvdmState = (i2c_over_vdm_state_t *)i2cOvdmHandle;

    i2cOvdmState->occupied = 0;
    return 0;
}

uint8_t PD_I2cOverVdmRead(pd_i2c_over_vdm_handle i2cOvdmHandle, uint16_t offset, uint8_t *buff, uint8_t length)
{
    i2c_over_vdm_state_t *i2cOvdmState = (i2c_over_vdm_state_t *)i2cOvdmHandle;

    if (i2cOvdmState->I2CoVDMState != I2CoVDM_Idle)
    {
        return 1;
    }
    i2cOvdmState->readDataBuffer = buff;
    i2cOvdmState->I2CoVDMRxBufEnd = 0;
    i2cOvdmState->I2CoVDMTxBufEnd = 0;
    i2cOvdmState->I2CoVDMCurrIndex = 0;
    i2cOvdmState->i2cOvdmCommonBuf[0] = length;
    i2cOvdmState->i2cOvdmCommonBuf[1] = i2cOvdmState->i2cSlaveAddress;
    i2cOvdmState->i2cOvdmCommonBuf[2] = (uint8_t)(offset >> 8);
    i2cOvdmState->i2cOvdmCommonBuf[3] = (uint8_t)(offset);
    i2cOvdmState->i2covdmhdr.cmd = I2COVDM_CMD_READ_REQ;
    i2cOvdmState->i2covdmhdr.total_seq = 1;
    i2cOvdmState->i2covdmhdr.curr_seq = 1;
    i2cOvdmState->i2covdmhdr.length = I2COVDM_READ_HDR; // payload is 4
    i2cOvdmState->I2CoVDMTxBufEnd = I2COVDM_READ_HDR;
    i2cOvdmState->I2CoVDMLastTxRxResult = 1u;
    i2cOvdmState->I2CoVDMState = I2CoVDM_MasterStart;
    return 0;
}

uint8_t PD_I2cOverVdmWrite(pd_i2c_over_vdm_handle i2cOvdmHandle,
                           uint16_t offset,
                           const uint8_t *write_buffer,
                           uint8_t length)
{
    i2c_over_vdm_state_t *i2cOvdmState = (i2c_over_vdm_state_t *)i2cOvdmHandle;
    uint8_t totallength = 4 + length;

    if (i2cOvdmState->I2CoVDMState != I2CoVDM_Idle)
    {
        return 1;
    }
    i2cOvdmState->I2CoVDMRxBufEnd = 0;
    i2cOvdmState->I2CoVDMTxBufEnd = 0;
    i2cOvdmState->I2CoVDMCurrIndex = 0;
    i2cOvdmState->i2cOvdmCommonBuf[0] = i2cOvdmState->i2cSlaveAddress;
    i2cOvdmState->i2cOvdmCommonBuf[1] = (uint8_t)(offset >> 8);
    i2cOvdmState->i2cOvdmCommonBuf[2] = (uint8_t)(offset);
    i2cOvdmState->i2cOvdmCommonBuf[3] = length;
    memcpy((void *)&i2cOvdmState->i2cOvdmCommonBuf[4], write_buffer, length);
    i2cOvdmState->i2covdmhdr.cmd = I2COVDM_CMD_WRITE_REQ;
    i2cOvdmState->i2covdmhdr.total_seq = (totallength + I2C_VDM_MAX_PAYLOAD - 1) / I2C_VDM_MAX_PAYLOAD;
    if (i2cOvdmState->i2covdmhdr.total_seq > 1)
    {
        i2cOvdmState->i2covdmhdr.length = I2C_VDM_MAX_PAYLOAD;
    }
    else
    {
        i2cOvdmState->i2covdmhdr.length = totallength;
    }

    i2cOvdmState->i2covdmhdr.curr_seq = 1;
    i2cOvdmState->I2CoVDMTxBufEnd = totallength;
    i2cOvdmState->I2CoVDMLastTxRxResult = 1u;
    i2cOvdmState->I2CoVDMState = I2CoVDM_MasterStart;
    return 0;
}
