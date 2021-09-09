/*
 * The Clear BSD License
 * Copyright 2017 NXP
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

#ifndef __PD_I2C_OVER_VDM_H__
#define __PD_I2C_OVER_VDM_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*********** package related structures and MACROs ***********/

/** Value TBD - Tag to identify this packet as I2C over VDM */
#define I2C_OVER_VDM_TAG 0x1006

/* offsets for read_req */
#define I2C_VDM_LENGTH_OFFSET 0
#define I2C_VDM_SLAVEADDR_OFFSET 1
#define I2C_VDM_CMDADDR_HB_OFFSET 2
#define I2C_VDM_CMDADDR_LB_OFFSET 3

/* offsets for  write_req */
#define I2C_VDM_WRITE_SLAVEADDR_OFFSET 0
#define I2C_VDM_WRITE_PAYLOAD_OFFSET 1
#define I2C_VDM_WRITE_LENGTH_OFFSET 3
#define I2C_VDM_WRITE_HEADER_SIZE 3

// VDM packet defines
#define I2C_VDM_MAX_PAYLOAD 22

/** Command field for i2covdm_cmd_hdr_t.cmd
*/
enum i2covdm_cmd
{
    /* Master->Slave */
    I2COVDM_CMD_WRITE_REQ = 0,      /* master start write */
    I2COVDM_CMD_READ_REQ = 1,       /* master start read */
    I2COVDM_CMD_WRITE_COMPLETE = 2, /* if write contain multiple packages, last one use this. */
    I2COVDM_CMD_EXTENDED = 3,       /* special commands that have to be processed per application */
    /* Slave->Master */
    I2COVDM_CMD_RESP_ACK_START =
        4, /* if slave response for multiple packages, first one and middle ones ned use this */
    I2COVDM_CMD_RESP_ACK_COMPLETE = 6, /* slave reponse for write/read, last response need use this. */
    I2COVDM_CMD_RESP_NACK = 5,         /* salve NACK for write/read */
    /* Master<->Slave */
    I2COVDM_CMD_STATUS = 7, /* (1) write multiple packages; (2) read multiple packages. */
};

typedef enum i2covdm_extendedCmd
{
    I2COVDM_EXT_CMD_SPECIAL = 0,
    I2COVDM_EXT_CMD_AUTH_STATUS = 1,
} I2CoVDM_extendedCmd_t;

enum i2covdm_status
{
    I2COVDM_STAT_ACK = 0,
    I2COVDM_STAT_ERR = 1,
};

/** Packet header for i2c over vdm packet .
- Located in VDM PDO 1
*/
typedef struct
{
    uint8_t total_seq : 4; /* bits   3:0 : Total index of this packet payload */
    uint8_t curr_seq : 4;  /* bits   7:4 : Current index of this packet payload */
    uint8_t length : 5;    /* bits  12:8 : Length of this packet payload in bytes */
    uint8_t cmd : 3;       /* bits 15:13 : Command for this payload */
} I2CoVDM_cmd_hdr_t;

/* i2c over vdm transfer callback.
 * result: 0 - success; other values - fail;
 */
typedef void (*pd_i2c_over_vdm_callback_t)(void *callbackParam, uint32_t result);

/*********** SVDM process related ***********/

#define I2COVDM_READ_HDR 4
#define I2COVDM_WRITE_HDR 4
#define I2COVDM_QUICKCMD_HDR 1

#define I2C_OVER_VDM_SLAVE_SUPPORT (1)
#define I2C_OVER_VDM_MASTER_SUPPORT (1)

/************************** From Master ***********************/
/* Master start write (short):
 * state machine:
 * I2CoVDM_MasterStart -> I2CoVDM_MasterWaitResp -> I2CoVDM_Idle
 * CMD sequence:
 * (m)I2COVDM_CMD_WRITE_REQ -> (s)I2COVDM_CMD_RESP_ACK_COMPLETE or I2COVDM_CMD_RESP_NACK -> done
 */

/* Master start write (long):
 * state machine:
 * {I2CoVDM_MasterStart/I2CoVDM_MasterAckReceived -> I2CoVDM_MasterWaitAck} ->
 * I2CoVDM_MasterAckReceived -> I2CoVDM_MasterWaitResp -> I2CoVDM_Idle
 * CMD sequence:
 * {(m)I2COVDM_CMD_WRITE_REQ -> (s)I2COVDM_CMD_STATUS} ->
 * (m)I2COVDM_CMD_WRITE_COMPLETE -> (s)I2COVDM_CMD_RESP_NACK or I2COVDM_CMD_RESP_ACK_COMPLETE -> done
 */

/* Master start read (short):
 * state machine:
 * I2CoVDM_MasterStart -> I2CoVDM_MasterWaitResp -> I2CoVDM_Idle
 * CMD sequence:
 * (m)I2COVDM_CMD_READ_REQ -> (s)I2COVDM_CMD_RESP_ACK_COMPLETE or I2COVDM_CMD_RESP_NACK -> done
 */

/* Master start read (long):
 * state machine:
 * I2CoVDM_MasterStart -> {I2CoVDM_MasterWaitResp -> I2CoVDM_MasterWaitRspSendAck} -> I2CoVDM_MasterWaitResp ->
 * I2CoVDM_Idle
 * CMD sequence:
 * (m)I2COVDM_CMD_READ_REQ -> {(s)I2COVDM_CMD_RESP_ACK_START -> (m)I2COVDM_CMD_STATUS} ->
 * (s)I2COVDM_CMD_RESP_ACK_COMPLETE -> done
 */

/************************** From Slave ***********************/
/* Master start write (short):
 * state machine:
 * I2CoVDM_Idle -> I2CoVDM_SlaveSendResp -> I2CoVDM_Idle
 * CMD sequence:
 * (m)I2COVDM_CMD_WRITE_REQ -> (s)I2COVDM_CMD_RESP_ACK_COMPLETE or I2COVDM_CMD_RESP_NACK -> done
 */

/* Master start write (long):
 * state machine:I2CoVDM_Idle
 * {I2CoVDM_Idle -> I2CoVDM_SlaveSendAck} ->
 * I2CoVDM_Idle -> I2CoVDM_SlaveSendResp -> I2CoVDM_Idle
 * CMD sequence:
 * {(m)I2COVDM_CMD_WRITE_REQ -> (s)I2COVDM_CMD_STATUS} ->
 * (m)I2COVDM_CMD_WRITE_COMPLETE -> (s)I2COVDM_CMD_RESP_NACK or I2COVDM_CMD_RESP_ACK_COMPLETE -> done
 */

/* Master start read (short):
 * state machine:
 * I2CoVDM_Idle -> I2CoVDM_SlaveSendResp -> I2CoVDM_Idle
 * CMD sequence:
 * (m)I2COVDM_CMD_READ_REQ -> (s)I2COVDM_CMD_RESP_ACK_COMPLETE or I2COVDM_CMD_RESP_NACK -> done
 */

/* Master start read (long):
 * state machine:
 * I2CoVDM_Idle -> {I2CoVDM_SlaveSendResp -> I2CoVDM_SlaveSendRespWaitAck} -> I2CoVDM_SlaveSendResp -> I2CoVDM_Idle
 * CMD sequence:
 * (m)I2COVDM_CMD_READ_REQ -> {(s)I2COVDM_CMD_RESP_ACK_START -> (m)I2COVDM_CMD_STATUS} ->
 * (s)I2COVDM_CMD_RESP_ACK_COMPLETE -> done
 */

typedef enum _i2cOvdm_state
{
    I2CoVDM_Idle,

    /* slave */
    I2CoVDM_SlaveSendResp,        /* send response for I2C read */
    I2CoVDM_SlaveSendRespWaitAck, /* wait host reply ACK for previouse resp package */
    I2CoVDM_SlaveSendAck,         /* slave receive the written data */

    /* master */
    I2CoVDM_MasterStart,    /* start I2C read or write */
    I2CoVDM_MasterWaitResp, /* wait response for I2C read or write */
    I2CoVDM_MasterWaitAck,  /*  */
    I2CoVDM_MasterAckReceived,
    I2CoVDM_MasterWaitRspSendAck, /* master send STATUS (ACK) for response */

    I2CoVDM_WaitVDMSendCallback,
} i2cOvdm_state_t;

/*control structure for I2C over VDM device, to be replicated for each I2C application over VDM*/
typedef struct _i2c_over_vdm_state
{
    I2CoVDM_cmd_hdr_t i2covdmhdr;
    uint32_t newReceivedMsg[7];
    uint32_t newSendMsg[7];
    uint8_t i2cOvdmCommonBuf[128 + 4];
    uint8_t *readDataBuffer;
    pd_i2c_over_vdm_callback_t callbackFn;
    void *callbackParam;
    usb_cmsis_wrapper_handle cmsisHandle;
    pd_handle pdHandle;
    volatile uint32_t timerValue;

    uint8_t occupied; /* this instance is used or not */
    uint8_t i2cSlaveAddress;
    uint8_t I2CoVDMState;
    uint8_t I2CoVDMCurrIndex; /* after send success, it is update */
    uint8_t I2CoVDMTxBufEnd;
    uint8_t I2CoVDMRxBufEnd;
    uint8_t I2CoVDMWaitSendPreState;

    volatile uint8_t newReceived : 1;
    volatile uint8_t sending : 1;
    volatile uint8_t sendResult : 1;
    volatile uint8_t I2CoVDMLastTxRxResult : 1;
    volatile uint8_t waitTimerOut : 1;
} i2c_over_vdm_state_t;

typedef struct pd_i2c_over_vdm_config
{
    /* the pd handle that used for transfer */
    pd_handle pdHandle;
    /* for i2c over vdm slave, don't need this; for i2c over vdm master, need this. */
    uint32_t i2cSlaveAddress;
    /* for i2c over vdm master, don't need this; for i2c over vdm slave, need this. */
    usb_cmsis_wrapper_handle cmsisHandle;
    /* for master it is callback function for read/write; for slave, it must be NULL */
    pd_i2c_over_vdm_callback_t callback;
    /* callback parameter */
    void *callbackParam;
} pd_i2c_over_vdm_config_t;

typedef void *pd_i2c_over_vdm_handle;

/*******************************************************************************
 * API
 ******************************************************************************/

/* Initialize the I2C over VDM,
 * return value: 0 - success; other values - fail.
 */
extern uint8_t PD_I2cOverVdmInit(pd_i2c_over_vdm_handle *i2cOvdmHandle, pd_i2c_over_vdm_config_t *config);

/* De-initialize the I2C over VDM,
 * return value: 0 - success; other values - fail.
 */
extern uint8_t PD_I2cOverVdmDeinit(pd_i2c_over_vdm_handle i2cOvdmHandle);

/* I2C over VDM task.
 */
extern void PD_I2cOverVdmTask(void *arg);

/* Read data through i2c over vdm.
 * return value: 0 - success; other values - fail.
 */
extern uint8_t PD_I2cOverVdmRead(pd_i2c_over_vdm_handle i2cOvdmHandle, uint16_t offset, uint8_t *buff, uint8_t length);

/* Write data through i2c over vdm.
 * return value: 0 - success; other values - fail.
 */
extern uint8_t PD_I2cOverVdmWrite(pd_i2c_over_vdm_handle i2cOvdmHandle,
                                  uint16_t offset,
                                  const uint8_t *write_buffer,
                                  uint8_t length);

/* Call in one 1ms isr function
 */
extern void PD_I2cOverVdmTimer1msIsr(pd_i2c_over_vdm_handle i2cOvdmHandle);

#endif
