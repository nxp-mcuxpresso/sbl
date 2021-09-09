/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
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
#include "virtual_nic_enetif.h"

#include "fsl_enet.h"
#include "fsl_phy.h"
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_cdc_acm.h"
#include "usb_device_cdc_rndis.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "virtual_nic_enet_adapter.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void ENETIF_Input(enet_handle_t *handle, uint8_t channel, void *param);
void VNIC_EnetCallback(pbuf_t *pbuffer);
void VNIC_EnetRxBufFree(pbuf_t *pbuf);
uint8_t *VNIC_EnetRxBufAlloc(void);
usb_status_t VNIC_EnetTxDone(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
enet_handle_t g_handle;
extern uint8_t g_hwaddr[ENET_MAC_ADDR_SIZE];
uint8_t g_hwaddr[6] = {0xd4, 0xbe, 0xd9, 0x45, 0x22, 0x60};

#if defined(__ICCARM__)
#pragma data_alignment = ENET_BUFF_ALIGNMENT
#endif
__ALIGN_BEGIN enet_rx_bd_struct_t g_rxBuffDescrip[ENET_RXBD_NUM] __ALIGN_END;
#if defined(__ICCARM__)
#pragma data_alignment = ENET_BUFF_ALIGNMENT
#endif
__ALIGN_BEGIN enet_tx_bd_struct_t g_txBuffDescrip[ENET_TXBD_NUM] __ALIGN_END;
#if defined(__ICCARM__)
#pragma data_alignment = ENET_BUFF_ALIGNMENT
#endif
__ALIGN_BEGIN uint8_t RxDataBuff[ENET_RXBD_NUM][ENET_RXBUFF_SIZE] __ALIGN_END;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Services an ethernet interrupt.
 *
 * @return none.
 */
void ENETIF_Callback(ENET_Type *base, enet_handle_t *handle, enet_event_t event, uint8_t channel, void *param)
{
    switch (event)
    {
        case kENET_RxIntEvent:
            ENETIF_Input(handle, channel, param);
            break;
        case kENET_TxIntEvent:
            VNIC_EnetTxDone();
            break;
        default:
            break;
    }
}

/*!
 * @brief Get the ethernet speed.
 *
 * @return Value of ethernet speed in Mbps.
 */
uint32_t ENETIF_GetSpeed(void)
{
    uint32_t phyAddr = 0;
    bool link = false;
    phy_speed_t speed;
    phy_duplex_t duplex;
    uint32_t count = 0;
    speed = kPHY_Speed100M;
    while ((count < ENET_PHY_TIMEOUT) && (!link))
    {
        PHY_GetLinkStatus(ENET, phyAddr, &link);
        if (link)
        {
            /* Get the actual PHY link speed. */
            PHY_GetLinkSpeedDuplex(ENET, phyAddr, &speed, &duplex);
        }

        count++;
    }

    if (count == ENET_PHY_TIMEOUT)
    {
        usb_echo("\r\nPHY Link down, please check the cable connection.\r\n");
    }
    return (kPHY_Speed100M == speed ? 100 : 10);
}

/*!
 * @brief Get the ethernet link status.
 *
 * @return Ture if linked, otherwise false.
 */
bool ENETIF_GetLinkStatus(void)
{
    uint32_t phyAddr = 0;
    bool link = false;

    uint32_t count = 0;
    while ((count < ENET_PHY_TIMEOUT) && (!link))
    {
        PHY_GetLinkStatus(ENET, phyAddr, &link);

        count++;
    }

    if (count == ENET_PHY_TIMEOUT)
    {
        usb_echo("\r\nPHY Link down, please check the cable connection.\r\n");
    }
    return link;
}

/*!
 * @brief Transmit the packet.
 *
 * @return Error code.
 */

enet_err_t ENETIF_Output(pbuf_t *packetBuffer)
{
    status_t sts;
    enet_err_t err;
    if (packetBuffer->length >= ENET_FRAME_MAX_FRAMELEN)
    {
        return ENET_ERROR;
    }
    /* Send a frame out. */
    sts = ENET_SendFrame(ENET, &g_handle, packetBuffer->payload, packetBuffer->length);
    if (kStatus_Success == sts)
    {
        err = ENET_OK;
    }
    else if (kStatus_ENET_TxFrameBusy == sts)
    {
        err = ENET_BUSY;
    }
    else
    {
        err = ENET_ERROR;
    }
    return err;
}

/*!
 * @brief Handle the receptions of packets from the network interface.
 *
 * @return none.
 */
void ENETIF_Input(enet_handle_t *handle, uint8_t channel, void *param)
{
    enet_header_t *ethhdr;
    status_t status;
    pbuf_t packetBuffer;
    uint32_t length = 0;

    /* Read all data from ring buffer and send to uper layer */
    do
    {
        length = 0;
        /* Get the Frame size */
        status = ENET_GetRxFrameSize(ENET, handle, &length, channel);
        /* Call ENET_ReadFrame when there is a received frame. */
        if (status == kStatus_Success)
        {
            packetBuffer.payload = VNIC_EnetRxBufAlloc();
            /* Received valid frame. Deliver the rx buffer with the size equal to length. */
            if (packetBuffer.payload != NULL)
            {
                packetBuffer.length = (uint16_t)length;
                ENET_ReadFrame(ENET, handle, packetBuffer.payload, packetBuffer.length, channel);

                /* points to packet payload, which starts with an Ethernet header */
                ethhdr = (enet_header_t *)packetBuffer.payload;

                switch (USB_SHORT_FROM_BIG_ENDIAN(ethhdr->type))
                {
                    /* IP or ARP packet? */
                    case ETHTYPE_IP:
                    case ETHTYPE_ARP:
                        /* send packet to application to process */
                        VNIC_EnetCallback(&packetBuffer);
                        break;

                    default:
                        VNIC_EnetRxBufFree(&packetBuffer);
                        break;
                }
            }
            else
            {
                /* No packet buffer available, ignore this packet. */
                ENET_ReadFrame(ENET, handle, NULL, length, channel);
            }
        }
        else if (status == kStatus_ENET_RxFrameError)
        {
            /* update the receive buffer. */
            ENET_ReadFrame(ENET, handle, NULL, 0, channel);
        }
    } while (kStatus_ENET_RxFrameEmpty != status);
}

/*!
 * @brief Initialize the ethernet.
 *
 * @return Error code.
 */
enet_err_t ENETIF_Init(void)
{
    enet_config_t config;
    uint32_t rxbuffer[ENET_RXBD_NUM];
    uint8_t index;
    uint32_t refClock = 50000000; /* 50MHZ for rmii reference clock. */
    phy_speed_t speed;
    phy_duplex_t duplex;
    uint32_t timedelay;
    uint32_t phyAddr = 0;
    enet_err_t result = ENET_OK;
    bool link = false;

    for (index = 0; index < ENET_RXBD_NUM; index++)
    {
        rxbuffer[index] = (uint32_t)(&RxDataBuff[index][0]);
    }

    /* prepare the buffer configuration. */
    enet_buffer_config_t buffConfig = {
        ENET_RXBD_NUM,       ENET_TXBD_NUM,
        &g_txBuffDescrip[0], &g_txBuffDescrip[0],
        &g_rxBuffDescrip[0], &g_rxBuffDescrip[ENET_RXBD_NUM],
        &rxbuffer[0],        ENET_BuffSizeAlign(ENET_RXBUFF_SIZE),
    };

    PHY_Init(ENET, phyAddr, 0);

    /* Get default configuration 100M RMII. */
    ENET_GetDefaultConfig(&config);

    /* Get the actual PHY link speed. */
    PHY_GetLinkSpeedDuplex(ENET, phyAddr, &speed, &duplex);
    /* Change the MII speed and duplex for actual link status. */
    config.miiSpeed = (enet_mii_speed_t)speed;
    config.miiDuplex = (enet_mii_duplex_t)duplex;

    /* Initialize ENET. */
    config.specialControl = kENET_StoreAndForward;
    ENET_Init(ENET, &config, &g_hwaddr[0], refClock);
    ENET_EnableInterrupts(ENET, kENET_DmaTx);
    ENET_EnableInterrupts(ENET, kENET_DmaRx);
    NVIC_SetPriority(ETHERNET_IRQn, 3U);

    /* Initialize Descriptor. */
    ENET_DescriptorInit(ENET, &config, &buffConfig);

    /* Create the handler. */
    ENET_CreateHandler(ENET, &g_handle, &config, &buffConfig, ENETIF_Callback, NULL);
    /* Active TX/RX. */
    ENET_StartRxTx(ENET, 1, 1);

    PHY_GetLinkStatus(ENET, phyAddr, &link);
    /* Wait for link up. */
    while (!link)
    {
        usb_echo("\r\nLink down, please check the cable connection and test will start when link up\r\n");

        for (timedelay = 0; timedelay < 0xFFFU; timedelay++)
        {
        }
        PHY_GetLinkStatus(ENET, phyAddr, &link);
    }

    return result;
}
