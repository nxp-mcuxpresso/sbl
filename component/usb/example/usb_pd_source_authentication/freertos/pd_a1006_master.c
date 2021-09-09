// ############################################################################
// #             Copyright (C), NXP Semiconductors                            #
// #                       (C), NXP B.V. of Eindhoven                         #
// #                                                                          #
// # All rights are reserved. Reproduction in whole or in part is prohibited  #
// # without the written consent of the copyright owner.                      #
// # NXP reserves the right to make changes without notice at any time.       #
// # NXP makes no warranty, expressed, implied or statutory, including but    #
// # not limited to any implied warranty of merchantability or fitness for    #
// # any particular purpose, or that the use will not infringe any third      #
// # party patent, copyright or trademark. NXP must not be liable for any     #
// # loss or damage arising from its use.                                     #
// ############################################################################

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "soft_timer.h"
#include "entropy.h"
#include "sha224.h"
#include "sha224_extra.h"
#include "a1006_crypto.h"
#include "usb_pd_config.h"
#include "usb_pd.h"
#include "board.h"
#include "pd_board_config.h"
#include "pd_i2c_over_vdm.h"
#include "pd_command_interface.h"
#include "pd_app.h"
#include "pd_a1006_master.h"
#include "usb_osa.h"
#include "usb_cmsis_wrapper.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define A1006_COMPRESSED_CERT_OFFSET (0x0200)
#define A1006_CHALLENGE_RESPONSE_OFFSET (0x0801)
#define A1006_UID_OFFSET (0x0310)
#define A1006_CHALLENGE_OFFSET (0x0800)

#define CERTORG_PATTERN "NXP *CA"
#define CERTCN_PATTERN "*Device*"

/*!
 * @brief NXP root CA public key for test samples.
 *
 * Uncompressed NIST-P224 (secp224r1) public key as uncompressed (x, y) coordinates, in little endian encoding and right
 * zero padded. Must be specified as 32-bit unsigned int for correct allignment.
 */
const uint32_t testRootPub[] = {0x19BD7FEE, 0x64DAF9DD, 0x6940EBE7, 0xEFC1CF82, 0x1C51B388, 0x3F2F754F, 0x321553F2,
                                0x0D41D6B4, 0x74773EEE, 0x2E468820, 0x229D400B, 0x2CBBE5C4, 0x96F57258, 0x2ECB5539};

/*!
 * @brief NXP root CA public key for production devices.
 *
 * Uncompressed NIST-P224 (secp224r1) public key as uncompressed (x, y) coordinates, in little endian encoding and right
 * zero padded. Must be specified as 32-bit unsigned int for correct allignment.
 */
const uint32_t prodRootPub[] = {0x4BBA911B, 0x3E3CF675, 0x972C8450, 0x5DEC45F2, 0xBEB4FA70, 0x3197ED8D, 0x25D04121,
                                0x717306EC, 0xC0BB9C7A, 0xBAA611C1, 0x266C80B8, 0xC99F95C3, 0xEA3685F5, 0x6935C983};

#define ASTERISK (uint8_t)'*'
#define QUESTION (uint8_t)'?'

/* Thresholds for entropy */
#define ENTROPY_MIN_SHANNON 3600                  /* Minimum 3.6 bit of entropy per 4 bits */
#define ENTROPY_MAX_CHISQR_PER_NIBBLE 1.33        /* Maximum 1.33 per 4 bits */
#define ENTROPY_MAX_DELTA_BITSSET_PER_NIBBLE 0.25 /* Max 0.25 more bits set on average per nibble */

/* maximum number of attempts to harvest reasonable hardware */
/* entropy from host capable of passing entropy tests,       */
/* before giving up                                          */
#define MAX_ENTROPY_TRYS 50

#define CERT_SLOT 0     /* 0 for NXP, 1 for user */
#define ENTROPY_SIZE 64 /* Must be power of 2 */
/* to protect against remote relay attacks, A100x must respond */
/* within 65 msec, if it NAK's beyond this is auth failure     */
#define MAX_RESPONSE_TIME 300

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_ifBuf[A1006_IFBUF_DLCERT_SIZE];
uint8_t cert[CERT_SIZE];

/*******************************************************************************
 * Code
 ******************************************************************************/

uint32_t ustrmatch(const uint8_t *upattern, const uint8_t *ustr, const uint32_t ustrlen)
{
    uint8_t const *p = upattern, *p0 = (uint8_t *)0, *q = ustr, *q0 = (uint8_t *)0, *Q;
    /* remove right space padding */
    for (Q = (q + ustrlen); Q != q && *(Q - 1) <= (uint8_t)' '; Q--)
        ;
    if (q == Q)
        return 0;
    if (!(*p))
        return 1;
    /* matching loop */
    for (;;)
    {
        if (*p == ASTERISK)
        {
            for (p++; *p == ASTERISK; p++)
                ;
            if (!(*p))
                return 0;
            if (*p != QUESTION)
            {
                for (; q != Q && *q != *p; q++)
                    ;
                if (q == Q)
                    return 1;
            }
            p0 = p;
            q0 = q;
        }
        else if (*p != QUESTION && *q != *p)
        {
            if (!p0)
                return 1;
            if (p != p0)
            {
                p = p0;
                if (*q != *p)
                {
                    q0++;
                    q = q0;
                    continue;
                }
                else
                    p++;
            }
            if (q != Q)
            {
                q++;
                continue;
            }
            return 1;
        }
        q++;
        p++;
        if (q == Q)
        {
            for (; *p == ASTERISK; p++)
                ;
            if (!(*p))
                return 0;
            return 1;
        }
    }
}

static void Print_Hex(uint8_t *buf, uint32_t bufLen)
{
    for (uint32_t index = 0; index < bufLen; ++index)
    {
        if (buf[index] > 0x0Fu)
        {
            PRINTF(":%X", buf[index]);
        }
        else
        {
            PRINTF(":0%X", buf[index]);
        }
        if ((index + 1) % 16 == 0)
        {
            PRINTF("\r\n");
        }
    }
    PRINTF("\r\n");
}

void PD_A1006i2cOvdmCallback(void *callbackParam, uint32_t result)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;
    pdAppInstance->transferResult = result;
    xSemaphoreGive(pdAppInstance->transferSemaphore);
}

uint32_t a100x_host_cert(pd_app_t *pdAppInstance,
                         const uint32_t interfaceAddr,
                         const uint32_t userRead,
                         const uint32_t *rootPub,
                         uint8_t *ifBufAndUID,
                         uint8_t *cert)
{
    uint32_t fail;

    pdAppInstance->transferResult = 1u;
    /* read A1006 compressed cert */
    if (PD_I2cOverVdmRead(pdAppInstance->i2cHandle, A1006_COMPRESSED_CERT_OFFSET, (ifBufAndUID + 18), 130))
    {
        PRINTF("I2C error\r\n");
        return 1;
    }
    if (pdTRUE != xSemaphoreTake(pdAppInstance->transferSemaphore, portMAX_DELAY)) /* wait the command */
    {
        PRINTF("sem error\r\n");
        return 1;
    }
    if (pdAppInstance->transferResult)
    {
        return 1;
    }
    PRINTF("interface-read of compressed cert:\r\n");
    Print_Hex((ifBufAndUID + 18), 130);
    if (ifBufAndUID[0] || ifBufAndUID[1])
    {
        return 1;
    }
    /* decompress A1006 cert */
    a100x_decompressCert(cert, (ifBufAndUID + 2), (ifBufAndUID + 20));
    PRINTF("X509v3 certificate:\r\n");
    Print_Hex(cert, CERT_SIZE);
    /* verify A1006 certificate */
    fail = a100x_verifyCert(rootPub, cert, &pdAppInstance->sha224_context);
    if (fail)
    {
        PRINTF("-- CERT VERIFY FAIL!! --\r\n");
        return 1;
    }
    PRINTF("## CERT VERIFY OKAY!! ##\r\n");
    return 0;
}

uint32_t a100x_host_auth(pd_app_t *pdAppInstance,
                         const uint32_t interfaceAddr,
                         const uint8_t *hostUID,
                         const uint32_t hostUIDsize,
                         const uint8_t *pubKey,
                         uint8_t *ifBuf)
{
    uint32_t j, k;
    uint32_t T0, T;

    /* read random from PRNG and generate challenge */
    sha224_hkdf_expand(&pdAppInstance->sha224_hkdf_context, hostUID, hostUIDsize);
    PRINTF("random <len=%d>:\r\n", 21);
    Print_Hex(sha224_hkdf_block(&pdAppInstance->sha224_hkdf_context), 21);
    k = a100x_generateChallenge(ifBuf, sha224_hkdf_block(&pdAppInstance->sha224_hkdf_context));
    PRINTF("interface-write of challenge <len=%d>:\r\n", k);
    Print_Hex(ifBuf, k);
    pdAppInstance->transferResult = 1u;
    /* send challenge */
    if (PD_I2cOverVdmWrite(pdAppInstance->i2cHandle, A1006_CHALLENGE_OFFSET, ifBuf, 0x2C))
    {
        PRINTF("I2C error\r\n");
        return 1;
    }
    if (pdTRUE != xSemaphoreTake(pdAppInstance->transferSemaphore, portMAX_DELAY)) /* wait the command */
    {
        PRINTF("sem error\r\n");
        return 1;
    }
    if (pdAppInstance->transferResult)
    {
        return 1;
    }

    /* precompute response while A100x is busy */
    a100x_precomputeResponse(pubKey);
    PRINTF("precompute response\r\n");
    /* read challenge response */
    T0 = PD_DemoSoftTimer_msGet();
    for (;;)
    {
        pdAppInstance->transferResult = 1u;
        if (PD_I2cOverVdmRead(pdAppInstance->i2cHandle, A1006_CHALLENGE_RESPONSE_OFFSET, ifBuf, 46))
        {
            return 1;
        }
        if (pdTRUE != xSemaphoreTake(pdAppInstance->transferSemaphore, portMAX_DELAY)) /* wait the command */
        {
            PRINTF("sem error\r\n");
            return 1;
        }

        T = PD_DemoSoftTimer_getInterval(T0);
        if ((!pdAppInstance->transferResult) || (T > MAX_RESPONSE_TIME))
        {
            break;
        }
        PD_DemoSoftTimer_msSleep(1); /* sleep 1 msec */
    }
    if (T > MAX_RESPONSE_TIME)
    {
        PRINTF("response time-out!\r\n");
        return 1;
    }
    PRINTF("response time:%d\r\n", T);
    PRINTF("interface-read of response:\r\n");
    Print_Hex(ifBuf, 46);
    /* validate challenge response */
    j = a100x_verifyResponse(ifBuf, 46);
    if (j)
    {
        PRINTF("-- AUTH VERIFY FAIL!! --\r\n");
        return 7;
    }
    PRINTF("## AUTH VERIFY OKAY!!! ##\r\n");
    return 0;
}

uint8_t A1006_EntropyInit(uint8_t *usedBuf, uint32_t entropySize)
{
#if (defined BOARD_SUPPORT_HARDWARE_RAND) && (BOARD_SUPPORT_HARDWARE_RAND)
    uint32_t i;
    uint32_t entropy_shannon;
    uint32_t entropy_chisqr;
    uint32_t entropy_setBits;

    if (entropy_init())
    {
        return 1;
    }

    for (i = 0; i < MAX_ENTROPY_TRYS; ++i)
    {
        entropy_harvest(usedBuf, entropySize);
        if (entropy_calculate(usedBuf, entropySize, &entropy_shannon, &entropy_chisqr, &entropy_setBits))
        {
            /* Failed to calculate entropy, try again */
            continue;
        }

        if ((entropy_shannon >= ENTROPY_MIN_SHANNON) &&
            (entropy_chisqr <= ENTROPY_MAX_CHISQR_PER_NIBBLE * entropySize * 2) &&
            (entropy_setBits <= ((entropySize * 8 / 2) + (entropySize * 2 * ENTROPY_MAX_DELTA_BITSSET_PER_NIBBLE))) &&
            (entropy_setBits >= ((entropySize * 8 / 2) - (entropySize * 2 * ENTROPY_MAX_DELTA_BITSSET_PER_NIBBLE))))
        {
            break;
        }
    }

    entropy_deinit();

    if (i < MAX_ENTROPY_TRYS)
    {
        PRINTF("entropy:\r\n");
        Print_Hex(usedBuf, entropySize);
        return 0;
    }
    else
    {
        PRINTF("entropy fail\r\n");
    }

    return 1;
#else
    uint32_t index;

    srand(0x34u);
    for (index = 0; index < entropySize; index++)
    {
        usedBuf[index] = rand();
    }
    PRINTF("entropy:\r\n");
    Print_Hex(usedBuf, entropySize);
    return 0;
#endif
}

uint8_t PD_A1006MasterInit(pd_app_t *pdAppInstance)
{
    /* initialize CMSIS i2c interface */
    /*
    s_A1006I2cHandle = NULL;
    CMSIS_PortControlInterfaceInit(&s_A1006I2cHandle, kInterface_i2c0 + BOARD_PD_I2C_INDEX, NULL);
    if (s_A1006I2cHandle == NULL)
    {
        return 1;
    }
    */
    pd_i2c_over_vdm_config_t config;
    /* demo sem */
    pdAppInstance->transferSemaphore = xSemaphoreCreateCounting(0x01U, 0x00U);
    if (NULL == pdAppInstance->transferSemaphore)
    {
        PRINTF("create sem fail\r\n");
    }

    PD_DemoSoftTimer_init();

    config.pdHandle = pdAppInstance->pdHandle;
    config.i2cSlaveAddress = I2C_SLAVE_ADDR;
    config.cmsisHandle = NULL;
    config.callback = PD_A1006i2cOvdmCallback;
    config.callbackParam = pdAppInstance;
    return PD_I2cOverVdmInit(&(pdAppInstance->i2cHandle), &config);
}

uint8_t PD_A1006MasterDeinit(pd_app_t *pdAppInstance)
{
    /* de-initialize CMSIS i2c interface */
    /* CMSIS_PortControlInterfaceDeinit(s_A1006I2cHandle); */
    /* release demo sem */
    vSemaphoreDelete(pdAppInstance->transferSemaphore);
    pdAppInstance->transferSemaphore = NULL;
    PD_DemoSoftTimer_deinit();
    return PD_I2cOverVdmDeinit(pdAppInstance->i2cHandle);
}

uint8_t PD_A1006MasterAuth(pd_app_t *pdAppInstance)
{
    uint8_t fail = 0;
    const uint32_t *rootPub;

    /* initialize entropy */
    fail |= A1006_EntropyInit(g_ifBuf, ENTROPY_SIZE);
    if (fail)
    {
        return fail;
    }
    PRINTF("entropy init success\r\n");

    /* initialize HKDF based PRNG */
    pdAppInstance->sha224_hkdf_context.sha224_context = &pdAppInstance->sha224_context;
    sha224_hkdf_extract(&pdAppInstance->sha224_hkdf_context, g_ifBuf, ENTROPY_SIZE);

    PRINTF("START...\r\n");

    pdAppInstance->transferResult = 1u;
    /* get uid */
    fail |= PD_I2cOverVdmRead(pdAppInstance->i2cHandle, A1006_UID_OFFSET, g_ifBuf, 18);
    if (fail)
    {
        return fail;
    }
    if (pdTRUE != xSemaphoreTake(pdAppInstance->transferSemaphore, portMAX_DELAY)) /* wait the command */
    {
        PRINTF("sem error\r\n");
        return 1;
    }
    if (pdAppInstance->transferResult)
    {
        return 1;
    }
    if (g_ifBuf[0] || g_ifBuf[1])
    {
        return 1;
    }
    PRINTF("UID:\r\n");
    Print_Hex(g_ifBuf, 18);

    if ((g_ifBuf[0] == 0x00 && g_ifBuf[2] == 0x00) &&
        ((g_ifBuf[1] == 0x01 && g_ifBuf[3] == 0x0F && g_ifBuf[6] == 0x00 && g_ifBuf[7] == 0x01) ||
         (g_ifBuf[1] == 0x00 && g_ifBuf[3] == 0x00 && g_ifBuf[4] == 0x00 && g_ifBuf[5] == 0x00)))
    {
        rootPub = testRootPub;
    }
    else
    {
        rootPub = prodRootPub;
    }

    fail |= a100x_host_cert(pdAppInstance, I2C_SLAVE_ADDR, CERT_SLOT, rootPub, g_ifBuf, cert);
    if (fail)
    {
        return fail;
    }
    fail |= a100x_host_auth(pdAppInstance, I2C_SLAVE_ADDR, (uint8_t *)HOST_UID, (sizeof(HOST_UID) - 1),
                            (cert + CERT_PUBKEY_OFF), g_ifBuf);

    if (!fail)
    {
        PRINTF("FINISHED - AUTH SUCCESS!\r\n");

        /* show certificate details */
        PRINTF("## ** CERT DETAILS *** ##\r\n");
        PRINTF("certificate unique ID");
        Print_Hex((cert + CERT_SUBUID_OFF), CERT_SUBUID_SIZE);
        /* cert org name */
        PRINTF("certificate organization");
        Print_Hex((cert + CERT_SUBORG_OFF), CERT_SUBORG_SIZE);
        /* match against specified pattern */
        if (ustrmatch((uint8_t *)CERTORG_PATTERN, (cert + CERT_SUBORG_OFF), CERT_SUBORG_SIZE))
            PRINTF("certificate organization mismatch with {" CERTORG_PATTERN "}\r\n");
        else
            PRINTF("certificate organization match with {" CERTORG_PATTERN "}\r\n");
        /* cert common name */
        PRINTF("certificate common-name", (cert + CERT_SUBCN_OFF), CERT_SUBCN_SIZE);
        /* match against specified pattern */
        if (ustrmatch((uint8_t *)CERTCN_PATTERN, (cert + CERT_SUBCN_OFF), CERT_SUBCN_SIZE))
            PRINTF("certificate common-name mismatch with {" CERTCN_PATTERN "}\r\n");
        else
            PRINTF("certificate common-name match with {" CERTCN_PATTERN "}\r\n");
    }

    return fail;
}

void PD_DemoReset(pd_app_t *pdAppInstance)
{
    pdAppInstance->authing = 0u;
    pdAppInstance->contractValid = 0;
    ((pd_power_port_config_t *)pdAppInstance->pdConfigParam->deviceConfig)->sourceCapCount = 1u;
}

void PD_DemoInit(pd_app_t *pdAppInstance)
{
    uint8_t index;

    for (index = 0; index < PD_DEMO_PORTS_COUNT; ++index)
    {
        if (g_PDAppInstanceArray[index]->pdHandle != NULL)
        {
            PD_DemoReset(g_PDAppInstanceArray[index]);
        }
    }
    return;
}

void PD_AuthI2cOverVdmTask(void *arg)
{
    while (1)
    {
        PD_I2cOverVdmTask(arg);
    }
}

void PD_AuthTaskFn(void *arg)
{
    pd_app_t *pdAppInstance = (pd_app_t *)arg;

    while (1)
    {
        if (!pdAppInstance->contractValid)
        {
            continue;
        }

        /* do authtication */
        if (pdAppInstance->selfSouceCapNumber > 1)
        {
            if (PD_A1006MasterInit(pdAppInstance))
            {
                PRINTF("a1006 master init fail\r\n");
            }
            else
            {
                if (xTaskCreate(PD_AuthI2cOverVdmTask, "i2c", 1024 / sizeof(portSTACK_TYPE), pdAppInstance->i2cHandle,
                                5, &pdAppInstance->i2cOvdmTaskHandle) != pdPASS)
                {
                    PRINTF("create task error\r\n");
                }
                if (!PD_A1006MasterAuth(pdAppInstance))
                {
                    /* auth success */
                    ((pd_power_port_config_t *)pdAppInstance->pdConfigParam->deviceConfig)->sourceCapCount =
                        pdAppInstance->selfSouceCapNumber;

                    if (PD_Command(pdAppInstance->pdHandle, PD_DPM_CONTROL_POWER_NEGOTIATION, NULL) !=
                        kStatus_PD_Success)
                    {
                        PRINTF("command fail\r\n");
                    }
                }
                else
                {
                    PRINTF("auth fail\r\n");
                }
                portENTER_CRITICAL();
                vTaskDelete(pdAppInstance->i2cOvdmTaskHandle);
                pdAppInstance->i2cOvdmTaskHandle = NULL;
                portEXIT_CRITICAL();
            }
            PD_A1006MasterDeinit(pdAppInstance);
        }
        portENTER_CRITICAL();
        pdAppInstance->authing = 0u;
        pdAppInstance->authTaskHandle = NULL;
        vTaskDelete(NULL); /* delete current task */
        portEXIT_CRITICAL();
    }
}

void PD_AuthTrigger(pd_app_t *pdAppInstance)
{
    portENTER_CRITICAL();
    if (pdAppInstance->authing)
    {
        if (pdAppInstance->authTaskHandle)
        {
            vTaskDelete(pdAppInstance->authTaskHandle);
        }
        if (pdAppInstance->i2cOvdmTaskHandle)
        {
            vTaskDelete(pdAppInstance->i2cOvdmTaskHandle);
        }
    }

    pdAppInstance->authTaskHandle = NULL;
    pdAppInstance->i2cOvdmTaskHandle = NULL;
    if (xTaskCreate(PD_AuthTaskFn, "auth", PD_DEMO_AUTH_TASK_STACK_SIZE / sizeof(portSTACK_TYPE), pdAppInstance, 4,
                    &pdAppInstance->authTaskHandle) != pdPASS)
    {
        PRINTF("create task error\r\n");
    }
    else
    {
        pdAppInstance->authing = 1u;
    }
    portEXIT_CRITICAL();
}
