/*
 * The Clear BSD License
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "fsl_flexspi.h"
#include "flexspi_nor_flash.h"
#include "fusemap.h"
#include "utilities/fsl_assert.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
enum
{
    kSerialNOR_3ByteAddressRead = 0U, // Device supports 0x03 Read with 24bit address
    kSerialNOR_4ByteAddressRead = 1U, // Device supports 0x13 Read with 32bit address
    kSerialNOR_HyperFlash1V8 = 2U,    // HyperFlash 1V8 part
    kSerialNOR_HyperFlash3V3 = 3U,    // HyperFlash 3V3 part
    kSerialNOR_MxicOctDDR = 4U,       // MXIC Octal Flash with OPI DDR read enabled by default
    kSerialNOR_MicronOctDDR = 5U,     // Micron octal Flash with OPI DDR read enabled by default
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
// Dedicated 3Byte Address Read(0x03), 24bit address
static const uint32_t s_dedicated3bRead[4] = {
    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x03, RADDR_SDR, FLEXSPI_1PAD, 0x18),
    FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0), 0, 0
};
// Dedicated 4Byte Address Read(0x13), 32 bit address
static const uint32_t s_dedicated4bRead[4] = {
    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x13, RADDR_SDR, FLEXSPI_1PAD, 0x20),
    FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0), 0, 0
};
// HyperFlash Read
static const uint32_t s_hyperflashRead[4] = {
    FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xA0, RADDR_DDR, FLEXSPI_8PAD, 0x18),
    FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10, READ_DDR, FLEXSPI_8PAD, 0x04), 0, 0
};

// MXIC Octal DDR read
static const uint32_t s_mxicOctDdrRead[4] = {
    FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xEE, CMD_DDR, FLEXSPI_8PAD, 0x11),
    FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20, READ_DDR, FLEXSPI_8PAD, 0x04), 0, 0
};

////////////////////////////////////////////////////////////////////////////////
// Codes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
