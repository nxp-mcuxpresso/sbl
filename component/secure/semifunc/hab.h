/*
 * Copyright 2021 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __HAB_H__
#define __HAB_H__

#include "hab_rvt.h"
#include "fsl_debug_console.h"

__PACKED_STRUCT ivt_header 
{
	uint8_t     tag;
	uint16_t	length;
	uint8_t		version;
};

/*************************************
 *  IVT Data
 *************************************/
struct _ivt_
{
    /** hdr with tag #HAB_TAG_IVT, length and HAB version fields
     */
	struct ivt_header hdr;
    /** Absolute address of the first instruction to execute from the
     *  image
     */
    uint32_t entry;
    /** Reserved in this version of HAB: should be NULL. */
    uint32_t reserved1;
    /** Absolute address of the image DCD: may be NULL. */
    uint32_t dcd;
    /** Absolute address of the Boot Data: may be NULL, but not interpreted
     *  any further by HAB
     */
    uint32_t boot_data;
    /** Absolute address of the IVT.*/
    uint32_t self;
    /** Absolute address of the image CSF.*/
    uint32_t csf;
    /** Reserved in this version of HAB: should be zero. */
    uint32_t reserved2;
};

/*************************************
 *  Boot Data
 *************************************/
typedef struct _boot_data_
{
    uint32_t start;       /* boot start location */
    uint32_t size;        /* size */
    uint32_t plugin;      /* plugin flag - 1 if downloaded application is plugin */
    uint32_t placeholder; /* placehoder to make even 0x10 size */
} BOOT_DATA_T;

int imx_hab_authenticate_image(uint32_t image_start, uint32_t image_size,
                               uint32_t ivt_offset);
#endif
