/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "sbl.h"
#include "boot.h" 
#include "fsl_debug_console.h"


iapfun jump2app; 

#define IOMUXC_GPR_GPR30_REG 0x400AC078//To specify the start address of flexspi1 and flexspi2
#define IOMUXC_GPR_GPR31_REG 0x400AC07C//To specify the end address of flexspi1 and flexspi2
#define IOMUXC_GPR_GPR32_REG 0x400AC080//To specify the offset address of flexspi1 and flexspi2

struct arm_vector_table {
    uint32_t msp;
    uint32_t reset;
};

static struct arm_vector_table *vt;

#pragma weak cleanup
void cleanup(void);

void set_image_addr(uint32_t start_addr, uint32_t end_addr)
{
	*((volatile uint32_t *)IOMUXC_GPR_GPR30_REG) = start_addr;
	*((volatile uint32_t *)IOMUXC_GPR_GPR31_REG) = end_addr;
}

void change_image_offset(uint32_t offset_size)
{
	*((volatile uint32_t *)IOMUXC_GPR_GPR32_REG) = offset_size;
}

/* The bootloader of MCUboot */
void do_boot(struct boot_rsp *rsp)
{
    uintptr_t flash_base;
    int rc;

    /* The beginning of the image is the ARM vector table, containing
     * the initial stack pointer address and the reset vector
     * consecutively. Manually set the stack pointer and jump into the
     * reset vector
     */
    rc = flash_device_base(rsp->br_flash_dev_id, &flash_base);
    assert(rc == 0);

    vt = (struct arm_vector_table *)(flash_base +
                                     rsp->br_image_off +
#ifdef MCUBOOT_SIGN_ROM
                                     HAB_IVT_OFFSET +
#endif
                                     rsp->br_hdr->ih_hdr_size);

    cleanup();

    __set_MSP(vt->msp);
	__set_CONTROL(0);
    ((void (*)(void))vt->reset)();
}
