/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mcuboot_config.h"
#include "hab_rvt.h"

#if defined(SOC_IMXRT1170_SERIES)
#define HABRVT_API_TREE_ADDR (0x00211C0C)
#elif defined(SOC_IMXRT1010_SERIES)
#define HABRVT_API_TREE_ADDR (0x002001E0)
#elif (defined(SOC_IMXRT1050_SERIES) || defined(SOC_IMXRT1060_SERIES) || defined(SOC_IMXRT1064_SERIES))
#define HABRVT_API_TREE_ADDR (0x00200300)
#elif (defined(SOC_IMXRT1015_SERIES) || defined(SOC_IMXRT1020_SERIES))
#define HABRVT_API_TREE_ADDR (0x002002C0)
#else
#error "Doesn't define HAB API table address"
#endif

#define HABRVT_API_TREE_POINTER ((hab_rvt_t *)HABRVT_API_TREE_ADDR)


/** Image entry function prototype
 *  @ingroup rvt
 *
 * This typedef serves as the return type for hab_rvt.authenticate_image().  It
 * specifies a void-void function pointer, but can be cast to another function
 * pointer type if required.
 */

typedef struct HABRVT
{
    hab_hdr_t hdr;
    hab_status_t (* entry)(void);
    hab_status_t (* exit)(void);
    hab_status_t (* check_target)(hab_target_t type, const void *start, size_t bytes);
    hab_image_entry_f (* authenticate_image)(uint8_t cid, ptrdiff_t ivt_offset, void **start, size_t *bytes, hab_loader_callback_f loader);
    hab_status_t (* run_dcd)(const uint8_t *dcd);
    hab_status_t (* run_csf)(const uint8_t *csf, uint8_t cid, uint32_t srkmask);
    hab_status_t (* assert)(hab_assertion_t type, const void *data, uint32_t count);
    hab_status_t (* report_event)(hab_state_t status, uint32_t index, uint8_t *event, size_t *bytes);
    hab_status_t (* report_status)(hab_config_t *config, hab_state_t *state);
    void (* failsafe)(void);
    hab_image_entry_f (* authenticate_image_no_dcd)(uint8_t cid, ptrdiff_t ivt_offset, void **start, size_t *bytes, hab_loader_callback_f loader);
    uint32_t (* get_vertion)(void);
    hab_state_t (* authenticate_container)(uint8_t cid, ptrdiff_t ivt_offset, void **start, size_t *bytes, hab_loader_callback_f loader, uint32_t srkmask, int skip_dcd);
} hab_rvt_t;

hab_status_t hab_Entry(void)
{
  return HABRVT_API_TREE_POINTER->entry();
}

uint32_t hab_Get_version(void)
{
  return HABRVT_API_TREE_POINTER->get_vertion();
}

hab_image_entry_f hab_Authenticate_image(uint8_t cid, uint32_t ivt_offset, uint32_t start, size_t bytes)
{
  return HABRVT_API_TREE_POINTER->authenticate_image(cid, ivt_offset, (void **)&start, (size_t *)&bytes, NULL);
}
  
hab_image_entry_f hab_authenticate_image_no_dcd(uint8_t cid, uint32_t ivt_offset, uint32_t start, size_t bytes)
{
  return HABRVT_API_TREE_POINTER->authenticate_image_no_dcd(cid, ivt_offset, (void **)&start, (size_t *)&bytes, NULL);
}

hab_status_t hab_exit(void)
{
  return HABRVT_API_TREE_POINTER->exit();
}

hab_status_t hab_rvt_report_status(hab_config_t *config, hab_state_t *state)
{
  return HABRVT_API_TREE_POINTER->report_status(config, state);
}

hab_status_t hab_rvt_report_event(hab_state_t status, uint32_t index, uint8_t *event, size_t *bytes)
{
  return HABRVT_API_TREE_POINTER->report_event(status, index, event, bytes);
}

hab_status_t hab_rvt_check_target(hab_target_t type, const void *start, size_t bytes)
{
  return HABRVT_API_TREE_POINTER->check_target(type, start, bytes);
}
