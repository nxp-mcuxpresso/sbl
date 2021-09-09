/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#ifndef _HAB_RVT_H_
#define _HAB_RVT_H_

#include "hab_type.h"

typedef void (*hab_image_entry_f)(void);
typedef hab_state_t (* hab_loader_callback_f)(void **start, size_t *bytes, const void *boot_data);

hab_status_t hab_Entry(void);
uint32_t hab_Get_version(void);
hab_image_entry_f hab_Authenticate_image(uint8_t cid, uint32_t ivt_offset, uint32_t start, size_t bytes);
hab_image_entry_f hab_authenticate_image_no_dcd(uint8_t cid, uint32_t ivt_offset, uint32_t start, size_t bytes);
hab_status_t hab_exit(void);
hab_status_t hab_rvt_report_status(hab_config_t *config, hab_state_t *state);
hab_status_t hab_rvt_report_event(hab_state_t status, uint32_t index, uint8_t *event, size_t *bytes);
hab_status_t hab_rvt_check_target(hab_target_t type, const void *start, size_t bytes);

#endif