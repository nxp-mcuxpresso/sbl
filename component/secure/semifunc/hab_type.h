/*
 * Copyright (c) 2007 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef HAB_TYPES_H
#define HAB_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>             /* for integer types */
#include <stdbool.h>            /* for bool type */
#include <stddef.h>             /* for NULL and offsetof() */

/** 
 * Tag definitions 
 */
#define HAB_TAG_IVT  0xd1         /* Alias for Image Vector Table V0 */
#define HAB_TAG_CSF  0xd4         /* CSF Header */
#define HAB_TAG_EVT  0xdb         /* Event */

/**
 * IVT header definitions
 */
#define IVT_TOTAL_LENGTH	0x20

/** Target check types
 */
typedef enum hab_target {
    HAB_TGT_MEMORY = 0x0f,      /* Check memory white list */
    HAB_TGT_PERIPHERAL = 0xf0,  /* Check peripheral white list */
    HAB_TGT_ANY = 0x55,         /* Check memory & peripheral white list */
} hab_target_t;

/** Security configuration types
 */
typedef enum hab_config {
    HAB_CFG_FAB = 0x00,         /* Un-programmed IC */
    HAB_CFG_RETURN = 0x33,      /* Field Return IC */
    HAB_CFG_OPEN = 0xf0,        /* Non-secure IC */
    HAB_CFG_CLOSED = 0xcc       /* Secure IC */
} hab_config_t;

/** HAB state definitions
 */
typedef enum hab_state {
    HAB_STATE_INITIAL = 0x33,   /* Initialising state (transitory) */
    HAB_STATE_CHECK = 0x55,     /* Check state (non-secure) */
    HAB_STATE_NONSECURE = 0x66, /* Non-secure state */
    HAB_STATE_TRUSTED = 0x99,   /* Trusted state */
    HAB_STATE_SECURE = 0xaa,    /* Secure state */
    HAB_STATE_FAIL_SOFT = 0xcc, /* Soft fail state */
    HAB_STATE_FAIL_HARD = 0xff, /* Hard fail state (terminal) */
    HAB_STATE_NONE = 0xf0,      /* No security state machine */
} hab_state_t;

/* HAB status definitions */
typedef enum hab_status 
{
    HAB_STS_ANY = 0x00,         /* Match any status in hab_rvt.report_event() */
    HAB_FAILURE = 0x33,         /* Operation failed */
    HAB_WARNING = 0x69,         /* Operation completed with warning */
    HAB_SUCCESS = 0xf0,         /* Operation completed successfully */
} hab_status_t;

/* Event reason definitions */
typedef enum hab_reason 
{
    HAB_RSN_ANY = 0x00,         /* Match any reason in hab_rvt.report_event() */
    HAB_ENG_FAIL = 0x30,        /* Engine failure */
    HAB_INV_ADDRESS = 0x22,     /* Invalid address: access denied */
    HAB_INV_ASSERTION = 0x0c,   /* Invalid assertion */
    HAB_INV_CALL = 0x28,        /* Function called out of sequence */
    HAB_INV_CERTIFICATE = 0x21, /* Invalid certificate */
    HAB_INV_COMMAND = 0x06,     /* Invalid command: command malformed */
    HAB_INV_CSF = 0x11,         /* Invalid Command Sequence File */
    HAB_INV_DCD = 0x27,         /* Invalid Device Configuration Data */
    HAB_INV_INDEX = 0x0f,       /* Invalid index: access denied */
    HAB_INV_IVT = 0x05,         /* Invalid Image Vector Table */
    HAB_INV_KEY = 0x1d,         /* Invalid key */
    HAB_INV_RETURN = 0x1e,      /* Failed callback function */
    HAB_INV_SIGNATURE = 0x18,   /* Invalid signature */
    HAB_INV_SIZE = 0x17,        /* Invalid data size */
    HAB_INV_BLOB = 0x3a,        /* Invalid Blob */
    HAB_INV_MAC = 0x3c,         /* Invalid MAC */
    HAB_MEM_FAIL = 0x2e,        /* Memory failure */
    HAB_OVR_COUNT = 0x2b,       /* Expired poll count */
    HAB_OVR_STORAGE = 0x2d,     /* Exhausted storage region */
    HAB_UNS_ALGORITHM = 0x12,   /* Unsupported algorithm */
    HAB_UNS_COMMAND = 0x03,     /* Unsupported command */
    HAB_UNS_ENGINE = 0x0a,      /* Unsupported engine */
    HAB_UNS_ITEM = 0x24,        /* Unsupported configuration item */
    HAB_UNS_KEY = 0x1b,         /* Unsupported key type or parameters */
    HAB_UNS_PROTOCOL = 0x14,    /* Unsupported protocol */
    HAB_UNS_STATE = 0x09,       /* Unsuitable state */
} hab_reason_t;

/* Event context definitions */
typedef enum hab_context {
    HAB_CTX_ANY = 0x00,         /* Match any context in hab_rvt.report_event() */
    HAB_CTX_FAB = 0xff,         /* Event logged in hab_fab_test() */
    HAB_CTX_ENTRY = 0xe1,       /* Event logged in hab_rvt.entry() */
    HAB_CTX_TARGET = 0x33,      /* Event logged in hab_rvt.check_target() */
    HAB_CTX_AUTHENTICATE = 0x0a, /* Event logged in hab_rvt.authenticate_image() */
    HAB_CTX_DCD = 0xdd,         /* Event logged in hab_rvt.run_dcd() */
    HAB_CTX_CSF = 0xcf,         /* Event logged in hab_rvt.run_csf() */
    HAB_CTX_COMMAND = 0xc0,     /* Event logged executing Command Sequence File or Device Configuration Data command */
    HAB_CTX_AUT_DAT = 0xdb,     /* Authenticated data block */
    HAB_CTX_ASSERT = 0xa0,      /* Event logged in hab_rvt.assert() */
    HAB_CTX_EXIT = 0xee,        /* Event logged in hab_rvt.exit() */
    HAB_CTX_MAX
} hab_context_t;

/* Assertion types */
typedef enum hab_assertion {
    HAB_ASSERT_BLOCK = 0, /* Assert that a memory block was authenticated */
} hab_assertion_t;

/* HAB header field components */
typedef struct hab_hdr {
    uint8_t tag;              /* Tag field */
    uint8_t len[2];           /* Including the Header and must be at least four*/
    uint8_t par;              /* Command parameters */
} hab_hdr_t;

#ifdef __cplusplus
}
#endif

#endif /* HAB_TYPES_H */
