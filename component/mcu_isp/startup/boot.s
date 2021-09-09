/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

        import Reset_Handler

        SECTION .boot:CONST(2)
        
        DATA

__boot
#ifdef DEBUG
        DCD     0xffffffff
        DCD     Reset_Handler
#else
        DCD     0xffffffff
        DCD     0xffffffff
#endif

        END

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
