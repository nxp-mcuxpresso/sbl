;/*****************************************************************************
; * @file:    security_misc.s
; *
; Copyright 2018-2021 NXP
; All rights reserved.
;
; SPDX-License-Identifier: BSD-3-Clause
;
        SECTION .secure_misc:CODE(4)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;

        THUMB

        PUBLIC __stack_chk_fail
__stack_chk_fail
        CPSID F            ; Set FAULTMASK
        WFI                ; Wait for interrupt
        B .
        END