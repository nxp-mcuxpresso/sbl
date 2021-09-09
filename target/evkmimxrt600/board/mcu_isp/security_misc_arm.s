;/*****************************************************************************
; * @file:    security_misc.s
; *
; Copyright 2018-2021 NXP
; All rights reserved.
;
; SPDX-License-Identifier: BSD-3-Clause
;
;#include "../authentication/skboot_common.h"
		AREA    |.secure_misc|, CODE, READONLY

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
SECURE_TERM_PART_LOCK   EQU 0x6ac3c36a

        PRESERVE8
        THUMB

        EXPORT is_fault_analysis_mode_enabled
is_fault_analysis_mode_enabled
        IMPORT  otp_fuse_read
        PUSH {R1-R3,LR}
        LDR R0, =0x65            ; pre-init R0 (OTP_FA_FUSE_IDX)
        PUSH {R1}                ; reserve space for fuse data
        MOV R1, SP
        BL otp_fuse_read         ; R0 = fuse addr, R1 = ptr to data
        CBNZ R0, go_fatal_mode
        POP {R1}                 ; place fuse value to R1
        LDR R0, =0x3CC35AA5      ; pre-load default return value (SECURE_FALSE)
        LDR R2, =0x10             ; R2 = mask (OTP_FA_MASK, bit 4 in fuse word 0x65)
        AND R1, R1, R2           ; apply mask
        MOV R2, R1
        ROR R2, R2, #1
        ORR R1, R1, R2
        MOV R2, R1
        ROR R2, R2, #2
        ORR R1, R1, R2
        MOV R2, R1
        ROR R2, R2, #4
        ORR R1, R1, R2
        MOV R2, R1
        ROR R2, R2, #8
        ORR R1, R1, R2
        MOV R2, R1
        ROR R2, R2, #16
        ORR R1, R1, R2
        EOR R0, R0, R1
        LDR R3, =0xFD0           ; (SYSCTL0 offset 0x0000 0FD0)
        STR R0, [R3]
        POP {R1-R3,PC}           ; return

        EXPORT skboot_refresh_stack_canary
skboot_refresh_stack_canary
        PUSH {lr}
        LDR r0, =g_prngCtx
        LDR r1, =skboot_get_random_word
        BLX r1
        LDR r1, =__stack_chk_guard
        STR r0, [r1]
        pop {pc}

        EXPORT __stack_chk_fail
__stack_chk_fail
        IMPORT  g_prngCtx
        IMPORT  __stack_chk_guard
        IMPORT  skboot_get_random_word
        CPSID F            ; Set FAULTMASK
        WFI                ; Wait for interrupt
        B go_fatal_mode    ; Mine field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field

        EXPORT go_test_mode
go_test_mode
        CPSID F            ; Set FAULTMASK
        WFI                ; Wait for interrupt
        B go_fatal_mode    ; Mine field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field

        EXPORT go_fatal_mode
go_fatal_mode
        IMPORT SECURE_termination
        IMPORT fatal_error_reboot_retry
        LDR R0, =fatal_error_reboot_retry
        BLX R0
        LDR R0, =SECURE_TERM_PART_LOCK
        LDR R1, =SECURE_termination
        BLX R1
        CPSID F            ; Set FAULTMASK
        WFI                ; Wait for interrupt
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        B go_fatal_mode    ; Mine Field
        END
