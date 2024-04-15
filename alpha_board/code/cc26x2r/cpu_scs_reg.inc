;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                               cpu_scs_reg.inc                              ;
;                         Core System Control Space Constants                ;
;                                 Include File                               ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains the constants for the CPU Core System Control Space 
; for the TI CC2652 microcontroller.
;
; Revision History:
;     11/7/23  Adam Krivka      initial revision


; base addresses
SCS_BASE_ADDR         .equ    0xE000E000  ; base address of the CPU SCS

; register offsets
SCS_NVIC_ISER0_OFFSET .equ    0x100       ; interrupt set enable (irq 0-31)
SCS_VTOR_OFFSET       .equ    0xD08       ; vector table offset register

; other constants
VEC_TABLE_SIZE        .equ    54          ; size of the vector table
VEC_TABLE_ALIGN       .equ    512         ; alignment of the vector table