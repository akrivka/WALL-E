;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                 gpio_reg.inc                               ;
;                            GPIO Register Constants                         ;
;                                 Include File                               ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains the constants for the GPIO control registers for the TI
; CC2652 microcontroller.
;
; Revision History:
;     11/7/23  Adam Krivka      initial revision


; base address

GPIO_BASE_ADDR  .equ            0x40022000  ; base address of GPIO registers


; register offsets 

GPIO_DOUT_OFFSET     .equ        0x80       ; data out register 0 to 31
GPIO_DIN_OFFSET      .equ        0xC0       ; data in register 0 to 31
GPIO_DOE_OFFSET      .equ        0xD0       ; data out enable register 0 to 31
GPIO_DOUTTGL_OFFSET  .equ        0xB0       ; data out toggle register 0 to 31
