;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                             imu_demo_symbols.inc                           ;
;                             Symbols for IMU Demo                           ;
;                                Include File                                ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This file contains the symbols used by the IMU demo.
;
; Revision History:



; local includes
    .include "cc26x2r/ioc_reg.inc"
    .include "cc26x2r/gpt_reg.inc"


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; IMU Serial Port
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Serial pins
SSI_TX_PIN .equ			    0			; Serial Transmit Pin
SSI_RX_PIN .equ			    1			; Serial Receive Pin
SSI_CLK_PIN .equ		    4			; Serial Clock Pin
SSI_CS_PIN .equ			    3			; Serial Chip Select Pin

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; IMU Test Timer
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

IMU_TESTTIMER_BASE_ADDR .equ    GPT0_BASE_ADDR
IMU_TESTTIMER_IRQ_NUMBER .equ   GPT0A_IRQ_NUMBER
IMU_TESTTIMER_EXCEPTION_NUMBER .equ GPT0A_EXCEPTION_NUMBER

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; LCD
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; LCD data bus
LCD_DATA_0_PIN .equ         15
LCD_DATA_1_PIN .equ         16
LCD_DATA_2_PIN .equ         17
LCD_DATA_3_PIN .equ         18
LCD_DATA_4_PIN .equ         19
LCD_DATA_5_PIN .equ         20
LCD_DATA_6_PIN .equ         21
LCD_DATA_7_PIN .equ         22

; LCD control pins
LCD_RS_PIN .equ             6   ; register select
LCD_RW_PIN .equ             5   ; read/write
LCD_E_PIN .equ              7   ; enable

; LCD timer
LCD_TIMER_BASE_ADDR .equ    GPT1_BASE_ADDR
