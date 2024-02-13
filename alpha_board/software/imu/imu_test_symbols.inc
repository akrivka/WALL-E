;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                           imu_test_symbols.inc                             ;
;                             IMU Test Symbols                               ;
;                               Include File                                 ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains symbols for the tests of the MPU-9250 IMU.
;
; Revision History:



; local includes 
	.include "../cc26x2r/gpt_reg.inc"
	.include "../imu_demo_symbols.inc"

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; DISPLAY LOCATIONS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; X acceleration at (0,0)
ACCEL_X_ROW .equ		0
ACCEL_X_COL .equ		0

; Y acceleration at (1,0)
ACCEL_Y_ROW .equ		1
ACCEL_Y_COL .equ		0

; Z acceleration at (2,0)
ACCEL_Z_ROW .equ		2
ACCEL_Z_COL .equ		0

; X gyro at (0,8)
GYRO_X_ROW .equ			0
GYRO_X_COL .equ			8

; Y gyro at (1,8)
GYRO_Y_ROW .equ			1
GYRO_Y_COL .equ			8

; Z gyro at (2,8)
GYRO_Z_ROW .equ			2
GYRO_Z_COL .equ			8

; X mag at (0,0)
MAG_X_ROW .equ			0
MAG_X_COL .equ			0

; Y mag at (1,0)
MAG_Y_ROW .equ			1
MAG_Y_COL .equ			0

; Z mag at (2,0)
MAG_Z_ROW .equ			2
MAG_Z_COL .equ			0

; display length
DISPLAY_LENGTH .equ		8

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; TEST TIMER
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

TESTTIMER_BASE_ADDR .equ	IMU_TESTTIMER_BASE_ADDR			; test timer base address
TESTTIMER_IRQ_NUMBER .equ	IMU_TESTTIMER_IRQ_NUMBER		; interrupt number
TESTTIMER_EXCEPTION_NUMBER .equ IMU_TESTTIMER_EXCEPTION_NUMBER ; exception number
TESTTIMER_CFG .equ			GPT_CFG_32BIT			; only timer A
TESTTIMER_IMR .equ			GPT_IMR_TATOIM_ENABLED	; enable interrupt
TESTTIMER_ENABLE .equ		GPT_CTL_TAEN_ENABLED	; timer A enable
TESTTIMER_TAMR .equ			GPT_TXMR_PERIODIC		; periodic

; 4 Hz frequency
; 48 000 000 / 4 = 6 000 000
TESTTIMER_LENGTH .equ		(48000000/2)
TESTTIMER_TAILR .equ		(TESTTIMER_LENGTH)      ; interval
TESTTIMER_TAPR .equ			0                     	; prescale

