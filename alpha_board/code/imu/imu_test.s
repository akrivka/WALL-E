;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                imu_test.s                                  ;
;                               IMU Test Code                                ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This file contains the function:
;   TestIMUAccelGyro
;	TestIMUMagnet
; which tests IMU functionality, defined in imu.s.
; 
; Revision History: 



; local includes
	.include "../std.inc"
	.include "imu_test_symbols.inc"
	.include "../cc26x2r/gpio_reg.inc"
	.include "../cc26x2r/gpt_reg.inc"
	.include "../cc26x2r/event_reg.inc"
	.include "../cc26x2r/aux_reg.inc"
	.include "../cc26x2r/cpu_scs_reg.inc"

; import functions from other files
	.ref InitIMU
	.ref GetAccelX
	.ref GetAccelY
	.ref GetAccelZ
	.ref GetGyroX
	.ref GetGyroY
	.ref GetGyroZ
	.ref GetMagX
	.ref GetMagY
	.ref GetMagZ
	.ref ClearDisplay
	.ref Display
	.ref i16ToString

; export functions to other files
	.def TestIMUAccelGyro
	.def TestIMUMagnet


; TestIMUAccelGyroShowOnLCD
;
; Description:			Reads the accelerometer and gyroscope X, Y, Z values and
;						displays them on the LCD like this:
;						| Accel X | Gyro X |
;						| Accel Y | Gyro Y |
;						| Accel Z | Gyro Z |
;						| 				   |
;
; Arguments:			None
; Returns:				None
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Notes:				This function is called by the 60Hz interrupt handler.
;						It is not meant to be called by the user.
;
; Error Handling:		None.
;
; Revision History:

TestIMUAccelGyroShowOnLCD:
	PUSH	{LR, R4}						; save return address and used registers

; create local 8-byte string buffer
	SUBS	R13, #8			
	MOV		R4, R13						; store pointer to it in R4

	BL		GetAccelX					; get accelerometer X value
	MOV		R1, R4						; string buffer pointer
	BL		i16ToString					; convert to string
	MOV		R0, #ACCEL_X_ROW			; set row
	MOV		R1, #ACCEL_X_COL			; set column
	MOV		R2, R4						; string buffer pointer
	MOV		R3, #DISPLAY_LENGTH			; display exactly DISPLAY_LENGTH characters
	BL		Display						; display string

	BL		GetGyroX					; get gyroscope X value
	MOV		R1, R4						; string buffer pointer
	BL		i16ToString					; convert to string
	MOV		R0, #GYRO_X_ROW				; set row
	MOV		R1, #GYRO_X_COL				; set column
	MOV		R2, R4						; string buffer pointer
	MOV		R3, #DISPLAY_LENGTH			; display exactly DISPLAY_LENGTH characters
	BL		Display						; display string

	BL		GetAccelY					; get accelerometer Y value
	MOV		R1, R4						; string buffer pointer
	BL		i16ToString					; convert to string
	MOV		R0, #ACCEL_Y_ROW			; set row
	MOV		R1, #ACCEL_Y_COL			; set column
	MOV		R2, R4						; string buffer pointer
	MOV		R3, #DISPLAY_LENGTH			; display exactly DISPLAY_LENGTH characters
	BL		Display						; display string

	BL		GetGyroY					; get gyroscope Y value
	MOV		R1, R4						; string buffer pointer
	BL		i16ToString					; convert to string
	MOV		R0, #GYRO_Y_ROW				; set row
	MOV		R1, #GYRO_Y_COL				; set column
	MOV		R2, R4						; string buffer pointer
	MOV		R3, #DISPLAY_LENGTH			; display exactly DISPLAY_LENGTH characters
	BL		Display						; display string

	BL		GetAccelZ					; get accelerometer Z value
	MOV		R1, R4						; string buffer pointer
	BL		i16ToString					; convert to string
	MOV		R0, #ACCEL_Z_ROW			; set row
	MOV		R1, #ACCEL_Z_COL			; set column
	MOV		R2, R4						; string buffer pointer
	MOV		R3, #DISPLAY_LENGTH			; display exactly DISPLAY_LENGTH characters
	BL		Display						; display string

	BL		GetGyroZ					; get gyroscope Z value
	MOV		R1, R4						; string buffer pointer
	BL		i16ToString					; convert to string
	MOV		R0, #GYRO_Z_ROW				; set row
	MOV		R1, #GYRO_Z_COL				; set column
	MOV		R2, R4						; string buffer pointer
	MOV		R3, #DISPLAY_LENGTH			; display exactly DISPLAY_LENGTH characters
	BL		Display						; display string

	; Clear interrupt
	MOV32	R1, TESTTIMER_BASE_ADDR	; prepare timer base address
	STREG	GPT_ICLR_TATOCINT_CLEAR, R1, GPT_ICLR_OFFSET	; clear Timer A Time-out bit

	ADD		R13, #8						; return stack pointer
	POP		{LR, R4}						; restore return address and used registers
	BX		LR							; return


; TestIMUMagnetShowOnLCD
;
; Description:			Reads the magnetometer X, Y, Z values and
;						displays them on the LCD like this:
;						| Mag X |
;						| Mag Y |
;						| Mag Z |
;						| 		|
;
; Arguments:			None
; Returns:				None
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Notes:				This function is called by the 60Hz interrupt handler.
;						It is not meant to be called by the user.
;
; Error Handling:		None.
;
; Revision History:

TestIMUMagnetShowOnLCD:
	PUSH	{LR, R4}						; save return address and used registers

; create local 8-byte string buffer
	SUBS	R13, #8			
	MOV		R4, R13						; store pointer to it in R4

	BL		GetMagX						; get magnetometer X value
	MOV		R1, R4						; string buffer pointer
	BL		i16ToString					; convert to string
	MOV		R0, #MAG_X_ROW				; set row
	MOV		R1, #MAG_X_COL				; set column
	MOV		R2, R4						; string buffer pointer
	MOV		R3, #DISPLAY_LENGTH			; display exactly DISPLAY_LENGTH characters
	BL		Display						; display string

	BL		GetMagY						; get magnetometer Y value
	MOV		R1, R4						; string buffer pointer
	BL		i16ToString					; convert to string
	MOV		R0, #MAG_Y_ROW				; set row
	MOV		R1, #MAG_Y_COL				; set column
	MOV		R2, R4						; string buffer pointer
	MOV		R3, #DISPLAY_LENGTH			; display exactly DISPLAY_LENGTH characters
	BL		Display						; display string

	BL		GetMagZ						; get magnetometer Z value
	MOV		R1, R4						; string buffer pointer
	BL		i16ToString					; convert to string
	MOV		R0, #MAG_Z_ROW				; set row
	MOV		R1, #MAG_Z_COL				; set column
	MOV		R2, R4						; string buffer pointer
	MOV		R3, #DISPLAY_LENGTH			; display exactly DISPLAY_LENGTH characters
	BL		Display						; display string

	; Clear interrupt
	MOV32	R1, TESTTIMER_BASE_ADDR	; prepare timer base address
	STREG	GPT_ICLR_TATOCINT_CLEAR, R1, GPT_ICLR_OFFSET	; clear Timer A Time-out bit

	ADD		R13, #8						; return stack pointer
	POP		{LR, R4}					; restore return address and used registers
	BX		LR							; return

; TestIMUAccelGyro
; 
; Description:			Tests the IMU accelerometer and gyroscope, printing
;						the values to the LCD in a grid fashion. It sets
;						a 60Hz interrupt to update the values and then returns
;						(the caller is responsible for looping).
;
; Arguments:			None
; Returns:				None
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.

TestIMUAccelGyro:
	PUSH	{LR}						; save return address and used registers

; set up interrupt to read IMU and update LCD
	MOV32	R1, TESTTIMER_BASE_ADDR		; prapre test timer base address
	STREG	TESTTIMER_CFG, R1, GPT_CFG_OFFSET
	STREG	TESTTIMER_IMR, R1, GPT_IMR_OFFSET
	STREG	TESTTIMER_TAMR, R1, GPT_TAMR_OFFSET
	STREG	TESTTIMER_TAILR, R1, GPT_TAILR_OFFSET
	STREG	TESTTIMER_TAPR, R1, GPT_TAPR_OFFSET

	STREG	TESTTIMER_ENABLE, R1, GPT_CTL_OFFSET	; enable timer

	; Set up interrupt in CPU
	MOV32	R1, SCS_BASE_ADDR
	STREG	(0x1 << TESTTIMER_IRQ_NUMBER), R1, SCS_NVIC_ISER0_OFFSET ; enable interrupt
	LDR		R1, [R1, #SCS_VTOR_OFFSET] 		; load VTOR address
	MOVA	R0, TestIMUAccelGyroShowOnLCD		; load event handler address
	STR		R0, [R1, #(BYTES_PER_WORD * TESTTIMER_EXCEPTION_NUMBER)] ; store event handler

	POP		{LR}						; restore return address and used registers
	BX		LR							; return


; TestIMUMagnet
;
; Description:			Reads the magnetometer X, Y, Z values, printing
;						the values to the LCD in a grid fashion. It sets
;						a 60Hz interrupt to update the values and then returns
;						(the caller is responsible for looping).
;
; Arguments:			None
; Returns:				None
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;

TestIMUMagnet:
	PUSH	{LR}						; save return address and used registers

; set up interrupt to read IMU and update LCD
	MOV32	R1, TESTTIMER_BASE_ADDR		; prapre test timer base address
	STREG	TESTTIMER_CFG, R1, GPT_CFG_OFFSET
	STREG	TESTTIMER_IMR, R1, GPT_IMR_OFFSET
	STREG	TESTTIMER_TAMR, R1, GPT_TAMR_OFFSET
	STREG	TESTTIMER_TAILR, R1, GPT_TAILR_OFFSET
	STREG	TESTTIMER_TAPR, R1, GPT_TAPR_OFFSET

	STREG	TESTTIMER_ENABLE, R1, GPT_CTL_OFFSET	; enable timer

	; Set up interrupt in CPU
	MOV32	R1, SCS_BASE_ADDR
	STREG	(0x1 << TESTTIMER_IRQ_NUMBER), R1, SCS_NVIC_ISER0_OFFSET ; enable interrupt
	LDR		R1, [R1, #SCS_VTOR_OFFSET] 		; load VTOR address
	MOVA	R0, TestIMUMagnetShowOnLCD		; load event handler address
	STR		R0, [R1, #(BYTES_PER_WORD * TESTTIMER_EXCEPTION_NUMBER)] ; store event handler

	POP		{LR}						; restore return address and used registers
	BX		LR							; return