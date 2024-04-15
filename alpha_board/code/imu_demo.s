;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                            EE 110b - Homework #1                           ;
;                                  IMU Demo                                  ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This file contains a program which demos the IMU functionality, defined
; in subfolder imu/. The demo can test either the accelerometer and the 
; gyroscope functionality, or the magnetometer functionality (switch
; the test function in the main loop). Both tests display the current
; x,y,z values on the LCD
;
; Revision History: 
debounce


; local include files
    .include "imu_demo_symbols.inc"
    .include "std.inc"

    .include "cc26x2r/gpt_reg.inc"
    .include "cc26x2r/gpio_reg.inc"
    .include "cc26x2r/cpu_scs_reg.inc"

; import symbols from other files
    .ref PeriphPowerInit
    .ref GPIOClockInit
    .ref StackInit
    .ref GPTClockInit
    .ref SSIClockInit
	.ref MoveVecTable

	.ref InitIMU
    .ref LCDInit
    .ref InitSSI
    .ref TestIMUAccelGyro
    .ref TestIMUMagnet
    .ref i16ToString
    .ref Display


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; MAIN CODE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	.text
; main
;
; Description:          Main loop of the Servo Demo program. Initializes
;                       the board, the Servo, and the LCD, and tests the 
;                       Servo using functions from servo/servo_test.s
;
; Arguments:            None.
; Return Values:        None.
; 
; Local Variables:      In nested functions...
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       Facilitated using FUNCTION_SUCCESS and FUNCTION_FAIL
;                       in called functions.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          5?
; 
; Revision History:
;		12/5/23	Adam Krivka		initial revision


    .global ResetISR					; expose the program entry-point
ResetISR:
main:
    BL      PeriphPowerInit				; turn on peripheral power domain
    BL      GPIOClockInit				; turn on GPIO clock
    BL      GPTClockInit				; turn on GPT clock
    BL      SSIClockInit				; turn on GPT clock
    BL      StackInit					; initialize stack in SRAM
	BL		MoveVecTable				; move interrupt vector table

; initialize IMU
	BL		InitSSI					    ; initialize Serial Interface

	BL		InitIMU 					; initialize IMU
	CMP		R0, #FUNCTION_FAIL			; check if initialization succeeded
	BEQ		Error

    BL      LCDInit						; initialize LCD

; test IMU
;    BL      TestIMUAccelGyro            ; test accelerometer and gyroscope
    BL      TestIMUMagnet               ; test magnetometer

; infinite loop
Loop:
    B       Loop						; infinite loop

Error:
