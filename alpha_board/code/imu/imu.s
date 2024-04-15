;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                   imu.s                                    ;
;                                 IMU Driver                                 ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This file contains the initialization and operation code of a MPU-9250 IMU,
; using the SPI interface. This implementation also includes the capability
; of reading the accelerometer, gyroscope, and magnetometer data.
; 
; This file defines functions:
;		InitIMU
;		GetAccelX
;		GetAccelY
;		GetAccelZ
;		GetGyroX
;		GetGyroY
;		GetGyroZ
;		GetMagX
;		GetMagY
;		GetMagZ

; Revision History:
;		12/5/23	Adam Krivka		initial revision



; local includes
	.include "../std.inc"
	.include "imu_symbols.inc"

; import functions from other files
	.ref SSITransact

; export functions to other files
	.def InitIMU
	.def GetAccelX
	.def GetAccelY
	.def GetAccelZ
	.def GetGyroX
	.def GetGyroY
	.def GetGyroZ
	.def GetMagX
	.def GetMagY
	.def GetMagZ



; InitIMU
;
; Description:			Initializes the IMU in 9 DoF mode.
;
; Arguments:			None.
; Returns:				R0: FUNCTION_SUCCESS if the IMU was initialized successfully, FUNCTION_FAIL otherwise.
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       None.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          1
;
; Revision History:

InitIMU:
	PUSH	{LR}			; save return address

	; wait 100 ms before initializing the IMU
	MOV32		R0, IMU_WAIT_CLOCKS
InitIMUWait:
	SUBS		R0, #1
	BNE			InitIMUWait

	; configure the IMU to 
	;	- disable I2C slave interface (use SPI only instead)
	;	- enable I2C master interface
	;	- reset signal paths (clears all sensor registers)
	MOV		R0, #USER_CTRL_OFFSET
	MOV		R1, #(USER_CTRL_I2C_IF_DIS | USER_CTRL_I2C_MST_EN)
	BL		WriteIMUReg

	; check IMU device ID
	BL		CheckDeviceID
	CMP		R0, #FUNCTION_FAIL
	BEQ		InitIMUFail

	; check magnetometer device ID
	BL		CheckMagnetID
	CMP		R0, #FUNCTION_FAIL
	BEQ		InitIMUFail
	;B		InitIMUConfigure

InitIMUConfigure:
	; configure the accelerometer
	MOV		R0, #ACCEL_CONFIG1_OFFSET
	MOV		R1, #ACCEL_FS_SEL_4
	BL		WriteIMUReg

	; configure the gyroscope
	MOV		R0, #GYRO_CONFIG_OFFSET
	MOV		R1, #GYRO_FS_SEL_250
	BL		WriteIMUReg

	; configure I2C master interface
	MOV		R0, #I2C_MST_CTRL_OFFSET
	MOV		R1, #(I2C_MST_CLK_400 | I2C_MST_P_NSR_STOP)
	BL		WriteIMUReg

	; configure the magnetometer
	MOV		R0, #MAG_CNTL2_OFFSET
	MOV		R1, #MAG_SRST
	BL		WriteMagnetReg		; write to the CNTL2 register

InitIMUSuccess:
	MOV		R0, #FUNCTION_SUCCESS
	B		InitIMUDone

InitIMUFail:
	MOV		R0, #FUNCTION_FAIL
	;B		InitIMUDone

InitIMUDone:
	POP		{LR}			; restor return address
	BX 		LR				; return



; CheckDeviceID
;
; Description:			Checks the device ID of the IMU.
;
; Arguments:			None.
; Returns:				FUNCTION_SUCCESS if the device ID is correct, FUNCTION_FAIL otherwise.
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       If the device ID is incorrect, the function will return FUNCTION_FAIL.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          1
;
; Revision History:

CheckDeviceID:
	PUSH	{LR}			; save return address

	; read the device ID
	MOV		R0, #WHO_AM_I_OFFSET
	BL		ReadIMUReg

	; check the device ID
	CMP		R0, #WHO_AM_I_ID
	BEQ		CheckDeviceIDSuccess
	;B		CheckDeviceIDFail

CheckDeviceIDFail:
	MOV		R0, #FUNCTION_FAIL
	B		CheckDeviceIDDone

CheckDeviceIDSuccess:
	MOV		R0, #FUNCTION_SUCCESS
	;B		CheckDeviceIDDone

CheckDeviceIDDone:
	POP		{LR}			; restore return address
	BX		LR				; return



; ReadIMUReg
;
; Description:			Reads a register on the IMU.
;						Registers are 8 bits wide. 
;
; Arguments:			R0 = register address
; Returns:				R0 = register value
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       None.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          1
;
; Revision History:

ReadIMUReg:
	PUSH	{LR}						; save return address and used registers

	ORR		R0, #IMU_READ				; specify read operation
	LSL		R0, #IMU_WORD				; move address to the first byte

	BL		SSITransact

	POP		{LR}						; restore return address and used registers
	BX		LR							; return


; WriteIMUReg
;
; Description:			Writes to a register on the IMU.
;						Registers are 8 bits wide.
;
; Arguments:			R0 = 8-bit register address
;						R1 = 8-bit register value to be store
; Returns:				None.
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       None.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          1
;
; Revision History:

WriteIMUReg:
	PUSH	{LR}						; save return address and used registers

	ORR		R0, #IMU_WRITE				; specify write operation
	LSL		R0, #IMU_WORD				; move address to the first byte
	ORR		R0, R1						; combine address and value

	BL		SSITransact

	POP		{LR}						; restore return address and used registers
	BX		LR							; return

; WriteMagnetReg
;
; Description:			Writes a register to the magnetometer.
;						Registers are 8 bits wide.
;
; Arguments:			R0 = register address
;						R1 = register value
; Returns:				None.
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       None.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          1
;
; Revision History:

WriteMagnetReg:
	PUSH	{LR, R4, R5}					; save return address and used registers

	; save variables
	MOV		R4, R0	; register address
	MOV		R5, R1	; register value

	; configure an I2C slave for the magnetometer
	MOV		R0,	#I2C_SLV4_ADDR_OFFSET
	MOV		R1,	#(I2C_SLV4_WRITE | MAG_ADDR)
	BL		WriteIMUReg

	; set register address we want to read
	MOV		R0, #I2C_SLV4_REG_OFFSET
	MOV		R1, R4
	BL		WriteIMUReg

	; TEST REMOVE
	MOV		R0, #I2C_SLV4_REG_OFFSET
	BL		ReadIMUReg

	; set register value we want to write
	MOV		R0, #I2C_SLV4_DO_OFFSET
	MOV		R1, R5
	BL		WriteIMUReg

	; enable I2C slave 4 transfer
	MOV		R0, #I2C_SLV4_CTRL_OFFSET
	MOV		R1, #I2C_SLV4_EN
	BL		WriteIMUReg
	
	; wait for transfer to complete by reading the I2C master status
WriteMagnetRegWait:
	MOV		R0, #I2C_MST_STATUS_OFFSET
	BL		ReadIMUReg
	TST		R0, #I2C_SLV4_DONE		; check whether SLV4 bit is asserted
	BEQ		WriteMagnetRegWait		; if not, wait
	;B		WriteMagnetRegTransferDone		; if so, we're done

WriteMagnetRegTransferDone:
WriteMagnetRegDone:	
	POP		{LR, R4, R5}					; restore return address and used registers
	BX		LR						; return

; ReadMagnetReg
;
; Description:			Reads a register from the magnetometer.
;						Registers are 8 bits wide. 
;
; Arguments:			R0 = register address
; Returns:				R0 = register value
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       None.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          1
;
; Revision History:

ReadMagnetReg:
	PUSH	{LR, R4}						; save return address and used registers

	; save variables
	MOV		R4, R0	; register address

	; configure an I2C slave for the magnetometer for reading
	MOV		R0, #I2C_SLV4_ADDR_OFFSET
	MOV		R1, #((I2C_SLV4_READ | MAG_ADDR))
	BL		WriteIMUReg

	; set register address we want to read
	MOV		R0, #I2C_SLV4_REG_OFFSET
	MOV		R1, R4
	BL		WriteIMUReg

	; enable I2C slave 4 transfer
	MOV		R0, #I2C_SLV4_CTRL_OFFSET
	MOV		R1, #I2C_SLV4_EN
	BL		WriteIMUReg

	; wait for transfer to complete by reading the I2C master status
ReadMagnetRegWait:
	MOV		R0, #I2C_MST_STATUS_OFFSET
	BL		ReadIMUReg
	TST		R0, #I2C_SLV4_DONE		; check whether SLV4 bit is asserted
	BEQ		ReadMagnetRegWait		; if not, wait
	;B		ReadMagnetRegTransferDone		; if so, we're done

ReadMagnetRegTransferDone:
	; read the register value
	MOV		R0, #I2C_SLV4_DI_OFFSET
	BL		ReadIMUReg

	;B		ReadMagnetRegDone

ReadMagnetRegDone:
	POP		{LR, R4}						; restore return address and used registers
	BX		LR							; return



; ReadMagnetData
;
; Description:			Reads measurement from the magnetometer.
;
; Arguments:			R0 = measruement register address
; Returns:				R0 = data value
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       None.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          1
;
; Revision History:

ReadMagnetData:
	PUSH	{LR}					; save return address and used registers

	; save variables
	MOV		R4, R0	; register address

; trigger measurement by writing to the CNTL1 register
	MOV		R0, #MAG_CNTL1_OFFSET
	MOV		R1, #MAG_SINGLE_MEASUREMENT
	BL		WriteMagnetReg

; poll the status register
ReadMagnetDataWait:
	MOV		R0, #MAG_ST1_OFFSET
	BL		ReadMagnetReg
	TST		R0, #MAG_ST1_DRDY
	BEQ		ReadMagnetDataWait
	;B		ReadMagnetDataMeasurementDone

ReadMagnetDataMeasurementDone:
	; configure an I2C slave for the magnetometer for reading
	MOV		R0, #I2C_SLV4_ADDR_OFFSET
	MOV		R1, #((I2C_SLV4_READ | MAG_ADDR))
	BL		WriteIMUReg

	; set register address we want to read
	MOV		R0, #I2C_SLV4_REG_OFFSET
	MOV		R1, R4
	BL		WriteIMUReg

	; enable I2C slave 4 transfer
	MOV		R0, #I2C_SLV4_CTRL_OFFSET
	MOV		R1, #I2C_SLV4_EN
	BL		WriteIMUReg

	; wait for transfer to complete by reading the I2C master status
ReadMagnetDataWait2:
	MOV		R0, #I2C_MST_STATUS_OFFSET
	BL		ReadIMUReg
	TST		R0, #I2C_SLV4_DONE		; check whether SLV4 bit is asserted
	BEQ		ReadMagnetDataWait2		; if not, wait
	;B		ReadMagnetDataTransferDone		; if so, we're done

ReadMagnetDataTransferDone:
	; read the register value
	MOV		R0, #I2C_SLV4_DI_OFFSET
	BL		ReadIMUReg
	;B		ReadMagnetDataDone

ReadMagnetDataDone:
	POP		{LR}					; restore return address and used registers
	BX		LR						; return



; GetAccelX, GetAccelY, GetAccelZ
;
; Description:			Reads the X-axis, Y-axis, or Z-axis accelerometer value.
;
; Arguments:			None.
; Returns:				R0 = X-axis, Y-axis, or Z-axis accelerometer value
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       None.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          1
;
; Revision History:


; CheckMagnetID
;
; Description:			Checks the device ID of the magnetometer.
;
; Arguments:			None.
; Returns:				FUNCTION_SUCCESS if the device ID is correct, FUNCTION_FAIL otherwise.
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       If the device ID is incorrect, the function will return FUNCTION_FAIL.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          1
;
; Revision History:

CheckMagnetID:
	PUSH	{LR}			; save return address

	; read the device ID
	MOV		R0, #MAG_WHO_AM_I_OFFSET
	BL		ReadMagnetReg

	; check the device ID
	CMP		R0, #MAG_WHO_AM_I_ID
	BEQ		CheckMagnetIDSuccess
	;B		CheckMagnetIDFail

CheckMagnetIDFail:
	MOV		R0, #FUNCTION_FAIL
	B		CheckMagnetIDDone

CheckMagnetIDSuccess:
	MOV		R0, #FUNCTION_SUCCESS
	;B		CheckMagnetIDDone

CheckMagnetIDDone:
	POP		{LR}			; restore return address
	BX		LR				; return

GetAccel_MACRO .macro axis
	PUSH	{LR, R4}					; save return address and used registers
	MOV		R0, #(ACCEL_:axis:OUT_H_OFFSET)	; set register address
	BL		ReadIMUReg					; read register
	MOV		R4, R0						; save high byte
	LSL		R4, #8						
	MOV		R0, #(ACCEL_:axis:OUT_L_OFFSET)	; set register address
	BL		ReadIMUReg					; read register
	ORR		R0, R4						; combine high and low bytes
	POP		{LR, R4}					; restore return address and used registers
	BX		LR							; return
	.endm

GetAccelX:
	GetAccel_MACRO X

GetAccelY:
	GetAccel_MACRO Y

GetAccelZ:
	GetAccel_MACRO Z

; GetGyroX, GetGyroY, GetGyroZ
;
; Description:			Reads the X-axis, Y-axis, or Z-axis gyroscope value.
;
; Arguments:			None.
; Returns:				R0 = X-axis, Y-axis, or Z-axis gyroscope value
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       None.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          1
;
; Revision History:

GetGyro_MACRO .macro axis
	PUSH	{LR, R4}						; save return address and used registers
	MOV		R0, #(GYRO_:axis:OUT_H_OFFSET)	; set register address
	BL		ReadIMUReg					; read register
	MOV		R4, R0						; save high byte
	LSL		R4, #8
	MOV		R0, #(GYRO_:axis:OUT_L_OFFSET)	; set register address
	BL		ReadIMUReg					; read register
	ORR		R0, R4						; combine high and low bytes
	POP		{LR, R4}						; restore return address and used registers
	BX		LR							; return
	.endm

GetGyroX:
	GetGyro_MACRO X

GetGyroY:
	GetGyro_MACRO Y

GetGyroZ:
	GetGyro_MACRO Z



; GetMagX, GetMagY, GetMagZ
;
; Description:			Reads the X-axis, Y-axis, or Z-axis magnetometer value.
;
; Arguments:			None.
; Returns:				R0 = X-axis, Y-axis, or Z-axis magnetometer value
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       None.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          1
;
; Revision History:

GetMag_MACRO .macro axis
	PUSH	{LR, R4}						; save return address and used registers
	MOV		R0, #(MAG_:axis:OUT_H_OFFSET)	; set register address
	BL		ReadMagnetData				; read register
	MOV		R4, R0						; save high byte
	LSL		R4, #8
	MOV		R0, #(MAG_:axis:OUT_L_OFFSET)	; set register address
	BL		ReadMagnetData				; read register
	ORR		R0, R4						; combine high and low bytes
	POP		{LR, R4}						; restore return address and used registers
	BX		LR							; return
	.endm

GetMagX:
	GetMag_MACRO X

GetMagY:
	GetMag_MACRO Y

GetMagZ:
	GetMag_MACRO Z
