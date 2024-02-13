;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                              imu_symbols.inc                               ;
;                                IMU Symbols                                 ;
;                               Include File                                 ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains symbols to configure an MPU-9250 IMU.
;
; Revision History:

; general
IMU_WRITE .equ              00000000b           ; write bit
IMU_READ .equ               10000000b           ; read bit
IMU_WORD .equ               8                   ; length of IMU address or data
IMU_MASK .equ				11111111b			; mask for IMU address or data
IMU_WAIT_CLOCKS .equ		4800000				; number of clocks to wait before initializing

; main register offsets
GYRO_CONFIG_OFFSET .equ					0x1B		; gyroscope configuration register
ACCEL_CONFIG1_OFFSET .equ				0x1C		; accelerometer configuration register 1

ACCEL_XOUT_H_OFFSET .equ 			0x3B		; accelerometer X-axis high byte
ACCEL_XOUT_L_OFFSET .equ 			0x3C		; accelerometer X-axis low byte
ACCEL_YOUT_H_OFFSET .equ 			0x3D		; accelerometer Y-axis high byte
ACCEL_YOUT_L_OFFSET .equ 			0x3E		; accelerometer Y-axis low byte
ACCEL_ZOUT_H_OFFSET .equ 			0x3F		; accelerometer Z-axis high byte
ACCEL_ZOUT_L_OFFSET .equ 			0x40		; accelerometer Z-axis low byte 
GYRO_XOUT_H_OFFSET .equ 			0x43		; gyroscope X-axis high byte
GYRO_XOUT_L_OFFSET .equ 			0x44		; gyroscope X-axis low byte
GYRO_YOUT_H_OFFSET .equ 			0x45		; gyroscope Y-axis high byte
GYRO_YOUT_L_OFFSET .equ 			0x46		; gyroscope Y-axis low byte
GYRO_ZOUT_H_OFFSET .equ 			0x47		; gyroscope Z-axis high byte
GYRO_ZOUT_L_OFFSET .equ 			0x48		; gyroscope Z-axis low byte
USER_CTRL_OFFSET .equ               0x6A        ; user control register
I2C_MST_CTRL_OFFSET .equ			0x24		; I2C master control register

I2C_SLV4_ADDR_OFFSET .equ			0x31		; I2C slave 4 address register
I2C_SLV4_REG_OFFSET .equ			0x32		; I2C slave 4 register address register
I2C_SLV4_DO_OFFSET .equ				0x33		; I2C slave 4 data out register
I2C_SLV4_CTRL_OFFSET .equ			0x34		; I2C slave 4 control register
I2C_SLV4_DI_OFFSET .equ				0x35		; I2C slave 4 data in register

I2C_MST_STATUS_OFFSET .equ			0x36		; I2C master status register

WHO_AM_I_OFFSET .equ				0x75		; who am I register


; register values
WHO_AM_I_ID .equ					0x71		; who am I register value

GYRO_FS_SEL_250 .equ				0x00 << 3		; gyroscope full scale range 250 deg/s
GYRO_FS_SEL_500 .equ				0x01 << 3		; gyroscope full scale range 500 deg/s
GYRO_FS_SEL_1000 .equ				0x02 << 3		; gyroscope full scale range 1000 deg/s
GYRO_FS_SEL_2000 .equ				0x03 << 3		; gyroscope full scale range 2000 deg/s

ACCEL_FS_SEL_2 .equ					0x00 << 3		; accelerometer full scale range 2g
ACCEL_FS_SEL_4 .equ					0x01 << 3		; accelerometer full scale range 4g
ACCEL_FS_SEL_8 .equ					0x02 << 3		; accelerometer full scale range 8g
ACCEL_FS_SEL_16 .equ				0x03 << 3		; accelerometer full scale range 16g

USER_CTRL_I2C_IF_DIS .equ       0x1 << 4    ; Reset I2C Slave module and put the serial interface in SPI mode only.
USER_CTRL_I2C_MST_EN .equ       0x1 << 5    ; Enable the I2C Master I/F module
USER_CTRL_SIG_COND_RST .equ     0x1         ; Reset accelerometer, gyroscope, and temperature sensors

I2C_MST_CLK_348 .equ   		0        ; 348kHz I2C master clock
I2C_MST_CLK_400 .equ   		13       ; 400kHz I2C master clock

I2C_MST_P_NSR_STOP .equ		0x1 << 4 ; Stop between reads

I2C_SLV4_EN .equ					0x1 << 7 ; Enable slave 4

I2C_SLV4_DONE .equ				0x1 << 6 ; Slave 4 done

I2C_SLV4_READ .equ				0x1 << 7 ; Read transfer
I2C_SLV4_WRITE .equ				0x0 << 7 ; Write transfer

; magnetometer
; MPU configurtion
MAG_ADDR .equ					0x0C		; magnetometer address

; register offsets
MAG_XOUT_L_OFFSET .equ 			0x03		; magnetometer X-axis high byte
MAG_XOUT_H_OFFSET .equ 			0x04		; magnetometer X-axis low byte
MAG_YOUT_L_OFFSET .equ 			0x05		; magnetometer Y-axis high byte
MAG_YOUT_H_OFFSET .equ 			0x06		; magnetometer Y-axis low byte
MAG_ZOUT_L_OFFSET .equ 			0x07		; magnetometer Z-axis high byte
MAG_ZOUT_H_OFFSET .equ 			0x08		; magnetometer Z-axis low byte

MAG_CNTL1_OFFSET .equ			0x0A		; magnetometer control register
MAG_CNTL2_OFFSET .equ			0x0B		; magnetometer control register
MAG_ST1_OFFSET .equ				0x02		; magnetometer status register

MAG_WHO_AM_I_OFFSET .equ		0x00		; magnetometer who am I register

; register values
MAG_WHO_AM_I_ID .equ			0x48		; magnetometer who am I register value
MAG_SINGLE_MEASUREMENT .equ		0x01		; single measurement mode

MAG_SRST .equ					0x01		; soft reset bit

MAG_ST1_DRDY .equ				0x01		; data ready bit
MAG_ST1_DOR .equ				0x02		; data overrun bit
