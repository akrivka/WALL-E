;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                 ioc_reg.inc                                ;
;                            IO Control Constants                            ;
;                                 Include File                               ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains the constants for the IOC registers for the TI
; CC2652 microcontroller.
;
; Revision History:
;    	11/7/23		Adam Krivka		initial revision
;		12/5/23		Adam Krivka		added port ids
;		1/12/24		Adam Krivka		added more port ids


; base address
IOC_BASE_ADDR .equ             0x40081000  ; base address of IOC registers


; register values
IO_PD .equ                    0x1 << 13   ; pull down
IO_PU .equ                    0x2 << 13   ; pull up
IO_NOPUPD .equ                0x3 << 13   ; no pull up or down
IO_INPUT .equ                 0x1 << 29   ; input pin

IO_PORT_ID_GPIO .equ			0x0			; General Purpose IO
IO_PORT_ID_AON .equ				0x7			; AON 32 KHz clock (SCLK_LF)
IO_PORT_ID_AUXIO .equ			0x8			; AUX IO
IO_PORT_ID_SSI0_RX .equ			0x9			; SSI0 RX
IO_PORT_ID_SSI0_TX .equ			0xA			; SSI0 TX
IO_PORT_ID_SSI0_FSS .equ		0xB			; SSI0 FSS
IO_PORT_ID_SSI0_CLK .equ		0xC			; SSI0 CLK
IO_PORT_ID_I2C_MSSDA .equ		0xD			; I2C Master SDA
IO_PORT_ID_I2C_MSSCL .equ		0xE			; I2C Master SCL
IO_PORT_ID_UART0_RX .equ		0xF			; UART0 RX
IO_PORT_ID_UART0_TX .equ		0x10		; UART0 TX
IO_PORT_ID_UART0_CTS .equ		0x11		; UART0 CTS
IO_PORT_ID_UART0_RTS .equ		0x12		; UART0 RTS
IO_PORT_ID_UART1_RX .equ		0x13		; UART1 RX
IO_PORT_ID_UART1_TX .equ		0x14		; UART1 TX
IO_PORT_ID_UART1_CTS .equ		0x15		; UART1 CTS
IO_PORT_ID_UART1_RTS .equ		0x16		; UART1 RTS
IO_PORT_ID_EVENT0 .equ			0x17		; PORT EVENT 0
IO_PORT_ID_EVENT1 .equ			0x18		; PORT EVENT 1
IO_PORT_ID_EVENT2 .equ			0x19		; PORT EVENT 2
IO_PORT_ID_EVENT3 .equ			0x1A		; PORT EVENT 3
IO_PORT_ID_EVENT4 .equ			0x1B		; PORT EVENT 4
IO_PORT_ID_EVENT5 .equ			0x1C		; PORT EVENT 5
IO_PORT_ID_EVENT6 .equ			0x1D		; PORT EVENT 6
IO_PORT_ID_EVENT7 .equ			0x1E		; PORT EVENT 7
IO_PORT_ID_CPU_SWV .equ 	   	0x20        ; CPU SWV
IO_PORT_ID_SSI1_RX .equ			0x21		; SSI1 RX
IO_PORT_ID_SSI1_TX .equ			0x22		; SSI1 TX
IO_PORT_ID_SSI1_FSS .equ		0x23		; SSI1 FSS
IO_PORT_ID_SSI1_CLK .equ		0x24		; SSI1 CLK
IO_PORT_ID_I2S_AD0 .equ			0x25		; I2S AD0
IO_PORT_ID_I2S_AD1 .equ			0x26		; I2S AD1
IO_PORT_ID_I2S_WCLK .equ		0x27		; I2S WCLK
IO_PORT_ID_I2S_BCLK .equ		0x28		; I2S BCLK
IO_PORT_ID_I2S_MCLK .equ		0x29		; I2S MCLK


; common pin settings
IOCFG_GENERIC_INPUT .equ      IO_NOPUPD | IO_INPUT
IOCFG_GENERIC_OUTPUT .equ     IO_NOPUPD


; other constants
IOCFG_REG_SIZE .equ    0x4
