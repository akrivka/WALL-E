;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                            serial_symbols.inc                              ;
;                              Serial Symbols                                ;
;                               Include File                                 ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains symbols to configure the serial interface.
;
;
; Revision History:



; local includes
	.include "../imu_demo_symbols.inc"
	.include "../cc26x2r/ioc_reg.inc"
	.include "../cc26x2r/ssi_reg.inc"

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; PINS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

TX_PIN .equ			SSI_TX_PIN			; Serial Transmit Pin
RX_PIN .equ			SSI_RX_PIN			; Serial Receive Pin
CLK_PIN .equ		SSI_CLK_PIN			; Serial Clock Pin
CS_PIN .equ			SSI_CS_PIN			; Serial Chip Select Pin

TX_PIN_CFG .equ		IO_PORT_ID_SSI1_TX | IO_NOPUPD	; Serial Transmit Pin Configuration
RX_PIN_CFG .equ		IO_PORT_ID_SSI1_RX | IO_INPUT | IO_NOPUPD	; Serial Receive Pin Configuration
CLK_PIN_CFG .equ	IO_PORT_ID_SSI1_CLK | IO_NOPUPD	; Serial Clock Pin Configuration
CS_PIN_CFG .equ		IO_PORT_ID_SSI1_FSS	| IO_NOPUPD ; Serial Chip Select Pin Configuration

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; SSI
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

SSI_BASE_ADDR .equ		SSI1_BASE_ADDR			; Serial Base Address
SSI_CR0 .equ			CR0_DSS_16_BIT			; Serial Control Register 0
SSI_CR1 .equ			CR1_SSE_ENABLE			; Serial Control Register 1
SSI_CR1_DISABLE .equ    CR1_SSE_DISABLE
SSI_CPSR .equ			48						; Serial Clock Prescale Register

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; OTHER
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

SSI_MASK .equ			0x000000FF				; Serial Mask