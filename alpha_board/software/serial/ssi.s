;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                  serial.s                                  ;
;                              Serial Interface                              ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This file contains functions for interfacing with periphreals using the 
; SPI interface.
; 
; This file defines functions:
;		SSITransact - sends and receives data over the SPI interface
; 
; Revision History:



; local includes
	.include "../std.inc"
	.include "serial_symbols.inc"
	.include "../cc26x2r/gpio_reg.inc"
	.include "../cc26x2r/gpt_reg.inc"
	.include "../cc26x2r/event_reg.inc"
	.include "../cc26x2r/aux_reg.inc"
	.include "../cc26x2r/ssi_reg.inc"

; export functions to other files
	.def InitSSI
	.def SSITransact



; InitSSSI
;
; Description:          Initializes the SSI interface.
;
; Arguments:            None.
; Return Values:        None.
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

InitSSI:
	PUSH	{LR}						; save return address and used registers

	; configure pins
	MOV32	R1, IOC_BASE_ADDR			; prepare IOC base address
	STREG	TX_PIN_CFG, R1, IOCFG_REG_SIZE * TX_PIN ; configure TX pin
	STREG	RX_PIN_CFG, R1, IOCFG_REG_SIZE * RX_PIN ; configure RX pin
	STREG	CLK_PIN_CFG, R1, IOCFG_REG_SIZE * CLK_PIN ; configure CLK pin
	STREG	CS_PIN_CFG, R1, IOCFG_REG_SIZE * CS_PIN ; configure CS pin

	; configure SSI module
	MOV32	R1, SSI_BASE_ADDR			; prepare SSI0 base address
	STREG	SSI_CR1_DISABLE, R1, CR1_OFFSET
	STREG	SSI_CPSR, R1, CPSR_OFFSET	; configure SSI0 CPSR
	STREG	SSI_CR0, R1, CR0_OFFSET		; configure SSI0 CR0
	STREG	SSI_CR1, R1, CR1_OFFSET		; configure SSI0 CR1

    POP     {LR}				        ; restore return address and used registers
    BX      LR                          ; return


; SSITransact
;
; Description:          Sends data over the serial interface.
;
; Arguments:            R0 = data to send.
; Return Values:        None.
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

SSITransact:
	PUSH	{LR}						; save return address 

	MOV32		R1, SSI_BASE_ADDR			; prepare SSI base address

	LDR		R2, [R1, #SR_OFFSET]		; load status register
	TST		R2, #SR_TNF_NOTFULL			; test if transmit FIFO is not full
	BEQ		SSITransactFail				; if transmit FIFO is full, fail
	;B		SSITransactSend				; if transmit FIFO is not full, continue with transaction

SSITransactSend:
	STR		R0, [R1, #DR_OFFSET]		; send data
	;B		SSITransactWait				; wait for data to be present in receive FIFO

SSITransactWait:
	LDR		R2, [R1, #SR_OFFSET]		; load status register
	TST		R2, #SR_RNE_NOTEMPTY		; test if receive FIFO is not empty
	BEQ		SSITransactWait				; if empty, return loop
	;B		SSITransactGet

SSITransactGet:
	LDR		R0, [R1, #DR_OFFSET]		; get data
	AND		R0, R0, #SSI_MASK			; mask data
	B		SSITransactDone				; return data

SSITransactFail:
	MOV		R0, #FUNCTION_FAIL			; return function fail
	;B		SSITransactDone

SSITransactDone:
	POP		{LR}						; restore return address
	BX		LR							; return

