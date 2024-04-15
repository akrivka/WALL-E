;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                               conversions.s                                ;
;                          Various type conversions                          ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This files contains functions that convert between various types:
;   i16ToString - convert a 16-bit integer to a string
;
; Revision History:

; local includes
	.include "ascii.inc"

; export functions to other files
	.def i16ToString



; i16ToString
;
; Description:          Converts a 16-bit integer to a string.
;
; Arguments:            R0 = integer to convert, R1 = string buffer pointer (must
;						exactly 8 bytes).
; Return Values:        None (string in buffer pointed by R1)
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       None.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          13
; 
; Revision History:

ASR_LENGTH	.equ	31
BUFFER_SIZE	.equ	8
WORD_SIZE	.equ	4
BASE		.equ	10
i16ToString:
	PUSH	{LR, R4, R5, R6, R7}		; save return address and used registers

; save variables
	MOV		R4, R0						; integer to convert
	MOV		R5, R1						; string buffer pointer

; sign extend to 32 bits
	SXTH	R4, R4						; R4 = R4[15:0] sign extended to 32 bits

; check if negative
	ASR		R6, R4, #ASR_LENGTH			; R6 = R4[31] (-1 if negative, 0 if positive)

; take absolute value
	EOR		R4, R4, R6					; R4 = R4 ^ R6 (R4 = R4 if positive, R4 = -R4 - 1 if negative)
	SUB		R4, R4, R6					; R4 = R4 - R6 (R4 = R4 if positive, R4 = -R4 if negative)

; clean buffer (set to zeros)
	MOV		R2, #0
	STR		R2, [R1], #WORD_SIZE				; set first word to zero
	STR		R2, [R1], #WORD_SIZE				; set second word to zero

; create zeroed local buffer of 8 bytes on the stack
	STR		R2, [R13, #(-WORD_SIZE)]!
	STR		R2, [R13, #(-WORD_SIZE)]!
	MOV		R7, R13						; store its top in a variable

	MOV		R3, #BASE					; prepare base 10
i16ToStringConversionLoop:
; divide by 10 and get remainder
	SDIV	R0, R4, R3					; R0 = R4 / 10 (rounded towards zero)
	MLS		R1, R0, R3, R4				; R1 = R4 - R0*10 (R1 = R4 % 10)

	ADD		R1, #ASCII_ZERO				; convert to ASCII

	STRB	R1, [R7], #1				; store in stack buffer

	CMP		R0, #0						; if result was zero, this was the last digit
	BEQ		i16ToStringConversionLoopDone

	MOV		R4, R0						; update number to convert

	B 		i16ToStringConversionLoop	; loop

i16ToStringConversionLoopDone:
	CMP		R6, #0						; check if negative
	BEQ		i16ToStringPositive			; if not, skip adding minus sign
	;B		i16ToStringNegative

i16ToStringNegative:
	MOV		R1, #ASCII_MINUS				; prepare minus sign ASCII code
	STRB	R1, [R7], #1				; add minus sign to buffer
	;B		i16ToStringPositive

i16ToStringPositive:
	SUB		R7, #1						; point to last byte in stack buffer
	;B		i16ToStringCopy

i16ToStringCopyLoop:
	LDRB	R1, [R7], #-1				; load byte from stack buffer
	STRB	R1, [R5], #1				; store byte in string buffer
	CMP		R7, R13						; check if below the top of stack
	BGE		i16ToStringCopyLoop
	;B		i16ToStringCopyDone

i16ToStringCopyDone:
	ADD		R13, #BUFFER_SIZE			; restore stack pointer
	POP		{LR, R4, R5, R6, R7}		; restore return address and used registers
	BX		LR							; return
