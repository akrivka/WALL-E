;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                               event_reg.inc                                ;
;                          EVENT Register Constants                          ;
;                                Include File                                ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains the constants for the EVENT control registers for
; the TI CC2652 microcontroller.
;
; Revision History:
;		12/5/23	Adam Krivka		initial revision


; base address
EVENT_BASE_ADDR .equ			0x40083000

; register offsets
EVENT_GPT0ACAPTSEL_OFFSET .equ	0x200		; Output Selection for GPT0 0 A
EVENT_GPT0BCAPTSEL_OFFSET .equ	0x204		; Output Selection for GPT0 0 B
EVENT_GPT1ACAPTSEL_OFFSET .equ	0x300		; Output Selection for GPT0 1 A
EVENT_GPT1BCAPTSEL_OFFSET .equ	0x304		; Output Selection for GPT0 1 B
EVENT_GPT2ACAPTSEL_OFFSET .equ	0x400		; Output Selection for GPT0 2 A
EVENT_GPT2BCAPTSEL_OFFSET .equ	0x404		; Output Selection for GPT0 2 B
EVENT_GPT3ACAPTSEL_OFFSET .equ	0x600		; Output Selection for GPT0 3 A
EVENT_GPT3BCAPTSEL_OFFSET .equ	0x604		; Output Selection for GPT0 3 B

; register values
EVENT_GPTXCAPTSEL_PORT .equ		0x59		; Port capture event from IOC, PORT_EVENT4