;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                 aux_reg.inc                                ;
;                                AUX Constants                               ;
;                                 Include File                               ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains the constants for the AUX registers for the TI
; CC2652 microcontroller.
;
; Revision History:
;		12/5/23	Adam Krivka		initial revision



; base addresses
AUX_SPIM_BASE_ADDR .equ				0x400C1000	; AUX Domain SPI Master
AUX_MAC_BASE_ADDR .equ				0x400C2000	; Sensor Controller Math Accelerator
AUX_TIMER2_BASE_ADDR .equ			0x400C3000	; AUX Domain Timer 2
AUX_TDC_BASE_ADDR .equ				0x400C4000	; AUX Time-to-Digital Converter
AUX_EVCTL_BASE_ADDR .equ			0x400C5000	; AUX Domain Event Controller
AUX_SYSIF_BASE_ADDR .equ			0x400C6000	; AUX Domain System Interface
AUX_TIMER1_BASE_ADDR .equ			0x400C7000	; AUX Domain Timer 1
AUX_SMPH_BASE_ADDR .equ				0x400C8000	; AUX Domain Semaphore
AUX_ANAIF_BASE_ADDR .equ			0x400C9000	; AUX Domain Analog Interface
AUX_DDI0_OSC_BASE_ADDR .equ			0x400CA000	; AUX Domain Clock Control
AUX_ADI4_BASE_ADDR .equ				0x400CB000	; AUX Analog Digital Interface Master
AUX_AIODIO0_BASE_ADDR .equ			0x400CC000	; AUX Domain Analog Digital I/O 0
AUX_AIODIO1_BASE_ADDR .equ			0x400CD000	; AUX Domain Analog Digital I/O 1
AUX_AIODIO2_BASE_ADDR .equ			0x400CE000	; AUX Domain Analog Digital I/O 2
AUX_AIODIO3_BASE_ADDR .equ			0x400CF000	; AUX Domain Analog Digital I/O 3
AUX_RAM_BASE_ADDR .equ				0x400E0000	; Sensor Controller AUX RAM
AUX_SCE_BASE_ADDR .equ				0x400E1000	; Sensor Controller Engine



; AUX_ADI4
; register offsets
AUX_ADI4_MUX0_OFFSET .equ			0x0			; MUX0
AUX_ADI4_MUX1_OFFSET .equ			0x1			; MUX1
AUX_ADI4_MUX2_OFFSET .equ			0x2			; MUX2
AUX_ADI4_MUX3_OFFSET .equ			0x3			; MUX3
AUX_ADI4_ISRC_OFFSET .equ			0x4			; ISRC
AUX_ADI4_COMP_OFFSET .equ			0x5			; COMP
AUX_ADI4_MUX4_OFFSET .equ			0x7			; MUX4
AUX_ADI4_ADC0_OFFSET .equ			0x8			; ADC0
AUX_ADI4_ADC1_OFFSET .equ			0x9			; ADC1
AUX_ADI4_ADCREF0_OFFSET .equ		0xA			; ADCREF0
AUX_ADI4_ADCREF1_OFFSET .equ		0xB			; ADCREF1

; register values
AUX_ADI4_ADC0_EN .equ				0x1			; ADC Enable
AUX_ADI4_ADC0_RESET .equ			0x0 << 1	; Reset ADC
AUX_ADI4_ADC0_NORMAL .equ			0x1 << 1	; Reset ADC
AUX_ADI4_ADC0_2700ns .equ			0x3 << 3	; 2.7us sampling duration
AUX_ADI4_ADC0_SYNC .equ				0x0 << 7	; Synchronous mode
AUX_ADI4_ADC0_ASYNC .equ			0x1 << 7	; Asynchronous mode

AUX_ADI4_ADCREF0_ENABLE .equ		0x1			; enable ADC reference module
AUX_ADI4_ADCREF0_REF_ON_IDLE .equ	0x1 << 6	; enable ADC in idle state



; AUX_AIODIO
; register offsets
AUX_AIODIO_IOMODE_OFFSET .equ		0x0			; Input Output Mode

; register values
AUX_AIODIO_IOMODE_IOSIZE .equ		0x2			; size of range for configuring each AUXIO
AUX_AIODIO_IOMODE_OUTPUT .equ		0x0			; Output Mode
AUX_AIODIO_IOMODE_INPUT .equ		0x1			; Input Mode
AUX_AIODIO_IOMODE_OPENDRAIN .equ	0x2			; Open-Drain Mode
AUX_AIODIO_IOMODE_OPENSOURCE .equ	0x0			; Open-Source Mode



; AUX_ANAIF
; register offsets
AUX_ANAIF_ADCCTL_OFFSET .equ		0x10		; ADC Control
AUX_ANAIF_ADCFIFOSTAT_OFFSET .equ	0x14		; ADC FIFO Status
AUX_ANAIF_ADCFIFO_OFFSET .equ		0x18		; ADC FIFO
AUX_ANAIF_ADCTRIG_OFFSET .equ		0x1C		; ADC Trigger

; register values
AUX_ANAIF_ADCCTL_DISABLE .equ		0x0			; Disable ADC Interface
AUX_ANAIF_ADCCTL_ENABLE .equ		0x1			; Enable ADC Interface
AUX_ANAIF_ADCCTL_FLUSH .equ			0x3			; Flush ADC FIFO
AUX_ANAIF_ADCCTL_TRIG_NO_EVENT .equ	0x3F << 8	; No trigger

AUX_ANAIF_ADCFIFOSTAT_EMPTY .equ	0x1			; FIFO Empty

AUX_ANAIF_ADCFIFO_MASK .equ			0xFFF		; FIFO Data

AUX_ANAIF_ADCTRIG_TRIG .equ			0x1			; Single ADC trigger



; AUX_SYSIF
; register offsets
AUX_SYSIF_ADCCLKCTL_OFFSET .equ		0x50		; ADC Clock Control

; register values
AUX_SYSIF_ADCCLKCTL_ENABLE .equ		0x1			; Enable ADC Clock
AUX_SYSIF_ADCCLKCTL_ACK_ENABLE .equ	0x1 << 1	; ADC clock is enabled

