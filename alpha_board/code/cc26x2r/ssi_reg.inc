;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                 ssi_reg.inc                                ;
;                            SSI Register Constants                          ;
;                                 Include File                               ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains the constants for the the Serial Synchronous Interface (SSI)
; module. 
;
; Revision History:


; base address

SSI0_BASE_ADDR .equ			0x40000000	; SSI0 base address
SSI1_BASE_ADDR .equ			0x40008000	; SSI1 base address


; register offsets

CR0_OFFSET		.equ		0x000		; Control Register 0
CR1_OFFSET		.equ		0x004		; Control Register 1
DR_OFFSET		.equ		0x008		; Data Register
SR_OFFSET		.equ		0x00C		; Status Register
CPSR_OFFSET		.equ		0x010		; Clock Prescale Register
IMSC_OFFSET		.equ		0x014		; Interrupt Mask Set and Clear Register
RIS_OFFSET		.equ		0x018		; Raw Interrupt Status Register
MIS_OFFSET		.equ		0x01C		; Masked Interrupt Status Register
ICR_OFFSET		.equ		0x020		; Interrupt Clear Register
DMACR_OFFSET	.equ		0x024		; DMA Control Register


; register values

; CR0 register values
CR0_DSS_4_BIT		.equ		0x3			; 4-bit data
CR0_DSS_5_BIT		.equ		0x4			; 5-bit data
CR0_DSS_6_BIT		.equ		0x5			; 6-bit data
CR0_DSS_7_BIT		.equ		0x6			; 7-bit data
CR0_DSS_8_BIT		.equ		0x7			; 8-bit data
CR0_DSS_9_BIT		.equ		0x8			; 9-bit data
CR0_DSS_10_BIT		.equ		0x9			; 10-bit data
CR0_DSS_11_BIT		.equ		0xA			; 11-bit data
CR0_DSS_12_BIT		.equ		0xB			; 12-bit data
CR0_DSS_13_BIT		.equ		0xC			; 13-bit data
CR0_DSS_14_BIT		.equ		0xD			; 14-bit data
CR0_DSS_15_BIT		.equ		0xE			; 15-bit data
CR0_DSS_16_BIT		.equ		0xF			; 16-bit data

CR0_FRF_SPI			.equ		0x0 << 4	; SPI frame format
CR0_FRF_TI			.equ		0x1 << 4	; TI frame format
CR0_FRF_MICROWIRE	.equ		0x2 << 4	; Microwire frame format

CR0_SPO_LOW			.equ		0x0 << 6	; SPO = 0
CR0_SPO_HIGH		.equ		0x1 << 6	; SPO = 1

CR0_SPH_FIRST		.equ		0x0 << 7	; SPH = 0
CR0_SPH_SECOND		.equ		0x1 << 7	; SPH = 1

CR0_SCR_OFFSET		.equ		8			; SCR bit offset


; CR1 register values
CR1_LBM_NORMAL		.equ		0x0			; normal mode
CR1_LBM_LOOPBACK	.equ		0x1			; loopback mode

CR1_SSE_DISABLE		.equ		0x0 << 1	; SSI operation is disabled
CR1_SSE_ENABLE		.equ		0x1 << 1	; SSI operation is enabled

CR1_MS_MASTER		.equ		0x0 << 2	; master mode
CR1_MS_SLAVE		.equ		0x1 << 2	; slave mode

CR1_SOD_NORMAL		.equ		0x0 << 3	; normal mode
CR1_SOD_FORCE		.equ		0x1 << 3	; force output data

; DR register values
DR_DATA_MASK		.equ		0xFFFF		; data mask

; SR register values
SR_TFE_EMPTY		.equ		0x1			; transmit FIFO empty
SR_TNF_NOTFULL		.equ		0x1 << 1	; transmit FIFO not full
SR_RNE_NOTEMPTY		.equ		0x1 << 2	; receive FIFO not empty
SR_RFF_FULL			.equ		0x1 << 3	; receive FIFO full
SR_BSY_BUSY			.equ		0x1 << 4	; busy

; CPSR register values
CPSR_CPSDVSR_MASK	.equ		0xFF		; clock prescale divisor mask

