;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                 gpt_reg.inc                                ;
;                            GPT Register Constants                          ;
;                                 Include File                               ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains the constants for the General Purpose Timer control 
; registers for the TI CC2652 microcontroller.
;
; Revision History:
;     	11/7/23  	Adam Krivka		initial revision
;		12/5/23		Adam Krivka		added IRQ numbers 


; base addresses

GPT0_BASE_ADDR .equ              0x40010000 ; general purpose timer 0
GPT1_BASE_ADDR .equ              0x40011000 ; general purpose timer 1
GPT2_BASE_ADDR .equ              0x40012000 ; general purpose timer 2
GPT3_BASE_ADDR .equ              0x40013000 ; general purpose timer 3


; register offsets

GPT_CFG_OFFSET .equ              0x0        ; configuration register
GPT_TAMR_OFFSET .equ             0x4        ; timer A mode register
GPT_TBMR_OFFSET .equ             0x8        ; timer B mode register
GPT_CTL_OFFSET  .equ             0xC        ; control register
GPT_SYNC_OFFSET .equ             0x10       ; synchronization register
GPT_IMR_OFFSET  .equ             0x18       ; interrupt mask register
GPT_RIS_OFFSET  .equ             0x1C       ; raw interrupt status register
GPT_MIS_OFFSET  .equ             0x20       ; masked interrupt status register
GPT_ICLR_OFFSET .equ             0x24       ; interrupt clear register
GPT_TAILR_OFFSET .equ            0x28       ; timer A interval load register
GPT_TBILR_OFFSET .equ            0x2C       ; timer B interval load register
GPT_TAMATCHR_OFFSET .equ         0x30       ; timer A match register
GPT_TBMATCHR_OFFSET .equ         0x34       ; timer B match register
GPT_TAPR_OFFSET .equ             0x38       ; timer A prescale register
GPT_TBPR_OFFSET .equ             0x3C       ; timer B prescale register
GPT_TAPMR_OFFSET .equ            0x40       ; timer A prescale match register
GPT_TBPMR_OFFSET .equ            0x44       ; timer B prescale match register
GPT_TAR_OFFSET .equ              0x48       ; timer A register
GPT_TBR_OFFSET .equ              0x4C       ; timer B register
GPT_TAV_OFFSET .equ              0x50       ; timer A value register
GPT_TBV_OFFSET .equ              0x54       ; timer B value register
GPT_TAPS_OFFSET .equ             0x5C       ; timer A prescale snapshot register
GPT_TBPS_OFFSET .equ             0x60       ; timer B prescale snapshot register
GPT_TAPV_OFFSET .equ             0x60       ; timer A prescale value register
GPT_TBPV_OFFSET .equ             0x64       ; timer B prescale value register
GPT_DMAEV_OFFSET .equ            0x68       ; DMA event register


; register value definitions 

; CFG - configuration register

GPT_CFG_32BIT .equ               0x0        ; 32-bit timer configuration
GPT_CFG_2_16BIT .equ             0x4        ; two 16-bit timers configuration


; TXMR - timer mode register

GPT_TXMR_ONESHOT .equ            0x1        ; one-shot timer mode
GPT_TXMR_PERIODIC .equ           0x2        ; periodic timer mode
GPT_TXMR_CAPTURE .equ            0x3        ; capture mode

GPT_TXMR_TXCM_EDGCNT .equ        0x0 << 2   ; edge count mode
GPT_TXMR_TXCM_EDGTIME .equ       0x1 << 2   ; edge time mode

GPT_TXMR_TXAMS_CAPTURE .equ      0x0 << 3   ; capture mode
GPT_TXMR_TXAMS_PWM .equ          0x1 << 3   ; PWM mode

GPT_TXMR_TXCDIR_DOWN .equ        0x0 << 4   ; count down
GPT_TXMR_TXCDIR_UP .equ          0x1 << 4   ; count up

GPT_TXMR_TXMIE_DISABLED .equ     0x0 << 5   ; time-out interrupt disabled
GPT_TXMR_TXMIE_ENABLED .equ      0x1 << 5   ; time-out interrupt enabled

GPT_TXMR_TXWOT_IMMEDIATE .equ    0x0 << 6   ; immediate trigger
GPT_TXMR_TXWOT_WAIT .equ         0x1 << 6   ; wait trigger

GPT_TXMR_TXSNAPS_DISABLED .equ   0x0 << 7   ; snapshot disabled
GPT_TXMR_TXSNAPS_ENABLED .equ    0x1 << 7   ; snapshot enabled

; PWM interval load write mode
GPT_TXMR_TXILD_NEXT_CLOCK .equ   0x0 << 8   ; next clock
GPT_TXMR_TXILD_NEXT_TIMEOUT .equ 0x1 << 8   ; next timeout

GPT_TXMR_TAPWMIE_DISABLED .equ   0x0 << 9   ; PWM interrupt disabled
GPT_TXMR_TAPWMIE_ENABLED .equ    0x1 << 9   ; PWM interrupt enabled

; match register update mode
GPT_TXMR_TXMRSU_NEXT_CYCLE .equ  0x0 << 10  ; next cycle
GPT_TXMR_TXMRSU_NEXT_TIMEOUT .equ 0x1 << 10 ; next timeout

; CCP pin output mode
GPT_TXMR_TXPLO_LEGACY .equ       0x0 << 11  ; legacy mode
GPT_TXMR_TXPLO_HIGH .equ         0x1 << 11  ; CCP pin high on time-out

; one-shot/periodic interrupt
GPT_TXMR_TXCINTD_NORMAL .equ     0x0 << 12  ; normal
GPT_TXMR_TXCINTD_DISABLED .equ   0x1 << 12  ; disabled

; timer compare action on time-out select
GPT_TXMR_TCACT_DISABLE .equ      0x0 << 13  ; disable
GPT_TXMR_TCACT_TOGGLE .equ       0x1 << 13  ; toggle
GPT_TXMR_TCACT_CLEAR .equ        0x2 << 13  ; clear CCP pin
GPT_TXMR_TCACT_SET .equ          0x3 << 13  ; set CCP pin
GPT_TXMR_TCACT_SET_TOGGLE .equ   0x4 << 13  ; set CCP pin and toggle
GPT_TXMR_TCACT_CLEAR_TOGGLE .equ 0x5 << 13  ; clear CCP pin and toggle
GPT_TXMR_TCACT_SET_CLEAR .equ    0x6 << 13  ; set CCP pin and clear
GPT_TXMR_TCACT_CLEAR_SET .equ    0x7 << 13  ; clear CCP pin and set


; CTL - control register

; timer A enable
GPT_CTL_TAEN_DISABLED .equ       0x0        ; timer A disabled
GPT_CTL_TAEN_ENABLED .equ        0x1        ; timer A enabled and starts

; behavior while halted by the processor
GPT_CTL_TASTALL_CONTINUE .equ    0x0 << 1   ; continue counting
GPT_CTL_TASTALL_FREEZE .equ      0x1 << 1   ; freeze counting

; timer A event mode
GPT_CTL_TAEVENT_POS_EDGE .equ    0x0 << 2   ; positive edge
GPT_CTL_TAEVENT_NEG_EDGE .equ    0x1 << 2   ; negative edge
GPT_CTL_TAEVENT_BOTH_EDGE .equ   0x3 << 2   ; both edges

; timer A PWM output level
GPT_CTL_TAPWML_NORMAL .equ       0x0 << 6   ; normal
GPT_CTL_TAPWML_INVERTED .equ     0x1 << 6   ; inverted

; timer B enable
GPT_CTL_TBEN_DISABLED .equ       0x0 << 8   ; timer B disabled
GPT_CTL_TBEN_ENABLED .equ        0x1 << 8   ; timer B enabled and starts

; behavior while halted by the processor
GPT_CTL_TBSTALL_CONTINUE .equ    0x0 << 9   ; continue counting
GPT_CTL_TBSTALL_FREEZE .equ      0x1 << 9   ; freeze counting

; timer B event mode
GPT_CTL_TBEVENT_POS_EDGE .equ    0x0 << 10  ; positive edge
GPT_CTL_TBEVENT_NEG_EDGE .equ    0x1 << 10  ; negative edge
GPT_CTL_TBEVENT_BOTH_EDGE .equ   0x3 << 10  ; both edges

; timer B PWM output level
GPT_CTL_TBPWML_NORMAL .equ       0x0 << 14  ; normal
GPT_CTL_TBPWML_INVERTED .equ     0x1 << 14  ; inverted


; IMR - interrupt mask register

; timer A
GPT_IMR_TATOIM_DISABLED .equ     0x0        ; time-out interrupt disabled
GPT_IMR_TATOIM_ENABLED .equ      0x1        ; time-out interrupt enabled

GPT_IMR_CAMIM_DISABLED .equ      0x0 << 1   ; capture mode interrupt disabled
GPT_IMR_CAMIM_ENABLED .equ       0x1 << 1   ; capture mode interrupt enabled

GPT_IMR_CAEIM_DISABLED .equ      0x0 << 2   ; capture mode event interrupt disabled
GPT_IMR_CAEIM_ENABLED .equ       0x1 << 2   ; capture mode event interrupt enabled

GPT_IMR_TAMIM_DISABLED .equ      0x0 << 4   ; match interrupt disabled
GPT_IMR_TAMIM_ENABLED .equ       0x1 << 4   ; match interrupt enabled

GPT_IMR_DMAAIM_DISABLED .equ     0x0 << 5   ; DMA interrupt disabled
GPT_IMR_DMAAIM_ENABLED .equ      0x1 << 5   ; DMA interrupt enabled

; timer B
GPT_IMR_TBTOIM_DISABLED .equ     0x0 << 8   ; time-out interrupt disabled
GPT_IMR_TBTOIM_ENABLED .equ      0x1 << 8   ; time-out interrupt enabled

GPT_IMR_CBMIM_DISABLED .equ      0x0 << 9   ; capture mode interrupt disabled
GPT_IMR_CBMIM_ENABLED .equ       0x1 << 9   ; capture mode interrupt enabled

GPT_IMR_CBEIM_DISABLED .equ      0x0 << 10  ; capture mode event interrupt disabled
GPT_IMR_CBEIM_ENABLED .equ       0x1 << 10  ; capture mode event interrupt enabled

GPT_IMR_TBMIM_DISABLED .equ      0x0 << 11  ; match interrupt disabled
GPT_IMR_TBMIM_ENABLED .equ       0x1 << 11  ; match interrupt enabled

GPT_IMR_DMABIM_DISABLED .equ     0x0 << 13  ; DMA interrupt disabled
GPT_IMR_DMABIM_ENABLED .equ      0x1 << 13  ; DMA interrupt enabled


; RIS - raw interrupt status register
; timer A
GPT_RIS_TATORIS .equ             0x1        ; time-out raw interrupt status
GPT_RIS_CAMRIS .equ              0x1 << 1   ; capture mode raw interrupt status
GPT_RIS_CAERIS .equ              0x1 << 2   ; capture mode event raw interrupt status
GPT_RIS_TAMRIS .equ              0x1 << 4   ; match raw interrupt status
GPT_RIS_DMAARIS .equ             0x1 << 5   ; DMA raw interrupt status

; timer B
GPT_RIS_TBTORIS .equ             0x1 << 8   ; time-out raw interrupt status
GPT_RIS_CBMRIS .equ              0x1 << 9   ; capture mode raw interrupt status
GPT_RIS_CBERIS .equ              0x1 << 10  ; capture mode event raw interrupt status
GPT_RIS_TBMRIS .equ              0x1 << 11  ; match raw interrupt status
GPT_RIS_DMABRIS .equ             0x1 << 13  ; DMA raw interrupt status


; ICLR - interrupt clear register

; timer A
GPT_ICLR_TATOCINT_CLEAR .equ     0x1        ; time-out interrupt clear
GPT_ICLR_CAMCINT_CLEAR .equ      0x1 << 1   ; capture mode interrupt clear
GPT_ICLR_CAECINT_CLEAR .equ      0x1 << 2   ; capture mode event interrupt clear
GPT_ICLR_TAMCINT_CLEAR .equ      0x1 << 4   ; match interrupt clear
GPT_ICLR_DMAAINT_CLEAR .equ      0x1 << 5   ; DMA interrupt clear

; timer B
GPT_ICLR_TBTOCINT_CLEAR .equ     0x1 << 8   ; time-out interrupt clear
GPT_ICLR_CBMCINT_CLEAR .equ      0x1 << 9   ; capture mode interrupt clear
GPT_ICLR_CBECINT_CLEAR .equ      0x1 << 10  ; capture mode event interrupt clear
GPT_ICLR_TBMCINT_CLEAR .equ      0x1 << 11  ; match interrupt clear
GPT_ICLR_DMABINT_CLEAR .equ      0x1 << 13  ; DMA interrupt clear

; exception numbers
GPT0A_IRQ_NUMBER .equ            15         ; GPT0A interrupt number
GPT0A_EXCEPTION_NUMBER .equ      31         ; GPT0A exception number

GPT0B_IRQ_NUMBER .equ            16         ; GPT0B interrupt number
GPT0B_EXCEPTION_NUMBER .equ      32         ; GPT0B exception number

GPT1A_IRQ_NUMBER .equ            17         ; GPT1A interrupt number
GPT1A_EXCEPTION_NUMBER .equ      33         ; GPT1A exception number

GPT1B_IRQ_NUMBER .equ            18         ; GPT1B interrupt number
GPT1B_EXCEPTION_NUMBER .equ      34         ; GPT1B exception number

GPT2A_IRQ_NUMBER .equ            19         ; GPT2A interrupt number
GPT2A_EXCEPTION_NUMBER .equ      35         ; GPT2A exception number

GPT2B_IRQ_NUMBER .equ            20         ; GPT2B interrupt number
GPT2B_EXCEPTION_NUMBER .equ      36         ; GPT2B exception number

GPT3A_IRQ_NUMBER .equ            21         ; GPT3A interrupt number
GPT3A_EXCEPTION_NUMBER .equ      37         ; GPT3A exception number

GPT3B_IRQ_NUMBER .equ            22         ; GPT3B interrupt number
GPT3B_EXCEPTION_NUMBER .equ      38         ; GPT3B exception number