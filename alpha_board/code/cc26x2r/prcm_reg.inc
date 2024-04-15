;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                 prcm_reg.inc                               ;
;                            PRCM Register Constants                         ;
;                                 Include File                               ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains the constants for the Power, Register, and Clock 
; Managment domain of the TI CC2652 microcontroller.
;
; Revision History:
;     11/7/23  Adam Krivka      initial revision


; base address

PRCM_BASE_ADDR .equ         0x40082000  ; PRCM base address


; register offsets

PDCTL0_OFFSET .equ          0x12c       ; Power Domain Control 0
PDSTAT0_OFFSET .equ         0x140       ; Power Domain Status 0
GPIOCLKGR_OFFSET .equ       0x48        ; GPIO Clock Gating Register
CLKLOADCTL_OFFSET .equ      0x28        ; Clock Load Control Register
GPTCLKGR_OFFSET .equ        0x54        ; GPT Clock Gating Register
GPTCLKGS_OFFSET .equ        0x58        ; GPT Clock Gating Register in sleep mode
GPTCLKGDS_OFFSET .equ       0x5C        ; GPT Clock Gating Register in deep sleep mode
SSICLKGR_OFFSET .equ        0x78        ; SSI Clock gating Register

; register values

; power domain
PDCTL0_RFC_ON .equ          1b          ; Enable RFC Power Domain
PDCTL0_SERIAL_ON .equ       10b         ; Enable Serial Power Domain
PDCTL0_PERIPH_ON .equ       100b        ; Enable Peripheral Power Domain

PDSTAT0_RFC_ON .equ         1b          ; RFC Power Domain On
PDSTAT0_SERIAL_ON .equ      10b         ; Serial Power Domain On
PDSTAT0_PERIPH_ON .equ      100b        ; Peripheral Power Domain On

; clock load
CLKLOADCTL_LOAD .equ        1b          ; Load Clock Control Registers
CLKLOADCTL_LOAD_DONE .equ   10b         ; Clock Control Registers Loaded

; GPIO clock
GPIOCLKGR_CLK_EN .equ       1b          ; Enable GPIO Clock

; GPT clock
GPTCLKGR_GPT0_ENABLE .equ  0x1          ; Enable GPT0 Clock
GPTCLKGR_GPT1_ENABLE .equ  0x2          ; Enable GPT1 Clock
GPTCLKGR_GPT2_ENABLE .equ  0x4          ; Enable GPT2 Clock
GPTCLKGR_GPT3_ENABLE .equ  0x8          ; Enable GPT3 Clock

GPTCLKGR_GPT_ENABLE_ALL .equ GPTCLKGR_GPT0_ENABLE | GPTCLKGR_GPT1_ENABLE | GPTCLKGR_GPT2_ENABLE | GPTCLKGR_GPT3_ENABLE
                                        ; Enable all clocks

; SSI clock
SSICLKGR_ENABLE_SSI0 .equ       0x1          ; Enable SSI0 Clock
SSICLKGR_ENABLE_SSI1 .equ       0x2          ; Enable SSI1 Clock

SSICLKGR_ENABLE_SSI0_FORCE .equ 0x1 << 8     ; Enable SSI0 Clock Override
SSICLKGR_ENABLE_SSI1_FORCE .equ 0x2 << 8     ; Enable SSI1 Clock Override
