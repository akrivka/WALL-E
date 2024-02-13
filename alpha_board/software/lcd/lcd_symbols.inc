;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                               lcd_symbols.inc                              ;
;                                  LCD Symbols                               ;
;                                 Include File                               ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains symbols to configure a 4-line character LCD display.
;
; Revision History:
;     11/22/23  Adam Krivka      initial revision



; local includes 
    .include "../cc26x2r/gpt_reg.inc"
    .include "../imu_demo_symbols.inc"



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; DIMENSIONS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

NUM_ROWS .equ           4       ; number of lines/rows
NUM_COLS .equ           16      ; number of columns

ROW_0_START .equ        0x0     ; row 0 start address
ROW_1_START .equ        0x40    ; row 1 start address
ROW_2_START .equ        0x10    ; row 2 start address
ROW_3_START .equ        0x50    ; row 3 start address



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; PINS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; data bus
DATA_0_PIN .equ         LCD_DATA_0_PIN
DATA_1_PIN .equ         LCD_DATA_1_PIN
DATA_2_PIN .equ         LCD_DATA_2_PIN
DATA_3_PIN .equ         LCD_DATA_3_PIN
DATA_4_PIN .equ         LCD_DATA_4_PIN
DATA_5_PIN .equ         LCD_DATA_5_PIN
DATA_6_PIN .equ         LCD_DATA_6_PIN
DATA_7_PIN .equ         LCD_DATA_7_PIN

; control pins
RS_PIN .equ             LCD_RS_PIN   ; register select
RW_PIN .equ             LCD_RW_PIN   ; read/write
E_PIN .equ              LCD_E_PIN   ; enable



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; INSTRUCTIONS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

BUSY_FLAG_MASK .equ 0x1 << 7    ; the busy flag is at the 7th bit
DATA_MASK      .equ 0xFF        ; 8-bit mask for the data bus

SET_DDRAM_ADDR .equ 0x1 << 7    ; set DDRAM address instruction
CLEAR_DISPLAY .equ  0x1         ; clear display


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; TIMER
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; timer base address (defined elsewhere for flexibility)
TIMER_BASE_ADDR .equ    LCD_TIMER_BASE_ADDR

; configuration (we only use timer A)
TIMER_CFG .equ          GPT_CFG_2_16BIT

; timer A enable
TIMER_ENABLE .equ       GPT_CTL_TAEN_ENABLED

; one-shot, count up, match interrupt enabled
TIMER_TAMR .equ         GPT_TXMR_ONESHOT | GPT_TXMR_TXCDIR_UP | GPT_TXMR_TXMIE_ENABLED

; 1 us time-out 
; 48 Mhz / 48 = 1 Mhz (1 us)
TIMER_TAILR .equ        48
TIMER_TAPR .equ         0

; match at 450 ns
; 450 ns / (1 / 48 Mhz) = 21.6 ~ 22
TIMER_TAMATCHR .equ     22

; number of 2-instruction loops per microsecond
; (48 Mhz / 2) = 24 Mhz ~ 24 loops per microsecond
LOOPS_PER_US .equ       24

; number of loops to wait before bringing enable high in read/write operation
; should be at least 140ns (round_up(140 / 40ns) = 3.5 ~ 4)
LOOPS_IN_SETUP .equ      4
