;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                   lcd_init.s                               ;
;                           14-pin LCD Initialization                        ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This file contains the initialization procedure for a 14-pin character LCD.
; To configure the LCD (pins, timer), look at `lcd_symbols.inc`.

; This file defines functions:
;   LCDInit
; 
; Revision History:
;     11/22/23  Adam Krivka      initial revision


; local includes
    .include "lcd_symbols.inc"
    .include "../std.inc"
    .include "../cc26x2r/gpio_reg.inc"
    .include "../cc26x2r/ioc_reg.inc"

; import functinos from other files
    .ref   LCDWaitForNotBusy
    .ref   LCDWrite
    .ref   LCDConfigureForWrite

; export functions to other files
    .def   LCDInit



    .text
; LCDInitTab
; 
; This table contains the initializition procedure of the LCD. Each step
; is 2-words and contains the command/data to be sent to the LCD, and the 
; delay to wait _before_ the command is sent. If the delay is -1, the busy
; is to be read.

    .align 4 ; align to word
LCDInitTab:
           ;Command         Delay Count
    .word   00111000b,     15000 * LOOPS_PER_US   ; wait for 150ms
    .word   00111000b,     4100 * LOOPS_PER_US    ; wait for 4.1ms
    .word   00111000b,     100 * LOOPS_PER_US     ; wait for 100us
    .word   00111000b,     -1      ; function set
    .word   00001000b,     -1      ; display off
    .word   00000001b,     -1      ; clear display
    .word   00000110b,     -1      ; entry mode set (increment cursor, no shift)
    .word   00001111b,     -1      ; display/cursor on

EndLCDInitTab:



; LCDInit
;
; Description:          Initializes the LCD by first initializing the GPT1 
;                       timers, and then sending the required set of commands
;                       to the LCD.  The commands are stored in a table in
;                       a table, and are sent to the LCD one at a time.
;
; Arguments:            r in R0, c in R1, str in R2
; Return Values:        None.
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Inputs:               None.
; Outputs:              None.
;
; Error Handling:       None.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          5
; 
; Revision History:
;     11/22/23  Adam Krivka      initial revision


LCDInit:
    PUSH        {LR, R4, R5, R6, R7}        ; save return address and used registers

    ; set up control pins
    MOV32       R1, IOC_BASE_ADDR   ; prepare IOC base address
    STREG       IOCFG_GENERIC_OUTPUT, R1, IOCFG_REG_SIZE * RS_PIN      ; RS pin
    STREG       IOCFG_GENERIC_OUTPUT, R1, IOCFG_REG_SIZE * RW_PIN      ; RW pin
    STREG       IOCFG_GENERIC_OUTPUT, R1, IOCFG_REG_SIZE * E_PIN       ; E pin

    ; enable output for control pins 
    MOV32       R1, GPIO_BASE_ADDR
    STREG       ((1 << RW_PIN) | (1 << RS_PIN) | (1 << E_PIN)), R1, GPIO_DOE_OFFSET

    ; keep default state of writing
    BL          LCDConfigureForWrite

    ; set up timer
    MOV32       R1, TIMER_BASE_ADDR
    STREG       TIMER_CFG, R1, GPT_CFG_OFFSET
    STREG       TIMER_TAMR, R1, GPT_TAMR_OFFSET
    STREG       TIMER_TAILR, R1, GPT_TAILR_OFFSET
    STREG       TIMER_TAMATCHR, R1, GPT_TAMATCHR_OFFSET
    STREG       TIMER_TAPR, R1, GPT_TAPR_OFFSET

    ; start it so that it's timed out for first read
    STREG       TIMER_ENABLE, R1, GPT_CTL_OFFSET

    ; start init loop
    ; get address of initialization table
    ADR     R4, LCDInitTab
    ADR        R5, EndLCDInitTab
LCDInitLoop:
    LDR     R6, [R4], #4    ; load command (DATA)
    LDR     R7, [R4], #4    ; load delay count

    ; check if delay count is -1
    CMP     R7, #-1
    BNE     LCDInitLoopWaitLoop ; if not use delay count
    ;B      LCDInitLoopWaitBusy ; if yes wait for busy flag to clear

LCDInitLoopWaitBusy:
    BL      LCDWaitForNotBusy   ; wait for busy flag to go be cleared
    B       LCDInitLoopWrite    ; carry to writing the next command

; wait using a loop
LCDInitLoopWaitLoop:
    SUBS    R7, #1              ; subtract from delay count
    BNE     LCDInitLoopWaitLoop ; if not zero, loop again
    ;B      LCDInitLoopWrite    ; if zero, carry on to writing

LCDInitLoopWrite:
    ; prepare arguments
    MOV32   R0, 0          ; RS = 0
    MOV     R1, R6         ; command/DATA
    BL      LCDWrite       ; write to LCD
        
    ; if address is equal to EndLCDInitTab, break init loop
    CMP     R4, R5
    BNE     LCDInitLoop

LCDInitEnd:
    POP     {LR, R4, R5, R6, R7}        ; restore return address and used registers
    BX      LR                          ; return
