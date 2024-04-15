;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                 lcd_display.s                              ;
;                             LCD Display function                           ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This file contains high-level display functionality of a 14-pin character LCD.
; 
; This file defines functions:
;   Display - display a string
;   DisplayChar - display a single character
;   ClearDisplay
; 
; Revision History:
;     11/22/23  Adam Krivka      initial revision



; local includes
    .include "../std.inc"
    .include "lcd_symbols.inc"
    .include "../lib/ascii.inc"

; import functions from other files
    .ref LCDWrite
    .ref LCDWaitForNotBusy

; export functions to other files
    .def Display
    .def DisplayChar
    .def ClearDisplay


; CursorBaseAddr
;
; Table containing the addresses of the first character in each row.
CursorBaseAddr:
    .byte ROW_0_START, ROW_1_START, ROW_2_START, ROW_3_START

; SetCursorPos
;
; Description:          Sets cursor position on the LCD. 
;
; Arguments:            r in R0, c in R1
; Return Values:        success/fail in R0.
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       If incorrect row and column values are passed, 
;                       returns FUNCTION_FAIL
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          0
; 
; Revision History:
;     11/22/23  Adam Krivka      initial revision

SetCursorPos:
    PUSH    {LR, R4, R5}                ; save return address and used registers

; check that row and column are in bounds
    CMP     R0, #0              ; check if r >= 0
    BLT     SetCursorPosFail    ; if r < 0, fail

    CMP     R0, #NUM_ROWS       ; check if r < NUM_ROWS
    BGE     SetCursorPosFail    ; if r >= NUM_ROWS, fail

    CMP     R1, #0              ; check if c >= 0
    BLT     SetCursorPosFail    ; if c < 0, fail

    CMP     R1, #NUM_COLS       ; check if c < NUM_COLS
    BGE     SetCursorPosFail    ; if c >= NUM_COLS
    ;B      SetCursorPosValid

SetCursorPosValid:
    MOVA    R4, CursorBaseAddr   ; load address of cursor base addresses
    LDRB    R5, [R4, R0]         ; load CursorBaseAddr[r] to R5
    ADD     R5, R1               ; add c to address
    ;B      SetCursorPosWrite

SetCursorPosWrite:
    BL      LCDWaitForNotBusy   ; wait for LCD to be ready

    ORR     R5, #SET_DDRAM_ADDR ; prepare a set DDRAM command
    MOV     R1, R5
    MOV32   R0, 0               ; RS = 0
    BL      LCDWrite            ; write set DDRAM command to LCD

    B       SetCursorPosSuccess

SetCursorPosFail:
    MOV32   R0, FUNCTION_FAIL   ; prepare fail return value
    B       SetCursorPosDone

SetCursorPosSuccess:
    MOV32   R0, FUNCTION_SUCCESS; prepare success return value
    ;B      SetCursorPosDone

SetCursorPosDone:
    POP     {LR, R4, R5}        ; restore return address and used registers
    BX      LR                  ; return


; Display
;
; Description:          Displays a string starting at the given row and column.
;                       Uses whatever cursor incrementing mode is currently
;                       set on the LCD. 
;
; Arguments:            R0: row,
;                       R1: column
;                       R2: str pointer
;                       R3: target length (0 for default length of string)
;
; Return Values:        success/fail in R0.
;
; Local Variables:      r, c, character in R3
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       If if the string ever goes out of screen,
;                       the function returns a fail value
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          4
; 
; Revision History:
;     11/22/23  Adam Krivka      initial revision
;     1/16/24   Adam Krivka      added target-length functionality


Display:
    PUSH    {LR, R4, R5, R6, R7}        ; save return address and used registers

    MOV     R4, R1                  ; save column
    MOV     R5, R2                  ; save string pointer
    MOV     R7, R3                  ; save target length

    BL      SetCursorPos            ; set cursor position based on r, c

    CMP     R0, #FUNCTION_FAIL      ; check if setting cursor failed
    BEQ     DisplayFail             ; fail this function too if so

    SUB		R4, #1					; subtract 1 from column so that it is
                                    ; correct on the first iteration of the loop
DisplayLoop:
    LDRB    R6, [R5], #1            ; load character from str (post-incr address)
    ADD     R4, #1                  ; add 1 to column

    CMP     R6, #0                  ; check if character is null terminator
    BEQ     DisplayPadRest          ; if yes, we're done printing the string

    CMP     R4, #NUM_COLS           ; check if we're off screen
    BGE     DisplayFail             ; if yes, stop and return fail value

    BL      LCDWaitForNotBusy       ; wait for LCD to be ready

    ; prepare arguments for writing to LCD
    MOV32   R0, 1                   ; RS = 1
    MOV     R1, R6                  ; copy character
    BL      LCDWrite                ; write data to LCD

    SUB     R7, #1                  ; decrement target length
    CMP     R7, #0                  ; check if we've reached target length
    BEQ     DisplaySuccess          ; if yes, we're done printing the string

    B       DisplayLoop             ; else, loop

DisplayFail:
    MOV32   R0, FUNCTION_FAIL       ; prepare fail return value
    B       DisplayDone

DisplayPadRest:
    BL      LCDWaitForNotBusy       ; wait for LCD to be ready

    ; prepare arguments for writing to LCD
    MOV32   R0, 1                   ; RS = 1
    MOV     R1, #ASCII_SPACE         ; copy space character
    BL      LCDWrite                ; write data to LCD

    SUB     R7, #1                  ; decrement target length
    CMP     R7, #0                  ; check if we've reached target length
    BEQ     DisplaySuccess          ; if yes, we're done printing the string

    B       DisplayPadRest          ; else, loop

DisplaySuccess:
    MOV32   R0, FUNCTION_SUCCESS    ; prepare success return value
    ;B      DisplayDone

DisplayDone:
    POP     {LR, R4, R5, R6, R7}        ; restore return address and used registers
    BX      LR                      ; return



; DisplayChar
;
; Description:          Displays a single character at the given row and column.
;
; Arguments:            r, c, ch
; Return Values:        None.
;
; Local Variables:      None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Error Handling:       If the row and column are out of bounds, the function
;                       returns a fail value.
;
; Registers Changed:    flags, R0, R1, R2, R3
; Stack Depth:          3
; 
; Revision History:
;     

DisplayChar:
    PUSH    {LR, R4, R5}        ; save return address and used registers

    MOV     R4, R1              ; save column
    MOV     R5, R2              ; save character

    BL      SetCursorPos        ; set cursor position based on r, c

    CMP     R0, #FUNCTION_FAIL  ; check if setting cursor failed
    BEQ     DisplayCharFail     ; fail this function too if so

    BL      LCDWaitForNotBusy   ; wait for LCD to not be busy

    ; write character to LCD
    MOV32   R0, 1               ; RS = 1
    MOV     R1, R5              ; copy character
    BL      LCDWrite            ; write data to LCD

    B       DisplayCharSuccess  ; we've successfully written the character

DisplayCharFail:
    MOV32   R0, FUNCTION_FAIL   ; prepare fail return value
    B       DisplayCharDone

DisplayCharSuccess:
    MOV32   R0, FUNCTION_SUCCESS; prepare success return value
    ;B      DisplayCharDone

DisplayCharDone:
    POP     {LR, R4, R5}        ; restore return address and used registers
    BX      LR                  ; return



; ClearDisplay
;
; Description:          Clears the LCD display and sets the cursor to top left.
;
; Arguments:            None
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
;     11/22/23  Adam Krivka      initial revision

ClearDisplay:
    PUSH    {LR}                ; store return address

    BL      LCDWaitForNotBusy   ; wait for LCD to not be busy

    MOV32   R0, 0               ; RS = 0
    MOV32   R1, CLEAR_DISPLAY   ; command = CLEAR_DISPLAY
    BL      LCDWrite            ; call LCDWrite(CLEAR_DISPLAY, RS = 0)

    POP     {LR}                ; restore return address
    BX      LR                  ; return
