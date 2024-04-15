;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                  lcd_test.s                                ;
;                           14-pin LCD I/O Test Code                         ; 
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This contain functions that test LCD functionality. Specifically it tests
; functions Display and DisplayChar defined in lcd_display:
;   TestDisplay
;   TestDisplayChar
; 
; Revision History: 
;     11/22/23  Adam Krivka      initial revision



; local includes
; none

; import functions from other files
	.ref Display
	.ref DisplayChar
	.ref ClearDisplay

; export symbols to other files
    .def TestDisplay
    .def TestDisplayChar



    .text

; Strings used in test cases
row_chars: .cstring "0123456789ABCDEF"
hello_world:   .cstring "Hello world"
adam_krivka:   .cstring "Adam Krivka"

; TestDisplayTab
;
; Table which contains test cases for the function Display. Each
; test case is 2 words long and has the following format:
;       row (16-bit), column (16-bit), string address (32-bit)

    .align 4    ; align to word
TestDisplayTab:
    ;     row,  col
    ;     string address
    .half 0,    0       ; Hello world in top left
    .word hello_world

    .half 3,    5       ; Name in bottom right
    .word adam_krivka

    .half 0,    0       ; Full first row, should return success
    .word row_chars

    .half 1,    0       ; Full second row, should return success
    .word row_chars

    .half 2,    0       ; Full third row, should return success
    .word row_chars

    .half 3,    0       ; Full fourth row, should return success
    .word row_chars

    .half 0,    4       ; First row going 4 chars over, should return fail
    .word row_chars

    .half 1,    4       ; Second row going 4 chars over, should return fail
    .word row_chars

    .half 2,    4   
	    ; Third row going 4 chars over, should return fail
    .word row_chars

    .half 3,    4       ; Fourth row going 4 chars over, should return fail
    .word row_chars

    .half 42,   255    ; Invalid row and column, should return fail
    .word hello_world

EndTestDisplayTab:


; TestDisplayTab
;
; Table which contains test cases for the function DisplayChar. 
; Each test case is 1 word long and has the following format:
;       row (8-bit), column (8-bit), character (8-bit), padding (8-bit)

    .align 4        ; align to word
TestDisplayCharTab:
    ;     row, col, character, padding
    .byte 0, 0, "a", 0      ; Print a, b, c spaced out on each row......
    .byte 0, 8, "b", 0      ; All calls should return success.
    .byte 0, 15,"c", 0 

    .byte 1, 0, "a", 0 
    .byte 1, 8, "b", 0
    .byte 1, 15,"c", 0 

    .byte 2, 0, "a", 0 
    .byte 2, 8, "b", 0
    .byte 2, 15,"c", 0 

    .byte 3, 0, "a", 0 
    .byte 3, 8, "b", 0
    .byte 3, 15,"c", 0 

    .byte 42, 4200, "0", 0  ; Invalid row and column, should return fail

EndTestDisplayCharTab:



; TestDisplay
;
; Description:          Tests the Display function by going over the test cases
;                       TestDisplayTab. You should put a breakpoint at
;                       "PUT BREAKPOINT HERE" and step through each test
;                       case, making sure the result is as expected.
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
; Stack Depth:          3
; 
; Revision History:
;     11/22/23  Adam Krivka      initial revision

TestDisplay:
    PUSH        {LR, R4, R5}            ; store return address and used registers
    ; main loop
    ADR         R4, TestDisplayTab      ; load address of test table
    ADR         R5, EndTestDisplayTab   ; load address of end of test table
TestDisplayLoop:
    BL          ClearDisplay            ; clear display

    ; load test case
    LDRH        R0, [R4], #2            ; load row
    LDRH        R1, [R4], #2            ; load column
    LDR         R2, [R4], #4            ; load string address

    BL          Display                 ; display string

    ; PUT BREAKPOINT HERE

    CMP         R4, R5                  ; compare current address to end address
    BNE         TestDisplayLoop         ; if not at end, loop
    ;B          TestDisplayEnd

TestDisplayEnd:
    POP         {LR, R4, R5}            ; restore return address and used registers
    BX          LR                      ; return



; TestDisplayChar
;
; Description:          Tests the Display function by going over the test cases
;                       TestDisplayCharTab. You should put a breakpoint at
;                       "PUT BREAKPOINT HERE" and step through each test
;                       case, making sure the result is as expected.
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
; Stack Depth:          3
; 
; Revision History:
;     11/22/23  Adam Krivka      initial revision

TestDisplayChar:
    PUSH        {LR, R4, R5}            ; store return address and used registers

    BL          ClearDisplay            ; clear display

    ; main loop
    ADR         R4, TestDisplayCharTab          ; load address of test table
    ADR         R5, EndTestDisplayCharTab       ; load address of end of test table
TestDisplayCharLoop:
    ; load test case
    LDRB        R0, [R4], #1            ; load row
    LDRB        R1, [R4], #1            ; load column
    LDRB        R2, [R4], #2            ; load character

    BL          DisplayChar             ; display character

    ; PUT BREAKPOINT HERE

    CMP         R4, R5                  ; compare current address to end address
    BNE         TestDisplayCharLoop             ; if not at end, loop
    ;B          TestDisplayCharEnd

TestDisplayCharEnd:
    POP         {LR, R4, R5}            ; restore return address and used registers
    BX          LR                      ; return
