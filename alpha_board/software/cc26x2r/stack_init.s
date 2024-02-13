; StackInit
;
;
; Description: This function initializes the stack pointers for the
; processor. It sets the Main Stack Pointer (MSP) to the
; top of the stack and the Process Stack Pointer (PSP) to
; the bottom of the stack. It also sets the Vector Table
;
; Operation
;
; Arguments: None.
; Return Values: None.
;
; Local Variables:         None.
; Shared Variables:     None.
; Global Variables:     None.
;
; Input:                 None.
; Output:                 None.
;
; Error Handling:         None.
;
; Registers Changed:     R0
; Stack Depth:             0 word
;
; Algorithms: None.
; Data Structures: None.
;
; Revision History: 11/03/21 Glen George initial revision


; local include files
    .include "stack_symbols.inc"
    .include "../std.inc"

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; MEMORY
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    .data
    .align STACK_ALIGN

    .space TOTAL_STACK_SIZE

TopOfStack:

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; CODE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    .text
    .def StackInit
StackInit:
    MOVA    R0, TopOfStack
    MSR        MSP, R0                        ; set the handler stack to the top of the stack
    SUB        R0, R0, #HANDLER_STACK_SIZE 
    MSR        PSP, R0                        ; set the process stack to be the rest of the stack
