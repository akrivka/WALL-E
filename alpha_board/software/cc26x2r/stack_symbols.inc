;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                              stack_symbols.inc                             ;
;                          Stack Initialization Symbols                      ;
;                                 Include File                               ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This file contains the stack initialization symbols for the application.
; The symbols are used by the startup code to initialize the stack.
;
; Revision History: 7/11/23 Adam Krivka     Initial revision


STACK_ALIGN .equ            8       ; 8-byte alignment

; Stack sizes (adjust these values for your application)
HANDLER_STACK_SIZE .equ        128     ; handler stack size 
PROCESS_STACK_SIZE .equ        256     ; process stack size

; total stack size
TOTAL_STACK_SIZE .equ        PROCESS_STACK_SIZE + HANDLER_STACK_SIZE
