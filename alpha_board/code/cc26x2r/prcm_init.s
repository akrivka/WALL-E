;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                            ;
;                                prcm_init.s                                 ;
;                        PRCM Initialization Procedures                      ;
;                                                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This file contains the initializion procedures for the Power Domain and 
; clocks of the TI CC2652 microcontroller:
;    PeriphPowerInit - initializes the Power Domain for the peripherals
;    GPIOClockInit - initializes the clock for the GPIO peripheral
;    GPTClockInit - initializes the clock for the GPT peripheral
;	 SSIClockInit - initializes the clock for the SSI module
;
; Revision History:
;     11/7/23  Adam Krivka      initial revision
;	  1/12/24  Adam Krivka		added SSIClockInit


; local include files
    .include "prcm_reg.inc"
    .include "../std.inc"

; exporting functions defined in this file
    .def PeriphPowerInit
    .def GPIOClockInit
    .def GPTClockInit
    .def SSIClockInit

; ----------------------------------------------------------------------------

; PeriphPowerInit
; 
; Description:        Initializes the Power Domain for the peripherals.
;
; Operation:        Sets PDCTL0 to turn on peripheral power, then waits for
;                    PDSTAT0 to indicate that the power is on.
;
; Arguments:        None
; Returns:            None
;
; Local Variables:    None
; Global Variables:    None
; 
; Error Handling:    None
;
; Algorithms:        None
; Data Structures:    None
;
; Registers Changed:    R0, R1
; Stack Depth:        0
;
; Revision History:    11/7/23  Adam Krivka  initial revision

PeriphPowerInit:
    MOV32     R0, PRCM_BASE_ADDR            ; prepare PRCM register base address
    MOV        R1, #PDCTL0_PERIPH_ON        ; prepare PDCTL0 value for turning
                                        ; on peripheral power
    STR     R1, [R0, #PDCTL0_OFFSET]    ; write PDCTL0 to turn on peripheral power
    ;B        PeriphPowerLoop

; Wait for PDSTAT0 to signal peripheral power on
PeriphPowerLoop:
    ; check if PDSTAT0 has PERIPH_ON bit set active
    LDR     R1, [R0, #PDSTAT0_OFFSET]
    CMP        R1, #PDSTAT0_PERIPH_ON

    BNE        PeriphPowerLoop        ; if not, loop/try again
    BX        LR                    ; if yes, return



; GPIOClockInit
;
; Description:        Initializes the clock for the GPIO peripheral.
;
; Operation:        Sets GPIOCLKGR to turn on the clock, then waits for
;                    CLKLOADCTL to indicate that the clock is on.
;
; Arguments:        None
; Returns:            None
;
; Local Variables:    None
; Global Variables:    None
;
; Error Handling:    None
;
; Algorithms:        None
; Data Structures:    None
;
; Registers Changed:    R0, R1
; Stack Depth:        0
;
; Revision History:    11/7/23  Adam Krivka  initial revision

GPIOClockInit:
    MOV32     R0, PRCM_BASE_ADDR            ; prepare PRCM register base address

    MOV        R1, #GPIOCLKGR_CLK_EN        ; prepare CLK_EN (clock enable) value
                                        ; for GPIOCLKGR
    STR        R1, [R0, #GPIOCLKGR_OFFSET]    ; write to GPIOCLKGR register to turn
                                        ; on the clock

    MOV        R1, #CLKLOADCTL_LOAD        ; prepare LOAD value for CLKLOADCTL
    STR        R1, [R0, #CLKLOADCTL_OFFSET]; write to CLKLOADCTL to load the clock
    ;GPIOClockLoop

; Wait for CLKLOADCTL to signal clock done loading
GPIOClockLoop:
    ; check if CLKLOADCTL has LOAD_DONE bit set active
    LDR        R1, [R0, #CLKLOADCTL_OFFSET]
    CMP        R1, #CLKLOADCTL_LOAD_DONE    

    BNE        GPIOClockLoop            ; if not, loop/try again
    BX        LR                        ; if yes, return



; GPTClockInit
;
; Description:        Initializes the clock for the GPT peripheral.
;
; Operation:        Sets GPTCLKGR to turn on the clock, then waits for
;                    CLKLOADCTL to indicate that the clock is on.
;
; Arguments:        None
; Returns:            None
;
; Local Variables:    None
; Global Variables:    None
;
; Error Handling:    None
;
; Algorithms:        None
; Data Structures:    None
;
; Registers Changed:    R0, R1
; Stack Depth:        0
;
; Revision History:    11/7/23  Adam Krivka  initial revision

GPTClockInit:
    MOV32     R0, PRCM_BASE_ADDR            ; prepare PRCM register base address
    MOV        R1, #GPTCLKGR_GPT_ENABLE_ALL ; prepare CLK_EN (clock enable) value
                                            ; for GPTCLKGR
    STR        R1, [R0, #GPTCLKGR_OFFSET]   ; write to GPTCLKGR register to turn
                                            ; on the clock

    MOV        R1, #CLKLOADCTL_LOAD         ; prepare LOAD value for CLKLOADCTL
    STR        R1, [R0, #CLKLOADCTL_OFFSET] ; write to CLKLOADCTL to load the clock
    ;GPTClockLoop

; Wait for CLKLOADCTL to signal clock done loading
GPTClockLoop:
    ; check if CLKLOADCTL has LOAD_DONE bit set active
    LDR        R1, [R0, #CLKLOADCTL_OFFSET]
    CMP        R1, #CLKLOADCTL_LOAD_DONE    

    BNE        GPTClockLoop                ; if not, loop/try again
    BX         LR                          ; if yes, return



; SSIClockInit
;
; Description:        Initializes the clock for the GPT peripheral.
;
; Operation:        Sets SSICLKGR to turn on the clock, then waits for
;                    CLKLOADCTL to indicate that the clock is on.
;
; Arguments:        None
; Returns:            None
;
; Local Variables:    None
; Global Variables:    None
;
; Error Handling:    None
;
; Algorithms:        None
; Data Structures:    None
;
; Registers Changed:    R0, R1
; Stack Depth:        0
;
; Revision History:    11/7/23  Adam Krivka  initial revision

SSIClockInit:
    MOV32      R0, PRCM_BASE_ADDR           ; prepare PRCM register base address
    MOV        R1, #SSICLKGR_ENABLE_SSI1_FORCE ; prepare CLK_EN (clock enable) value
                                            ; for GPTCLKGR
    STR        R1, [R0, #SSICLKGR_OFFSET]   ; write to GPTCLKGR register to turn
                                            ; on the clock

    MOV        R1, #CLKLOADCTL_LOAD         ; prepare LOAD value for CLKLOADCTL
    STR        R1, [R0, #CLKLOADCTL_OFFSET] ; write to CLKLOADCTL to load the clock
    ;SSIClockLoop

; Wait for CLKLOADCTL to signal clock done loading
SSIClockLoop:
    ; check if CLKLOADCTL has LOAD_DONE bit set active
    LDR        R1, [R0, #CLKLOADCTL_OFFSET]
    CMP        R1, #CLKLOADCTL_LOAD_DONE

    BNE        SSIClockLoop                ; if not, loop/try again
    BX         LR                          ; if yes, return
