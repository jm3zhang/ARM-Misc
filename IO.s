;
; This template provides working code to drive the speaker and two 7-segment LEDs
;
; Template subroutines are provided to read button inputs, configure interrupts (Lab #4) and drive the 8 LEDs on the ECE Tiva Shield
;
; In order to get the code to assemble you will have to fix lines which have had critical numbers removed - or comment them out if they're not needed yet

;
; Port C init and use kills programming the CPU!
; Port D bits 0 and 1 are disabled in mask as they're shorted with port B bits 6 and 7 on the Tiva-C
;
;

; This code is based upon InputOutput.s from the book:
;  "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
;  ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2014
;
; The code provided initializes all 3 ports (A,B,E) with the ECE LED bar array plugged into the Tiva board
; Port F with the Tiva 3 LEDs (Red, Green, Blue) and two buttons is also initialized
; Then the LEDs on each port are turned off and on with time delays - while the Tiva board R, G, B LEDs are turned on and off

        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  Start

; These equates allow one to associate a name with a value to make the code more readable

RED       EQU 0x02		; These are the values (bit locations) for various LEDs on the Tiva (Port F)
BLUE      EQU 0x04
GREEN     EQU 0x08
SW1       EQU 0x10          ; on the left side of the Tiva board
SW2       EQU 0x01          ; on the right side of the Tiva board

Start
	BL  Port_Init			; initialize input and output pins of Ports A to F
	BL	Interrupt_Init		; Init interrupts for the switches on the Tiva for Lab #4




;DEMONSTRATIONS START
;;comment out start of demo sbroutine to run reflex meter.
	mov r0,#0 ;initialize r0 
Start_Of_Demo
	BL Display_LED	
	BL tenth_of_a_second ;wait 0.1 s 
	ADD R0,#1 ;R0++ , will overflow 8bit led limit and "restart"
	B Start_Of_Demo 



	MOV R11,#13 ;initializaing non-zero random num
initial_seed
	BL RandomNum ;modifies the value in r11 and stores the value back in r11
	BL Check_Buttons
	TEQ R0,#7 ;if button is pressed
	BNE initial_seed




reflex_meter
	mov r0,#0
	BL Display_LED
	
	mov r1,#2 ;two second minimum delay
	BL Delay
	
	BL scale_random_number ;puts a random number 16 bit num into r2
	mov r1,r2
	BL Init_Del_loop ;runs a delay loop which ranges from 0-8 seconds

	
	;BL Delay ;scaled random num put into r1 and delayed
	BL tenth_ms_loop ;wait tenth of a ms
	MOV r0,#1 ; to signify button should be pressed
	BL Display_LED
	BL count_reaction_time ;r3 holds rxn time


	B perpetual_display ;after getting rxn time, enters an infinite loop to display rxn time


;DEMONSTRATIONS END

;SUBROUTINES




perpetual_display
	BL display_full_reaction_time	; displays the 32 bit value, 8 bits at a time
	BL zero_disp ;zeros display to indicate that full 32 bits have been shown
	mov R1,#5
	BL Delay	; 5 second delay after all 32 bits shown
	B perpetual_display







;;;;;;;
;;displays full 32 bits of register r3 onto the LED strip and then returns
display_full_reaction_time
	STMFD R13!,{r0,r1,r2,r14}
	mov r2,#4 ;32 bits / 8 bits = 4	
	mov r0,r3	; move r3 -> r0, so r3 is not modified on repeated display of reaction time.	

l_loop	;inner loop var
	BL Display_LED ;disp lowest 8 bits of r0
	mov r1,#2 ;two second delay to show the 8 bits
	BL Delay
	
	lsr r0,#8 ;shift 8 bits so that the
	SUBS R2,R2,#1 ;increment var for loop
	TEQ R2,#0 ;l_loop exit conditions
	BNE l_loop		
	LDMFD R13!,{r0,r1,r2,r15}
;;;;;;;

;;;;;;;;;;
;;counts rxn time in register r3 
count_reaction_time
	STMFD R13!,{r0,r1,r14}
	mov r3,#0 ;initialize r3 to zero
button_not_pressed	
	BL tenth_ms_loop
	ADD R3,R3,#1 ;increment r3 for every 0.1 ms while button is not pressed
	BL Check_Buttons
	TEQ R0,#7 ;A 7 is returned from check_buttons sbrtn if button was pressed during polling period
	BNE button_not_pressed ;continue polling if button not pressed
	
	LDMFD R13!,{r0,r1,r15}
;;;;;;;;;;;;;	
	
	
	
	
	
	
	
;;;;;;	
;Zeros (turns off) all LED's on LED strip
zero_disp 	
	STMFD R13!,{r0,r1,r2,r14}
	mov R0,#0
	BL Display_LED	
	LDMFD R13!,{r0,r1,r2,r15}
;;;;;;;	
	


;Stores a random number 16-bit number in R2
scale_random_number
	STMFD R13!,{r0,r1,r14}
	BL RandomNum ;retrieves random number from RandomNum into R11
	MOV R2,R11	
	LSR R2,#16 	;Shift 32bit random number to 16 bits, for requirement of 16 bits of randomness.
	LDMFD R13!,{r0,r1,r15}
;;;;;;;;;	
	






; ----------Display_LED----------------
; Display the 8-bit number on the 8 LEDs
; Input: R0
; Output: none
Display_LED
	STMFD		R13!,{R2, R14}		; push the LR or return address

;Input data is in R0 - the lowest 8 bits
;Use Appendix H in the lab manual to write the correct bits to port B and E to drive all 8 LEDs
;This is much easier than the Keil board, or the old Tiva shield, as you'll notice that 7 of the 8 LEDs on the ECE Tiva Shield are in order on one port

	LDR R2, =GPIO_PORTB + (PORT_B_MASK << 2)
	STR R0, [R2, #GPIO_DATA_OFFSET]		; write output to port B where the LEDs are

	LDR R2, =GPIO_PORTE + (PORT_E_MASK << 2)
	STR R0, [R2, #GPIO_DATA_OFFSET]		; write output to port B where the LEDs are

	LDMFD		R13!,{R2, R15}		; pull the LR or return address and return


;------------RandomNum------------
; R11 holds a 16-bit random number via a pseudo-random sequence as per the Linear feedback shift register (Fibonacci) on WikiPedia
; R11 holds a non-zero 16-bit number.  If a zero is fed in, as the seed, the pseudo-random sequence will stay stuck at 0
; Take as many bits of R11 as you need.  If you take the lowest 4 bits then you get a number between 1 and 15.
;   If you take bits 5..1 you'll get a number between 0 and 15 (assuming you right shift by 1 bit).
;
; R11 MUST be initialized to a non-zero 16-bit value at the start of the program OR ELSE!
; R11 can be read anywhere in the code but must only be written to by this subroutine
;
RandomNum		STMFD		R13!,{R1, R2, R3, R14}

				AND			R1, R11, #0x8000
				AND			R2, R11, #0x2000
				LSL			R2, #2
				EOR			R3, R1, R2
				AND			R1, R11, #0x1000
				LSL			R1, #3
				EOR			R3, R3, R1
				AND			R1, R11, #0x0400
				LSL			R1, #5
				EOR			R3, R3, R1		; the new bit to go into the LSB is present
				LSR			R3, #15
				LSL			R11, #1
				ORR			R11, R11, R3
				
				
				LDMFD		R13!,{R1, R2, R3, R15}




; Check Tiva Buttons subroutine
; R0 returns 0 if no button pressed and non-zero upon a button press (SW1 and SW2 on the Tiva board)
;
Check_Buttons
	STMFD		R13!,{R1, R14}		; push the LR or return address
	LDR R1, =GPIO_PORTF_DATA_R ; pointer to Port F data register
	LDR R0, [R1]		; Read in the current Port F inputs
	AND R0, #0x7		; return only the bits that have a button ;
	EORS R0, #0x7		; bits go to 0 if button is pressed - so invert them
	LDMFD		R13!,{R1, R15}		; pull the LR or return address and return




;LOOPS

	
; Delay subroutine - delay for a fixed amount of time
; Input: R1 - how many times do we multiply the fixed delay
; Output: none


Init_DELAY             EQU 0x29a ;special loop with delay that scales properly to 16 bits of randomness, generating a delay of 0-8 seconds. The 2-10 second 
								 ; requirement is reached by performing a 2 second delay before this is run.	
								 ;Init_DELAY calculated as follows,
								 ;		 	((5451849 iterations))         8 s                1
								 ; delay =    -------------------- X  ----------------- X ------------------- == ~666 = 0x29a
								 ;                   s                                  (2^16 -1 iterations)
Init_Del_loop
	STMFD		R13!,{R0, R1, R14}		; push the LR or return address

initial_delay_outer_loop
	TEQ R1, #0
	BEQ initial_done_delay
	SUB R1, #1
	LDR R0, =Init_DELAY  	       ; 
inner_delay_loop
    SUBS R0, R0, #1                 ; R0 = R0 - 1 (count = count - 1) and set N, Z, C status bits
	BNE	inner_delay_loop
	B initial_delay_outer_loop
	; Note: For SUBs the "s" suffix means to set the status bits, without this the loops would not exit
initial_done_delay
	LDMFD		R13!,{R0, R1, R15}		; pull the LR or return address and return





SOMEDELAY             EQU 0x533049 ;one second delay

Delay
	STMFD		R13!,{R0, R1, R14}		; push the LR or return address

delay_outer_loop
	TEQ R1, #0
	BEQ done_delay
	SUB R1, #1
	LDR R0, =SOMEDELAY   	       ; R0 = a value to get about a second delay
delay_loop
    SUBS R0, R0, #1                 ; R0 = R0 - 1 (count = count - 1) and set N, Z, C status bits
	BNE	delay_loop
	B delay_outer_loop
	; Note: For SUBs the "s" suffix means to set the status bits, without this the loops would not exit
done_delay
	LDMFD		R13!,{R0, R1, R15}		; pull the LR or return address and return


tenth_of_a_second
	STMFD		R13!, {r0,r1,r14}
	mov R1,#1000
j     ;inner loop var
	BL tenth_ms_loop
	SUBS R1,#1
	BNE j	
	
	LDMFD		R13!, {R0, R1, R15}

tenth_ms_loop
	STMFD		R13!, {r0,r1,r14}
	mov r0, #0x186 ; 0.1 ms (0x186) ;was 0x27 
f ;inner loop var
	SUBS R0,#1
	TEQ R0,#1
	BNE f
	LDMFD		R13!, {R0, R1, R15}
	





;
; Code to setup interrupts
; Table 10-4 GPIO Interrupt Configuration Example


; Registers have one bit (starting at LSB - same as other I/O such as LEDS) for each I/O pin

GPIO_IS_OFFSET  EQU 0x404	;GPIOIS - Interrupt Sense : 0 = edge, 1 = level interrupt
GPIO_IBE_OFFSET  EQU 0x408	;GPIOIBE - Interrupt Both Edges : 0 = single edge, 1 = both edges
GPIO_IEV_OFFSET  EQU 0x40c	;GPIOIEV - Interrupt Event : 0 = low level or falling edge, 1= high level or rising edge
GPIO_IM_OFFSET  EQU 0x410	;GPIOIM - Interrupt Mask : 0 = masked, 1 = unmasked
GPIO_RIS_OFFSET  EQU 0x414		; Raw Interrupt Status - READ ONLY
GPIO_MIS_OFFSET  EQU 0x418		; Masked Interrupt Status - READ ONLY
GPIO_ICR_OFFSET  EQU 0x41c		; Interrupt Clear - writing a 1 clears the RIS and MIS registers

Interrupt_Init
	STMFD		R13!,{R14}		; push the LR or return address

;Program the GPIOIS, GPIOIBE, GPIOEV, and GPIOIM registers to configure the type, event,
;and mask of the interrupts for each port.
;Note: To prevent false interrupts, the following steps should be taken when re-configuring
;GPIO edge and interrupt sense registers:

;a. Mask the corresponding port by clearing the IME field in the GPIOIM register.
	
	LDR R1, =GPIO_PORTF
    MOV R0, #0x00             ; 0 means mask or block interrupts
    STR R0, [R1, #GPIO_IM_OFFSET]	; mask interrupts from happening

; b. Configure the IS field in the GPIOIS register and the IBE field in the GPIOIBE register.

    MOV R0, #0x00             ; 0 means edge detecting interrupt
    STR R0, [R1, #GPIO_IS_OFFSET]	; 

    MOV R0, #0x00             ; 0 means falling edge detecting
    STR R0, [R1, #GPIO_IEV_OFFSET]	; 

    MOV R0, #0x00             ; 0 means single edge detection
    STR R0, [R1, #GPIO_IBE_OFFSET]	; 

;c. Clear the GPIORIS register using the ICR register to clear any pending interrupts.
; The switches are bits 0 and 4
    MOV R0, #0x17             ; 0 means mask or block interrupts
    STR R0, [R1, #GPIO_ICR_OFFSET]	; clear any interrupts recieved

;d. Unmask the port by setting the IME field in the GPIOIM register.
; Set the appropiate bit to 1 to enable interrupts for only the one switch required
    MOV R0, #0x17             ; 0 means mask or block interrupts

	STR R0, [R1, #GPIO_IM_OFFSET]	; mask interrupts from happening

;Looking in the Startup.s file one will find an EXPORT of the address for interrupt handlers, one for each GPIO port

; Interrupt Enable Registers
CORE_PERIPHERALS 		EQU 0xe000e000
INTERRUPT_EN0_OFFSET 	EQU 0x100

; The Interrupt Number (Bit in Interrupt Registers) value written to the EN0 register to enable port F interrupts can be found in Table 2-9 (page 104)
	MOV R0,# 0xB8	; this 32-bit value enables GPIO Port F Interrupts via a single bit set

	LDR R1, =CORE_PERIPHERALS
	STR R0, [R1, #INTERRUPT_EN0_OFFSET]		; GPIO Interrupts require this enable

	LDMFD		R13!,{R15}		; push the LR or return address
	
;Section 3.1.2 Nested Vector Interrupt Controller

;The Cortex-M4F processor supports interrupts and system exceptions. The processor and the
;Nested Vectored Interrupt Controller (NVIC) prioritize and handle all exceptions. An exception
;changes the normal flow of software control. The processor uses Handler mode to handle all
;exceptions except for reset. See “Exception Entry and Return” on page 108 for more information.
;The NVIC registers control interrupt handling. See “Nested Vectored Interrupt Controller
;(NVIC)” on page 124 for more information.

;Table 3-8 on page 134 details interrupt Set / Clear 
; they allow one to enable individual interrupts and DIS? lets one disable individual interrupt numbers

; Table 2-9 Interrupts on page 104 details interrupt number / bit assignments
; Port F - Bit 30
; Timer 0A Bit 19
; Timer 0B Bit 20
 
;For edge-triggered interrupts, software must clear the interrupt to enable any further interrupts.


; NOTE: The NMI (non-maskable interrupt) is on PF0.  That means that
; the Alternate Function Select, Pull-Up Resistor, Pull-Down Resistor,
; and Digital Enable are all locked for PF0 until a value of 0x4C4F434B
; is written to the Port F GPIO Lock Register.  After Port F is
; unlocked, bit 0 of the Port F GPIO Commit Register must be set to
; allow access to PF0's control registers.  On the LM4F120, the other
; bits of the Port F GPIO Commit Register are hard-wired to 1, meaning
; that the rest of Port F can always be freely re-configured at any
; time.  Requiring this procedure makes it unlikely to accidentally
; re-configure the JTAG and NMI pins as GPIO, which can lock the
; debugger out of the processor and make it permanently unable to be
; debugged or re-programmed.

GPIO_PORTF_DIR_R   EQU 0x40025400		; Port F Data Direction Register setting pins as input or output
GPIO_PORTF_DATA_R  EQU 0x400253FC		; address for reading button inputs
	
; These are the configuration registers which should not be touched
; Port Base addresses for the legacy (not high-performance) interface to I/O ports
GPIO_PORTA			EQU 0x40004000
GPIO_PORTB			EQU 0x40005000
GPIO_PORTC			EQU 0x40006000
GPIO_PORTD			EQU 0x40007000
GPIO_PORTE			EQU 0x40024000
GPIO_PORTF			EQU 0x40025000

; These are the masks for pins which are outputs
PORT_A_MASK			EQU 0xfc	;0xE0		; PA7,6,5 are outputs for LEDs
PORT_B_MASK			EQU 0xfd	;3f	; exclude B2:3 0xff	;33		; PB5,4,1,0 are outputs %0011 0011 
PORT_C_MASK			EQU 0x30	; this hangs the CPU 0xf0	
PORT_D_MASK			EQU 0xcc	;exclude d7 0xcf	Disable D0, D1 due to short with B6, B7
PORT_E_MASK			EQU 0x2	;0x30		; PE5,4 are outputs %0011 0000
PORT_F_MASK			EQU 0x0e		; PF has LEDs on PF 1,2,3 and buttons PF0, PF4 (don't enable buttons as outputs)
	
; Offsets are from table 10-6 on page 660
GPIO_DATA_OFFSET	EQU 0x000		; Data address is the base address - YOU HAVE TO ADD AN ADDRESS MASK TOO to read or write this!!
GPIO_DIR_OFFSET		EQU 0x400		; Direction register
GPIO_AFSEL_OFFSET EQU 0x420			; Alternate Function SELection
GPIO_PUR_OFFSET   EQU 0x510			; Pull Up Resistors
GPIO_DEN_OFFSET   EQU 0x51C			; Digital ENable
GPIO_LOCK_OFFSET  EQU 0x520
GPIO_CR_OFFSET    EQU 0x524
GPIO_AMSEL_OFFSET EQU 0x528			; Analog Mode SELect
GPIO_PCTL_OFFSET  EQU 0x52C

SYSCTL_HBCTL  EQU   0x400FE06C		; high performance bus control for ports A to F

GPIO_LOCK_KEY      EQU 0x4C4F434B  ; Unlocks the GPIO_CR register
SYSCTL_RCGCGPIO_R  EQU   0x400FE608		; Register to enable clocks to the I/O port hardware

;------------Port_Init------------
; Initialize GPIO Port F for negative logic switches on PF0 and
; PF4 as the Launchpad is wired.  Weak internal pull-up
; resistors are enabled, and the NMI functionality on PF0 is
; disabled.  Make the RGB LED's pins outputs.
; Input: none
; Output: none
; Modifies: R0, R1, R2, R3
Port_Init
	STMFD		R13!,{R14}		; push the LR or return address

; First enable the clock to the I/O ports, by default the clocks are off to save power
; If a clock is not enabled to a port and you access it - then the processor hard faults
	LDR R1, =SYSCTL_RCGCGPIO_R      ; activate clock for Ports (see page 340)
    LDR R0, [R1]                 
    ORR R0, R0, #0x3F               ; turn on clock to all 6 ports (A to F, bits 0 to 5)
    STR R0, [R1]                  
    NOP
    NOP                             ; allow time for clock to finish
	
; Set all ports to APB bus instead of AHB - this should be unnecessary
;	LDR R1, =SYSCTL_HBCTL
;	LDR R0, [R1]
;	AND R0, #0xFFFFFFE0		; set Ports A thru F to APB (0) and leave the rest at their default
;	STR R0, [R1]

; Page 650, Table 10-1 GPIO Pins with Special Considerations.
; These pins must be left as configured after reset:
;  PA[5:0] (UART0 and SSIO), PB[3:2] (I2C), PC[3:0] (JTAG)

; Initialize the I/O ports A, B, E, F via a common subroutine Port_Init_Individual
; Call Port_Init_Individual with the following paramaters passed:
; R1 is the base port address
; R2 is the output pin mask (which bits are outputs)
; R3 is the input pin mask  (which bits get configured as inputs)

	MOV R3, #0x00				; Select no pins as input (unless it's changed as for port F)
	
; Init Port A, B, E are by default GPIO - set all output pins used to a 1 to enable them
;   and leave all of the other pins as previously configured!
    LDR R1, =GPIO_PORTA
    MOV R2, #PORT_A_MASK            ; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port B
    LDR R1, =GPIO_PORTB
    MOV R2, #PORT_B_MASK            ; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port C
    LDR R1, =GPIO_PORTC
    MOV R2, #PORT_C_MASK
	;BL Port_Init_Individual

; Init Port D
    LDR R1, =GPIO_PORTD
    MOV R2, #PORT_D_MASK
	BL Port_Init_Individual

; Init Port E
	LDR R1, =GPIO_PORTE
    MOV R2, #PORT_E_MASK			; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port F
	LDR R1, =GPIO_PORTF
    MOV R2, #PORT_F_MASK		; enable commit for Port, 1 means allow access
	MOV R3, #0x11				; enable weak pull-up on PF0 and PF4 (buttons)
	BL Port_Init_Individual

	LDMFD		R13!,{R15}		; pull the LR or return address from the stack and return


;------------Port_Init_Individual------------
; Initialize one GPIO Port with select bits as inputs or outputs
; Output: none
; Input: R1, R2, R3
; R1 has to be the port address
; R2 has to hold the mask for output pins
; R3 has to be the mask for input pins
; Modifies: R0

Port_Init_Individual
	STMFD		R13!,{R14}		; push the LR or return address
    LDR R0, =0x4C4F434B             ; unlock GPIO Port F Commit Register
    STR R0, [R1, #GPIO_LOCK_OFFSET]	; 2) unlock the lock register
	ORR R0, R2, R3					; all access to inputs and outputs as masked in R2 and R3
    STR R0, [R1, #GPIO_CR_OFFSET]	; enable commit for Port F
    MOV R0, #0                      ; 0 means analog is off
    STR R0, [R1, #GPIO_AMSEL_OFFSET]	; 3) disable analog functionality
    MOV R0, #0x00000000             ; 0 means configure Port F as GPIO
    STR R0, [R1, #GPIO_PCTL_OFFSET]	; 4) configure as GPIO
    LDR R0, [R1, #GPIO_DIR_OFFSET]	; 5) read default direction register configuration
    ORR R0, R2						; ORR in only the bits we want as outputs
    STR R0, [R1, #GPIO_DIR_OFFSET]	; 5) set direction register
    MOV R0, #0                      ; 0 means disable alternate function 
    STR R0, [R1, #GPIO_AFSEL_OFFSET]	; 6) regular port function
    STR R3, [R1, #GPIO_PUR_OFFSET]	; pull-up resistors for PF4,PF0
    MOV R0, #0xFF                   ; 1 means enable digital I/O
    STR R0, [R1, #GPIO_DEN_OFFSET]
	LDMFD		R13!,{R15}		; pull the LR or return address and return

	ALIGN
Port_Table
	DCD	GPIO_PORTA + (PORT_A_MASK << 2)		; DCD - Define Constant Double Word (32-bits)
	DCD	GPIO_PORTB + (PORT_B_MASK << 2), GPIO_PORTC + (PORT_C_MASK << 2)
	DCD	GPIO_PORTD + (PORT_D_MASK << 2), GPIO_PORTE + (PORT_E_MASK << 2)
	DCD	GPIO_PORTF + (PORT_F_MASK << 2), 0

	ALIGN
Seven_Seg_Table
	DCW 0x7e, 0x0c, 0xb6, 0x9e, 0xcc, 0xda, 0xfa, 0x0e, 0xfe, 0xce	; 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
	DCW 0xee, 0xf8, 0xb0, 0xbc, 0xf2, 0xe2		; A, B, C, D, E, F with LSB being 0 for the DP
	
    ALIGN                           ; make sure the end of this section is aligned
    END                             ; end of file
		
;;;;;; 	POST LAB QUESTIONS ;;;;;

;1) In a 32-bit value, ~119.3 hours of reaction time could be stored ((2^32)-1)/(10000*60*60)
;In a 24-bit value, ~27.96 minutes of reaction time could be stored ((2^24)-1)/(10000*60)
;In a 16-bit value, ~6.55 seconds of reaction time could be stored ((2^16)-1)/(10000)
;In a 8-bit value, ~25.5 ms of reaction time could be stored ((2^8)-1)/(10)

;2) The average human reaction time ranges from around 200-400ms, thus making a 16-bit value the best value.
;
;
; PROOF TIME DELAY MEETS SPEC
;  Our strategy to implement a 2-10 second delay is implemented as follows,
;  1) Call our one second "Delay" subroutine to delay for two seconds. That ensures we delay a minimum of two seconds.
;  2) The random subroutine places 32 random bits in R11. We then shift this value down to 16 bits in order to get full 16 bit randomness requirements.
;  3) Our "Delay" subroutine uses 5451849 iterations of the loop to delay by a second.
;  4) The seperate constant "Init_DELAY" to delay 0-8 seconds is calculated as shown below.
;
;
; ;Init_DELAY calculated as follows,
;		 	((5451849 iterations))         8 s                1
; delay =    -------------------- X  ----------------- X ------------------- == ~666 = 0x29a
;                   s                                    (2^16 -1 iterations)
;
;
; 5) The max number in the 16 bits of randomness is (2^16 -1) and the minimum is 0.
; 6) In the maximum case, the loop would run the full 8 seconds, in the minimum case the loop will run for 0 seconds.
; 7) Because we offset this delay by two seconds, this results in a range of 2-10 second delay overall.
;
;