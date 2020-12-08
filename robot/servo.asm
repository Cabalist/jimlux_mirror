	TITLE " R/C servo controller"
		LIST P=16F84
;
;*****************************************************************************
;**  Syncronous serial port controlled R/C servo controller
;**  routines for Microchip's PIC16F84 8-bit CMOS single chip
;**  microcomputer.
;**
;**  
;**   Program:  SERVO.ASM 
;**
;**   Revision	Revision Date:
;**     0.00	1-20-00		Initial release
;**     0.01	1-30-00		Modified for new PCB design and Enabled Watchdog
;**				timer and address switch checking.
;**			
;**
;**
;**     Part used = PIC16F84
;**
;**	Program Configuration Bits:
;**    -----------------------------
;**        Oscillator: HS
;**    Watchdog Timer: ON
;**    Power Up Timer: ON
;**      Code Protect: OFF
;**
;*****************************************************************************
;
;-----------------------------------------------------------------------------
;       File Register Assignment
;-----------------------------------------------------------------------------
;
	include <p16F84.inc>
;
; Serial port registers
;
SR_POS4    	EQU     20h     ; 5th byte of serial in shift register
SR_POS3    	EQU     21h     ; 4th byte of serial in shift register
SR_POS2    	EQU     22h     ; 3th byte of serial in shift register
SR_POS1  	EQU     23h     ; 2th byte of serial in shift register
SR_CMD     	EQU     24h	; 1th byte of serial in shift register
SR_BITCNT	EQU	25h	; bit count for serial input
LAST_STATE	EQU	26h	;
THIS_STATE	EQU	27h	;
TEMP_CMD	EQU	28h	;
;
TEMP_W		EQU	29h	; Storage for W register
TEMP_STAT	EQU	2Ah	; Storage for STATUS register
TEMP_INTR	EQU	2Bh	; Interrupt temp storage
;
SERVO1_POS	EQU	2Ch	; Pulse width of servo1, 0==1ms... 255==2ms
SERVO2_POS	EQU	2Dh	; Pulse width of servo1, 0==1ms... 255==2ms
SERVO3_POS	EQU	2Eh	; Pulse width of servo1, 0==1ms... 255==2ms
SERVO4_POS	EQU	2Fh	; Pulse width of servo1, 0==1ms... 255==2ms
TIMESTEPS	EQU	30h	; Number of 20ms steps to reach new position
PWM_STATE	EQU	31h	; PWM state register
IPD_COUNT	EQU	32h	; Counter for 1MS delays between pulses
;
;-----------------------------------------------------------------------------
;                     Bit Assignments
;-----------------------------------------------------------------------------

; SSP Device Ports
SSP_PORT 	EQU	PORTA
SSP_TRIS   	EQU	TRISA

; SSP Device Bits
SSP_SO  	EQU     0               ; RA0, serial data out
SSP_CAK  	EQU     1               ; RA1, clock acknowledge
SSP_SI  	EQU     2               ; RA2, data in
SSP_CLK 	EQU     3               ; RA3, clock
SSP_CS  	EQU     4               ; RA4, chip select
; SSP Device Bit Masks
SSP_SO_MASK	EQU	01h		; RA0, data out BITMASK
SSP_CAK_MASK	EQU	02h		; RA1, clk acknowledge BITMASK
SSP_SI_MASK	EQU	04h		; RA2, data in BITMASK
SSP_CLK_MASK	EQU	08h		; RA3, clk BITMASK
SSP_CS_MASK	EQU	10h		; RA4, chip select BITMASK

; DIP SWITCH Port
SWITCH_PORT 	EQU	PORTB
SWITCH_TRIS   	EQU	TRISB

; DIP SWITCH Bits
SWITCH_1	EQU	0		; DIP SWITCH bit 1
SWITCH_2	EQU	1		; DIP SWITCH bit 2
SWITCH_4	EQU	2		; DIP SWITCH bit 4
SWITCH_8	EQU	3		; DIP SWITCH bit 8
SWITCH_MASK	EQU	0fh		; MASK for all 4 switchs
CMD_ADDR_MASK 	EQU	0f0h		; MASK for address in CMD byte
CMD_STEP_MASK	EQU	0fh		; MASK for time step bits

; PWM SWITCH Port
PWM_PORT 	EQU	PORTB
PWM_TRIS   	EQU	TRISB

; PWM output Bits
PWM_1		EQU	4		; Pulse Width Modulation Ouput 1
PWM_2		EQU	5		; Pulse Width Modulation Ouput 2
PWM_3		EQU	6		; Pulse Width Modulation Ouput 3
PWM_4		EQU	7		; Pulse Width Modulation Ouput 4
PWM_1_BIT	EQU	10h		; Pulse Width Modulation Ouput 1
PWM_2_BIT	EQU	20h		; Pulse Width Modulation Ouput 2
PWM_3_BIT	EQU	40h		; Pulse Width Modulation Ouput 3
PWM_4_BIT	EQU	80h		; Pulse Width Modulation Ouput 4
;
PWM_STATE1A	EQU	0
PWM_STATE1B	EQU	1
PWM_STATE1C	EQU	2
PWM_STATE1D	EQU	3
PWM_STATE2A	EQU	4
PWM_STATE2B	EQU	5
PWM_STATE2C	EQU	6
PWM_STATE2D	EQU	7
PWM_STATE3A	EQU	8
PWM_STATE3B	EQU	9
PWM_STATE3C	EQU	0ah
PWM_STATE3D	EQU	0bh
PWM_STATE4A	EQU	0ch
PWM_STATE4B	EQU	0dh
PWM_STATE4C	EQU	0eh
PWM_STATE4D	EQU	0fh

IPULSE_DELAY	EQU	3		;Number of 1MS delays between pulses
; Timer prescaler value
					; 8 MHz crystal provides 2MHz into
					; prescaler.  We need 250KHz so
TIMER_PRESCALE_VALUE 	EQU	2	; set to Divide-by 8
TIMER_1MS		EQU	0	; Timer value for 1MS delay


;End of files/bits equate

	ORG	00h			; Reset Vector
	goto	start

	ORG	04h			; Interrupt Vector
	goto	interrupt_service

	ORG	10h			; Begining of Program space
start:
; Now in bank 0
	bcf	STATUS, RP0		; Select register bank 0

	clrf	INTCON			; Turn off all interrupts
	call	init_ports		; Initialize I/O ports
	call	init_timer		; Initialize Timer
	
	movfw	SSP_PORT		; Read SSP port
	movwf	LAST_STATE		; Save in LAST_STATE
;
	movlw	80h			; Set initial servo positions 1.5MS (center)
	movwf	SERVO1_POS
	movwf	SERVO2_POS
	movwf	SERVO3_POS
	movwf	SERVO4_POS
;
; Turn on Interrupts
	bsf	INTCON, GIE		; Turn on global interrupt enable
;
;
spin:
	bcf	STATUS,RP0		; switch to bank 0
	movfw	LAST_STATE
	movwf	TEMP_INTR		; Put LAST_STATE into TEMP_INTR
	movfw	SSP_PORT		; Read SSP port into W
	movwf	LAST_STATE		; Save in LAST_STATE
	xorwf	TEMP_INTR, F		; XOR PORT_DATA with LAST & save
;
; Check state of ssp clock pin
	btfss	TEMP_INTR, SSP_CLK	; Did SSP_CLK bit change?
	goto	ssp_cs			; No, check SSP_CS
; SSP_CLK bit did change
	btfss	LAST_STATE, SSP_CLK	; Is SSP_CLK bit high?
	goto	ssp_clk_low		; Clock is low
;
; SSP_CLK bit is high
	bsf	SSP_PORT, SSP_CAK	; Set clock acknowledge high
	call	ssp_clock_went_high
	goto	ssp_cs			; Now check SSP_CS

; SSP_CLK bit is high
ssp_clk_low:
	bcf	SSP_PORT, SSP_CAK	; Set clock acknowledge low
;
ssp_cs:
	btfss	TEMP_INTR, SSP_CS	; Did SSP_CS bit change?
	goto	spin			; No, done
;
; SSP_CS bit did change
	btfss	LAST_STATE, SSP_CS	; Is SSP_CS bit high?
	goto	ssp_cs_low		; No, it's low
;
; SSP_CS bit went high
	call	ssp_cs_went_high
	goto	spin			; Done
;
; SSP_CS bit went low
ssp_cs_low:
	call	ssp_cs_went_low
	goto	spin			; Done

;*
;*
;*
init_timer:
; Now in bank 1
	clrwdt 				; Clear WDT and prescaler
	bsf	STATUS, RP0		; Select register bank 1
;
	bcf	OPTION_REG, T0CS	; Timer clock source internal
	bcf	OPTION_REG, T0SE	; Clock on positive edge
	bcf	OPTION_REG, PSA		; Prescaler assigned to TMR0
;
; T0CS = 0, Internal instruction cycle clock
; T0SE = 0, Don't care
; PSA = 0,  Prescaler assigned to TMR0
; PS<2:0> = TIMER_PRESCALE_VALUE  (value to create a clock of 250 KHz)
;	    for a 8MHz crystal, PS<2:0> = 002b
;
	clrwdt 				; Clear WDT and prescaler
	movfw	OPTION_REG		; Load OPTION_REG
	andlw	0c0h
	iorlw   TIMER_PRESCALE_VALUE
	movwf	OPTION_REG		; Save OPTION_REG
	
	bsf	INTCON, T0IE		; turn on timer interrupt
;
; Now in bank 0
	bcf	STATUS, RP0		; Select register bank 0
;
	movlw	PWM_STATE1A
	movwf	PWM_STATE
	return

init_ports:
; Now in bank 1
	bsf	STATUS, RP0		; Select register bank 1
;	
	movlw	~(PWM_1_BIT | PWM_2_BIT | PWM_3_BIT | PWM_4_BIT)
	andwf	PWM_TRIS, F		; Set PWM pins as outputs
;
	movlw	~(SSP_CAK_MASK | SSP_SO_MASK)
	andwf	SSP_TRIS, F		; Setup SSP outputs
;
	movlw	(SSP_SI_MASK | SSP_CLK_MASK | SSP_CS_MASK)
	iorwf	SSP_TRIS, F		; Setup SSP inputs
;
	movlw	SWITCH_MASK
	iorwf	SWITCH_TRIS, F		; Set switch pins as inputs
;
	bcf	OPTION_REG, NOT_RBPU	; turn on weak pull-ups in reg b
;
; Now in bank 0
	bcf	STATUS, RP0		; Select register bank 0
;
; Turn off PWM outputs
	movlw	~(PWM_1_BIT | PWM_2_BIT | PWM_3_BIT | PWM_4_BIT)
	andwf	PWM_PORT, F
;
	return				; Done setting up ports

;**
;** BEGIN PWM state machine
;**
;
; State dispatcher
;
PWM_controller:
	clrwdt 				; Clear Watch Dog Timer
	bcf	STATUS, RP0		; Select register bank 0
;
	movfw	PWM_STATE
	addwf	PCL, F			; Add input to pc to create jump table
	goto	state_pwm1a		; Turn on PWM1, turn off others and delay 1MS
	goto	state_pwm1b		; delay 1MS * SERVO1_POS / 256
	goto	state_pwm1c		; delay until 2MS total
	goto	state_pwm1d		; delay 1MS * IPULSE_DELAY
	goto	state_pwm2a		; Turn on PWM1, turn off others and delay 1MS
	goto	state_pwm2b		; delay 1MS * SERVO2_POS / 256
	goto	state_pwm2c		; delay until 2MS total
	goto	state_pwm2d		; delay 1MS * IPULSE_DELAY
	goto	state_pwm3a		; Turn on PWM1, turn off others and delay 1MS
	goto	state_pwm3b		; delay 1MS * SERVO3_POS / 256
	goto	state_pwm3c		; delay until 2MS total
	goto	state_pwm3d		; delay 1MS * IPULSE_DELAY
	goto	state_pwm4a		; Turn on PWM1, turn off others and delay 1MS
	goto	state_pwm4b		; delay 1MS * SERVO4_POS / 256
	goto	state_pwm4c		; delay until 2MS total
	goto	state_pwm4d		; delay 1MS * IPULSE_DELAY


;
; Turn on PWM1 turn off PWM2,3,4
; Set timer to 1MS
;
state_pwm1a:
; turn on PWM1
	movlw	PWM_1_BIT		; Turn on PWM1
	iorwf	PWM_PORT, F
;
	movlw	TIMER_1MS		; Load timer with 1ms delay
	movwf	TMR0
;
; move to next state
	movlw	PWM_STATE1B
	movwf	PWM_STATE
	return

;
; Set timer to delay 1MS * SERVO1_POS / 256
;
state_pwm1b:
	movfw	SERVO1_POS	; Load timer with delay of 1MS * SERVO1_POS / 256
	xorlw	0ffh		; Complement Timer value
	movwf	TMR0
;
; move to next state
	movlw	PWM_STATE1C
	movwf	PWM_STATE
	return

;
; Delay the remainder of the 1MS period
; Set timer to delay 1MS - (1MS * SERVO2_POS / 256)
;
;
state_pwm1c:
; Turn off all pwm outputs
	movlw	~(PWM_1_BIT | PWM_2_BIT | PWM_3_BIT | PWM_4_BIT)
	andwf	PWM_PORT, F
;
	movfw	SERVO1_POS		; Load timer with remaider of 1MS
	movwf	TMR0

	movlw	IPULSE_DELAY		; Number of 1MS delays between pulses
	movwf	IPD_COUNT
;
	movlw	PWM_STATE1D		; Move to next state
	movwf	PWM_STATE
	return

state_pwm1d:
	movlw	TIMER_1MS		; Load timer with 1ms delay
	movwf	TMR0
;
; move to next state
	decfsz	IPD_COUNT, F		; Decrement IPD_COUNT & skip next if zero
	goto	state_pwm1d_exit	; Not done yet
	movlw	PWM_STATE2A		; Move to next state
	movwf	PWM_STATE
state_pwm1d_exit:
	return


;
; Turn on PWM2 turn off PWM1,3,4
; Set timer to 1MS
;
state_pwm2a:
	movlw	PWM_2_BIT		; Turn on PWM2
	iorwf	PWM_PORT, F
;
	movlw	TIMER_1MS 		; Load timer with 1ms delay
	movwf	TMR0
;
; move to next state
	movlw	PWM_STATE2B
	movwf	PWM_STATE
	return

;
; Set timer to delay 1MS * SERVO2_POS / 256
;
state_pwm2b:
	movfw	SERVO2_POS	; Load timer with delay of 1MS * SERVO1_POS / 256
	xorlw	0ffh		; Complement Timer value
	movwf	TMR0
;
; move to next state
	movlw	PWM_STATE2C
	movwf	PWM_STATE
	return

;
; Delay the remainder of the 1MS period
; Set timer to delay 1MS - (1MS * SERVO2_POS / 256)
;
;
state_pwm2c:
; Turn off all pwm outputs
	movlw	~(PWM_1_BIT | PWM_2_BIT | PWM_3_BIT | PWM_4_BIT)
	andwf	PWM_PORT, F
;
	movfw	SERVO2_POS		;Load timer with delay of 1MS * SERVO2_POS / 256
	movwf	TMR0
;
; move to next state
	movlw	IPULSE_DELAY
	movwf	IPD_COUNT
;
	movlw	PWM_STATE2D
	movwf	PWM_STATE
	return

state_pwm2d:
	movlw	TIMER_1MS		;Load timer with 1ms delay
	movwf	TMR0
;
; move to next state
	decfsz	IPD_COUNT, F		;decrement IPD_COUNT & skip next if zero
	goto	state_pwm2d_exit	;Not done yet
	movlw	PWM_STATE3A		;move to next state
	movwf	PWM_STATE
state_pwm2d_exit
	return


;
; Turn on PWM3 turn off PWM1,2,4
; Set timer to 1MS
;
state_pwm3a:
	movlw	PWM_3_BIT		; turn on PWM3
	iorwf	PWM_PORT, F
;
	movlw	TIMER_1MS		; Load timer with 1ms delay
	movwf	TMR0
;
; move to next state
	movlw	PWM_STATE3B
	movwf	PWM_STATE
	return

;
; Set timer to delay 1MS * SERVO3_POS / 256
;
state_pwm3b:
	movfw	SERVO3_POS	; Load timer with delay of 1MS * SERVO1_POS / 256
	xorlw	0ffh		; Complement Timer value
	movwf	TMR0
;
; move to next state
	movlw	PWM_STATE3C
	movwf	PWM_STATE
	return

;
; Delay the remainder of the 1MS period
; Set timer to delay 1MS - (1MS * SERVO2_POS / 256)
;
state_pwm3c:
; Turn off all pwm outputs
	movlw	~(PWM_1_BIT | PWM_2_BIT | PWM_3_BIT | PWM_4_BIT)
	andwf	PWM_PORT, F
;
	movfw	SERVO3_POS
	movwf	TMR0
;
	movlw	IPULSE_DELAY
	movwf	IPD_COUNT
;
; move to next state
	movlw	PWM_STATE3D
	movwf	PWM_STATE
	return

state_pwm3d:
	movlw	TIMER_1MS		;Load timer with 1ms delay
	movwf	TMR0
;
; move to next state
	decfsz	IPD_COUNT, F		;decrement IPD_COUNT & skip next if zero
	goto	state_pwm3d_exit	;Not done yet
	movlw	PWM_STATE4A		;move to next state
	movwf	PWM_STATE
state_pwm3d_exit:
	return


;
; Turn on PWM4 turn off PWM1,2,3
; Set timer to 1MS
;
state_pwm4a:
; turn on PWM4
	movlw	PWM_4_BIT
	iorwf	PWM_PORT, F
;
	movlw	TIMER_1MS		;Load timer with 1ms delay
	movwf	TMR0
;
; move to next state
	movlw	PWM_STATE4B
	movwf	PWM_STATE
	return

;
; Set timer to delay 1MS * SERVO4_POS / 256
;
state_pwm4b:
	movfw	SERVO4_POS		; Load timer with delay of 1MS * SERVO1_POS / 256
	xorlw	0ffh			; Complement Timer value
	movwf	TMR0
;
; move to next state
	movlw	PWM_STATE4C
	movwf	PWM_STATE
	return

;
; Delay the remainder of the 1MS period
; Set timer to delay 1MS - (1MS * SERVO2_POS / 256)
;
;
state_pwm4c:
; Turn off all pwm outputs
	movlw	~(PWM_1_BIT | PWM_2_BIT | PWM_3_BIT | PWM_4_BIT)
	andwf	PWM_PORT, F
;
	movfw	SERVO4_POS		;Load timer with delay of 1MS - (1MS * SERVO4_POS / 256)
	movwf	TMR0
;
; move to next state
	movlw	IPULSE_DELAY
	movwf	IPD_COUNT
	movlw	PWM_STATE4D
	movwf	PWM_STATE
	return

state_pwm4d:
	movlw	TIMER_1MS		;Load timer with 1ms delay
	movwf	TMR0
;
; move to next state
	decfsz	IPD_COUNT, F		;decrement IPD_COUNT & skip next if zero
	goto	state_pwm4d_exit	;Not done yet
	movlw	PWM_STATE1A		;move to next state
	movwf	PWM_STATE
state_pwm4d_exit:
	return

;**
;** END PWM state machine
;**

;
;
;
ssp_clock_went_high:
; Shift SSP_SI bit into 5 byte shift register
	bcf	STATUS, RP0		; switch to bank 0
	bcf	STATUS, C		; clear carry
	rrf	SR_CMD, F		; Shift right CMD & clr bit 7
	rrf	SR_POS1, F		; Shift into POS1
	rrf	SR_POS2, F		; Shift into POS2
	rrf	SR_POS3, F		; Shift into POS3
	rrf	SR_POS4, F		; Shift into POS4
	movlw	80h			; 
	btfsc	LAST_STATE, SSP_SI	; Is SSP_SI bit high?
	iorwf	SR_CMD, F		; ior a 1 into the into bit7
;
	incf	SR_BITCNT, F		; Increment bit count
	return

ssp_cs_went_high:
; End of a SSP frame
; Compare address in SR_CMD with DIP Switch
	rrf	SR_CMD, W		; move address into 4 LS bits
	movwf	TEMP_CMD
	rrf	TEMP_CMD, F
	rrf	TEMP_CMD, F
	rrf	TEMP_CMD, F
	movlw	SWITCH_MASK
	andwf	TEMP_CMD, F		; Mask out address bits
;
	movfw	SWITCH_PORT		; Read switch from port
	xorlw	SWITCH_MASK		; invert because switch 'on' reads 0
	andlw	SWITCH_MASK		; Remove all but switch bits
	subwf	TEMP_CMD, W		; Compare switch with CMD
	btfss	STATUS, Z
	goto	ssp_cs_went_high_done	; Address mismatch
;
	movfw	SR_CMD			; Get command byte
	andlw	CMD_STEP_MASK		; Mask out switch bits
	movwf	TIMESTEPS		; Save timesteps
;
	movfw	SR_POS1			; Move SSP register to PWM1 register
	movwf	SERVO1_POS
	movfw	SR_POS2			; Move SSP register to PWM2 register
	movwf	SERVO2_POS
	movfw	SR_POS3			; Move SSP register to PWM3 register
	movwf	SERVO3_POS
	movfw	SR_POS4			; Move SSP register to PWM4 register
	movwf	SERVO4_POS

ssp_cs_went_high_done:
	return

ssp_cs_went_low:
; start of a SSP frame
	clrf	SR_BITCNT		; Clear bit count
	clrf	SR_CMD			; Initialize shift register
	clrf	SR_POS1			; Initialize shift register
	clrf	SR_POS2			; Initialize shift register
	clrf	SR_POS3			; Initialize shift register
	clrf	SR_POS4			; Initialize shift register
	return
;
;
;
interrupt_service:
	movwf	TEMP_W			; Save W register
	swapf	STATUS, W		; Get STATUS register
	movwf	TEMP_STAT		; Save STATUS register
;
	btfss	INTCON, T0IF		; test for timer overflow intr
	goto	interrupt_exit		; not timer... should not happen

; Was timer interrupt
	call	PWM_controller		; run PWM state machine
	bcf	INTCON, T0IF		; clear timer overflow flag
;
interrupt_exit:
	bcf	STATUS, RP0
	swapf	TEMP_STAT, W		; Restore STATUS register
	movwf	STATUS
	swapf	TEMP_W, F
	swapf	TEMP_W, W		; Restore W register
	retfie


	END
