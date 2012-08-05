;**** **** **** **** ****
;
;Die Benutzung der Software ist mit folgenden Bedingungen verbunden:
;
;1. Da ich alles kostenlos zur Verf�gung stelle, gebe ich keinerlei Garantie
;   und �bernehme auch keinerlei Haftung f�r die Folgen der Benutzung.
;
;2. Die Software ist ausschlie�lich zur privaten Nutzung bestimmt. Ich
;   habe nicht gepr�ft, ob bei gewerblicher Nutzung irgendwelche Patentrechte
;   verletzt werden oder sonstige rechtliche Einschr�nkungen vorliegen.
;
;3. Jeder darf �nderungen vornehmen, z.B. um die Funktion seinen Bed�rfnissen
;   anzupassen oder zu erweitern. Ich w�rde mich freuen, wenn ich weiterhin als
;   Co-Autor in den Unterlagen erscheine und mir ein Link zur entprechenden Seite
;   (falls vorhanden) mitgeteilt wird.
;
;4. Auch nach den �nderungen sollen die Software weiterhin frei sein, d.h. kostenlos bleiben.
;
;!! Wer mit den Nutzungbedingungen nicht einverstanden ist, darf die Software nicht nutzen !!
;
; Dezember 2006
; autor: Bernhard Konze
; email: bernhard.konze@versanet.de
; minor changes: Oct 2007, Hans Haider, h.haider@comdesign.at - marked with #HH#
;
;**** **** **** **** ****
; Device
;**** **** **** **** ****
;**** **** **** **** ****
; z.Z. Nur f�r den Kaufregler BL17A-3P mit 16 MHz geeignet
;**** **** **** **** ****
;**** **** **** **** ****
.include "m8def.inc"
.include "bl-17a.inc"
;
; 8K Bytes of In-System Self-Programmable Flash
; 512 Bytes EEPROM
; 1K Byte Internal SRAM
;**** **** **** **** ****
;**** **** **** **** ****
; fuses must be set to internal calibrated oscillator = 8 mhz
;**** **** **** **** ****
;**** **** **** **** ****

.include "mnum.inc"

.equ	CHANGE_TIMEOUT	= 0x01
.equ	CHANGE_TOT_LOW	= 0x01

;.equ	POWER_RANGE	= 200			; full range of tcnt0 setting 10kHz
.equ	POWER_RANGE	= 125			; full range of tcnt0 setting 16kHz
;.equ	POWER_RANGE	= 100			; full range of tcnt0 setting 20kHz
;.equ	MIN_DUTY	= 15			; no power 10kHz
.equ	MIN_DUTY	= 14			; no power 16kHz
;.equ	MIN_DUTY	= 13			; no power 20kHz
.equ	NO_POWER	= 256-MIN_DUTY		; (POWER_OFF)
.equ	MAX_POWER	= 256-POWER_RANGE	; (FULL_POWER)

.equ	PWR_MAX_RPM1	= POWER_RANGE/4
.equ	PWR_MAX_RPM2	= POWER_RANGE/2

;.equ	PWR_STARTUP	= 40			; startup power 10kHz
;.equ	PWR_MAX_STARTUP	= PWR_STARTUP+20
.equ	PWR_STARTUP	= 20			; startup power 16kHz ; bko: 25
.equ	PWR_MAX_STARTUP	= PWR_STARTUP+13
;.equ	PWR_STARTUP	= 20			; startup power 20kHz
;.equ	PWR_MAX_STARTUP	= PWR_STARTUP+10


.equ	timeoutSTART	= 65000
.equ	timeoutMIN		= 48000

.equ	T1STOP	= 0x00
.equ	T1CK8	= 0x02

; timing(-l-h-x) holds the time of 4 commutations
; e.g.: timing = 0x010000 = 65536 * 0.5�s (timer1)
; 1 commutation = 8192�s
; 1/RPS (round per secone) = 6 commutations = 49152�s
; RPS = 20,34 ==> RPM = 1221
; PWR_RANGEx refers to timing_h if (timing_x==0)
;.equ	PWR_RANGE1	= 0x80	; ( ~2400 RPM )
;.equ	PWR_RANGE2	= 0x40	; ( ~4800 RPM )
.equ	PWR_RANGE1	= 0xc0	; ( ~1800 RPM )
.equ	PWR_RANGE2	= 0x80	; ( ~2400 RPM )

.equ	ENOUGH_GOODIES	= 60

;**** **** **** **** ****
; Register Definitions
.def	i_sreg		 = r1	; status register save in interrupts
.def	tcnt0_power_on	 = r2	; timer0 counts nFETs are switched on
.def	tcnt0_change_tot = r3	; when zero, tcnt0_power_on is changed by one (inc or dec)
.def	byte_cnt	 = r4
.def	uart_cnt	 = r5
.def	tcnt0_pwron_next = r6

.def	start_rcpuls_l	 = r7
.def	start_rcpuls_h	 = r8
.def	motor_count 	 = r9
;.def	motor_total = r10 ;
.def	control_timeout	 = r11
;.equ	CONTROL_TOT	 = 50	; time = NUMBER x 65ms
.equ	CONTROL_TOT	 = 4	; time = NUMBER x 65ms

.def	current_err	 = r12	; counts consecutive current errors
.equ	CURRENT_ERR_MAX  = 3	; performs a reset after MAX errors

.def	sys_control	 = r13
.def	t1_timeout	 = r14
.def	run_control	 = r15


.def	temp1	= r16			; main temporary
.def	temp2	= r17			; main temporary
.def	temp3	= r18			; main temporary
.def	temp4	= r19			; main temporary

.def	i_temp1	= r20			; interrupt temporary
.def	i_temp2	= r21			; interrupt temporary
.def	i_temp3	= r22			; interrupt temporary

.def	flags0	= r23	; state flags
	.equ	OCT1_PENDING	= 0	; if set, output compare interrunpt is pending
	.equ	UB_LOW 		= 1	; set if accu voltage low
	.equ	I_pFET_HIGH	= 2	; set if over-current detect
	.equ	GET_STATE	= 3	; set if state is to be send
	.equ	C_FET		= 4	; if set, C-FET state is to be changed
	.equ	A_FET		= 5	; if set, A-FET state is to be changed
	     ; if neither 1 nor 2 is set, B-FET state is to be changed
	.equ	I_OFF_CYCLE	= 6	; if set, current off cycle is active
	.equ	T1OVFL_FLAG	= 7	; each timer1 overflow sets this flag - used for voltage + current watch

.def	flags1	= r24	; state flags
	.equ	POWER_OFF	= 0	; switch fets on disabled
	.equ	FULL_POWER	= 1	; 100% on - don't switch off, but do OFF_CYCLE working
	.equ	CALC_NEXT_OCT1	= 2	; calculate OCT1 offset, when wait_OCT1_before_switch is called
	.equ	RC_PULS_UPDATED	= 3	; new rc-puls value available
	.equ	EVAL_SYS_STATE	= 4	; if set, overcurrent and undervoltage are checked
	.equ	EVAL_RPM	= 5	; if set, next PWM on should look for current
	.equ	EVAL_PWM	= 6	; if set, PWM should be updated

.def	flags2	= r25
	.equ	RPM_RANGE1	= 0	; if set RPM is lower than 1831 RPM
	.equ	RPM_RANGE2	= 1	; if set RPM is between 1831 RPM and 3662 RPM
	.equ	SCAN_TIMEOUT	= 2	; if set a startup timeout occurred
	.equ	POFF_CYCLE	= 3	; if set one commutation cycle is performed without power
	.equ	COMP_SAVE	= 4	; if set ACO was high
	.equ	STARTUP		= 5	; if set startup-phase is active
	.equ	RC_INTERVAL_OK	= 6	; 
	.equ	NO_SYNC		= 7	; 

; here the XYZ registers are placed ( r26-r31)

; ZH = new_duty		; PWM destination


;**** **** **** **** ****
; RAM Definitions
.dseg					;EEPROM segment
.org SRAM_START

tcnt1_sav_l:	.byte	1	; actual timer1 value
tcnt1_sav_h:	.byte	1
last_tcnt1_l:	.byte	1	; last timer1 value
last_tcnt1_h:	.byte	1
timing_l:	.byte	1	; holds time of 4 commutations 
timing_h:	.byte	1
timing_x:	.byte	1

timing_acc_l:	.byte	1	; holds the average time of 4 commutations 
timing_acc_h:	.byte	1
timing_acc_x:	.byte	1

rpm_l:		.byte	1	; holds the average time of 4 commutations 
rpm_h:		.byte	1
rpm_x:		.byte	1

wt_comp_scan_l:	.byte	1	; time from switch to comparator scan
wt_comp_scan_h:	.byte	1       
com_timing_l:	.byte	1	; time from zero-crossing to switch of the appropriate FET
com_timing_h:	.byte	1
wt_OCT1_tot_l:	.byte	1	; OCT1 waiting time
wt_OCT1_tot_h:	.byte	1
zero_wt_l:	.byte	1
zero_wt_h:	.byte	1
last_com_l:	.byte	1
last_com_h:	.byte	1

stop_rcpuls_l:	.byte	1
stop_rcpuls_h:	.byte	1
new_rcpuls_l:	.byte	1
new_rcpuls_h:	.byte	1

duty_offset:	.byte	1
goodies:	.byte	1
comp_state:	.byte	1
uart_command:	.byte	1

uart_data:	.byte	100		; only for debug requirements


;**** **** **** **** ****
; ATmega8 interrupts

;.equ	INT0addr=$001	; External Interrupt0 Vector Address
;.equ	INT1addr=$002	; External Interrupt1 Vector Address
;.equ	OC2addr =$003	; Output Compare2 Interrupt Vector Address
;.equ	OVF2addr=$004	; Overflow2 Interrupt Vector Address
;.equ	ICP1addr=$005	; Input Capture1 Interrupt Vector Address
;.equ	OC1Aaddr=$006	; Output Compare1A Interrupt Vector Address
;.equ	OC1Baddr=$007	; Output Compare1B Interrupt Vector Address
;.equ	OVF1addr=$008	; Overflow1 Interrupt Vector Address
;.equ	OVF0addr=$009	; Overflow0 Interrupt Vector Address
;.equ	SPIaddr =$00a	; SPI Interrupt Vector Address
;.equ	URXCaddr=$00b	; USART Receive Complete Interrupt Vector Address
;.equ	UDREaddr=$00c	; USART Data Register Empty Interrupt Vector Address
;.equ	UTXCaddr=$00d	; USART Transmit Complete Interrupt Vector Address
;.equ	ADCCaddr=$00e	; ADC Interrupt Vector Address
;.equ	ERDYaddr=$00f	; EEPROM Interrupt Vector Address
;.equ	ACIaddr =$010	; Analog Comparator Interrupt Vector Address
;.equ	TWIaddr =$011	; Irq. vector address for Two-Wire Interface
;.equ	SPMaddr =$012	; SPM complete Interrupt Vector Address
;.equ	SPMRaddr =$012	; SPM complete Interrupt Vector Address
;-----bko-----------------------------------------------------------------

;**** **** **** **** ****
.cseg
.org 0
;**** **** **** **** ****

;-----bko-----------------------------------------------------------------
; reset and interrupt jump table
		rjmp	reset
		nop	; ext_int0
		nop	; ext_int1
		nop	; t2oc_int
		nop	; t2ovfl_int
		nop	; icp1
		rjmp	t1oca_int
		nop	; t1ocb_int
		rjmp	t1ovfl_int
		rjmp	t0ovfl_int
		nop	; spi_int
		rjmp	urxc
		nop	; udre
		nop ; utxc ; #HH# not used - only receiver in action!
; not used	nop	; adc_int
; not used	nop	; eep_int
; not used	nop	; aci_int
; not used	nop	; wire2_int
; not used	nop	; spmc_int


version:	.db	0x0d, 0x0a
		.db	"bk",Typ,"410r06p40-uart06"
		.db	0x0d, 0x0a

;-----bko-----------------------------------------------------------------
; init after reset

reset:		ldi	temp1, high(RAMEND)	; stack = RAMEND
		out	SPH, temp1
		ldi	temp1, low(RAMEND)
		out 	SPL, temp1

	; runs with 16MHz crystal - no calibration needed

	; portB
		ldi	temp1, INIT_PB
		out	PORTB, temp1
		ldi	temp1, DIR_PB
		out	DDRB, temp1

	; portC
		ldi	temp1, INIT_PC
		out	PORTC, temp1
		ldi	temp1, DIR_PC
		out	DDRC, temp1

	; portD
		ldi	temp1, INIT_PD
		out	PORTD, temp1
		ldi	temp1, DIR_PD
		out	DDRD, temp1
		
		HHDEBUGLED_on

	; timer0: PWM + beep control = 0x02 	; start timer0 with CK/8 (0.5�s/count)
		ldi	temp1, 0x02
		out	TCCR0, temp1

	; timer1: commutation control = 0x02	; start timer1 with CK/8 (0.5�s/count)
		ldi	temp1, T1CK8
		out	TCCR1B, temp1

	; reset state flags
		clr	flags0
		clr	flags1
		clr	flags2

		ldi	temp1, MOTOR_NUMBER
		mov	motor_count, temp1
		sbr	flags2, (1<<NO_SYNC)
		
	; clear RAM
		clr	XH
		ldi	XL, low (SRAM_START)
		clr	temp1
clear_ram:	st	X+, temp1
		cpi	XL, uart_data+1
		brlo	clear_ram

	; power off
		rcall	switch_power_off

	; reset input timeout
		ldi	temp1, CONTROL_TOT
		mov	control_timeout, temp1
		
; 	rjmp	control_start		; for simulator only

		ldi	ZH,high(version*2)
		ldi	ZL,low(version*2)

;**** UART Initialization ****
		ldi	temp1, 0
		out	UBRRH, temp1

		ldi	temp1, 16 ; 115200
		out	UBRRL, temp1
		ldi	temp1, 0x02	; #BK# set U2X
		out	UCSRA, temp1

;		ldi	temp1, 34 ; 57600
;		out	UBRRL, temp1
;		ldi	temp1, 0x02	; #BK# set U2X
;		out	UCSRA, temp1

		ldi	temp1, 0x10 ; #HH# enable rx only!
		out	UCSRB, temp1	; 
		in 	temp1, UDR	; clear possibly pending rxc
		ldi	temp1, 0
		mov	uart_cnt, temp1

;   #HH# sending of version-string is commented out
;		lpm
;		adiw	ZL,1		;increment Z-pointer
;		mov	temp1, r0	; (1)
;		rcall	send_byte

		rcall	wait260ms	; wait a while
		rcall	wait260ms

		rcall	beep_f1
		rcall	wait30ms
		rcall	beep_f2
		rcall	wait30ms
		rcall	beep_f3
		rcall	wait30ms
		rcall	wait30ms
		rcall	wait30ms
		rcall	wait30ms

;   #HH# sending of version-string is commented out
;		ldi	temp2, reset-version-1
;v_str_rest:	lpm
;		adiw	ZL,1		;increment Z-pointer
;		mov	temp1, r0	; (6)
;		rcall	send_byte
;		dec	temp2
;		brne	v_str_rest

		in	temp1, UDR  ; clear serial input - if existing
		in	temp1, UDR
		sbi	UCSRA, RXC		; clear flag
		sbi	UCSRB, RXCIE		; enable reception irq

		rcall	beep_f4			; signal: rcpuls ready
		rcall	beep_f4
		rcall	beep_f4


control_start:	; init variables
		ldi	temp1, CHANGE_TIMEOUT
		mov	tcnt0_change_tot, temp1
		ldi	temp1, NO_POWER
		mov	tcnt0_power_on, temp1

		ldi	temp1, 0		; reset error counters
		mov	current_err,temp1
		mov	sys_control, temp1

	; init registers and interrupts
		ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0)
		out	TIFR, temp1		; clear TOIE1,OCIE1A & TOIE0
		out	TIMSK, temp1		; enable TOIE1,OCIE1A & TOIE0 interrupts

		sei				; enable all interrupts

		ldi	temp1, 30
		sts	duty_offset, temp1

		rcall	set_all_timings

		rcall	wait30ms

		rjmp	init_startup
		
;-----bko-----------------------------------------------------------------
; output compare timer1 interrupt
t1oca_int:	in	i_sreg, SREG
		cbr	flags0, (1<<OCT1_PENDING) ; signal OCT1 passed
		out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; overflow timer1 / happens all 32768�s
t1ovfl_int:	in	i_sreg, SREG
		sbr	flags0, (1<<T1OVFL_FLAG)

		tst	t1_timeout
		breq	t1ovfl_10
		dec	t1_timeout

t1ovfl_10:	tst	control_timeout
		brne	t1ovfl_20
		clr	ZH
		rjmp	t1ovfl_99
t1ovfl_20:	dec	control_timeout


t1ovfl_99:	out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; timer0 overflow interrupt
t0ovfl_int:	in	i_sreg, SREG
		sbrc	flags0, I_OFF_CYCLE
		rjmp	t0_on_cycle

t0_off_cycle:	sbr	flags2, (1<<COMP_SAVE)
		sbic	ACSR, ACO		; mirror inverted ACO to bit-var
		cbr	flags2, (1<<COMP_SAVE)

		; fastest possible ; #HH# ; DANGEROUS!!!
		mov i_temp1, tcnt0_pwron_next
		rjmp set_next_pwm

	; changes in PWM ?
		mov	i_temp1, tcnt0_power_on
		mov	i_temp2, tcnt0_pwron_next
		cp	i_temp2, i_temp1
		brsh	lower_pwm		; next power-on-time is lower or same
higher_pwm:	dec	tcnt0_change_tot	; change-timeout passed ?
		brne	nFET_off		; .. no
		ldi	i_temp2, CHANGE_TIMEOUT	; .. yes - change-timeout for more power
		mov	tcnt0_change_tot, i_temp2 ; reset change-timeout and decrement
		dec	i_temp1			; <dec> increases power-on-time
	
		rjmp	set_next_pwm

lower_pwm:	breq	nFET_off		; pwm is unchanged
		dec	tcnt0_change_tot	; change-timeout passed ?
		brne	nFET_off		; .. no
		ldi	i_temp2, CHANGE_TOT_LOW ; .. yes - change-timeout for lower power
		mov	tcnt0_change_tot, i_temp2 ; reset change-timeout and increment
		inc	i_temp1			; <inc> decreases power-on-time
		
set_next_pwm:	mov	tcnt0_power_on, i_temp1

nFET_off:	sbr	flags0, (1<<I_OFF_CYCLE) ; PWM state = off cycle

	; switch appropriate nFET off
		sbrs	flags0, C_FET
		rjmp	test_AnFET

; C_FET is active
		sbrs	flags1, FULL_POWER
		CnFET_off		; Cn off
		rjmp	reload_t0_off_cycle

test_AnFET:	sbrs	flags0, A_FET
		rjmp	switch_BnFET

; A_FET is active
switch_AnFET:	sbrs	flags1, FULL_POWER
		AnFET_off		; An off
		rjmp	reload_t0_off_cycle

; B_FET is active
switch_BnFET:	sbrs	flags1, FULL_POWER
		BnFET_off		; Bn off

	; reload timer0 with the appropriate value
reload_t0_off_cycle:
		mov	i_temp1, tcnt0_power_on
		subi	i_temp1, -POWER_RANGE	; adi i_temp1, POWER_RANGE
		com	i_temp1			; timer0 increments
		out	TCNT0, i_temp1

		rjmp	t0_int_exit

; reload timer90 + switch appropriate nFET on
t0_on_cycle:	mov	i_temp1, tcnt0_power_on
		out	TCNT0, i_temp1		; reload t0
		cbr	flags0, (1<<I_OFF_CYCLE) ; PWM state = on cycle (no off cycle)

; switch appropriate nFET on
nFET_on:	sbrs	flags0, C_FET		; is Cn choppered ?
		rjmp	test_AnFET_on			; .. no - test An
		sbrs	flags1, POWER_OFF
		CnFET_on		; Cn on
		rjmp	eval_power_state
test_AnFET_on:	sbrs	flags0, A_FET		; is An choppered ?
		rjmp	sw_BnFET_on			; .. no - Bn has to be choppered
		sbrs	flags1, POWER_OFF
		AnFET_on		; An on
		rjmp	eval_power_state
sw_BnFET_on:	sbrs	flags1, POWER_OFF
		BnFET_on		; Bn on

	; evaluate power state
eval_power_state:
		cpi	i_temp1, MAX_POWER+1
		brsh	not_full_power
	; FULL POWER
		sbr	flags1, (1<<FULL_POWER)	; tcnt0_power_on = MAX_POWER means FULL_POWER
		cbr	flags1, (1<<POWER_OFF)
		rjmp	t0_int_exit
not_full_power:	cpi	i_temp1, NO_POWER
		brlo	neither_full_nor_off
	; POWER OFF
		cbr	flags1, (1<<FULL_POWER)	; tcnt0_power_on = NO_POWER means power off
		sbr	flags1, (1<<POWER_OFF)
		rjmp	t0_int_exit
neither_full_nor_off:
		cbr	flags1, (1<<FULL_POWER)	; tcnt0_power_on = MAX_POWER means FULL_POWER
		cbr	flags1, (1<<POWER_OFF)

t0_int_exit:	sbrc	flags2, POFF_CYCLE
		sbr	flags1, (1<<POWER_OFF)
		out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
urxc:		in	i_sreg, SREG
		in	i_temp1, UDR

		cpi	i_temp1, 0xf5
		breq	x3d_sync
		brcc	unknown_cmd

		sbrc	flags2, NO_SYNC
		rjmp	urxc_exit		; trough away
		mov i_temp2, motor_count
		breq urxc_exit
		dec	motor_count
		brne	urxc_exit

; 10kHz do nothing range is 0-200
;		mov	ZH, i_temp1

; 16kHz range is 0-125 - divide by 2 and add a quarter
		lsr	i_temp1				; 0-200 ==> 0-100
		mov	i_temp2, i_temp1
		lsr	i_temp2
		lsr	i_temp2
		add	i_temp1, i_temp2
		mov	ZH, i_temp1

; 20kHz range is 0-100 - divide by 2
;		lsr	i_temp1				; 0-200 ==> 0-100
;		mov	ZH, i_temp1

		sbr	flags2, (1<<NO_SYNC)
		ldi	i_temp1, CONTROL_TOT
		mov	control_timeout, i_temp1
		rjmp	urxc_exit

unknown_cmd:	sts	uart_command, i_temp1
		rjmp	urxc_exit

x3d_sync:	cbr	flags2, (1<<NO_SYNC)
		ldi	i_temp2, MOTOR_NUMBER
		mov	motor_count, i_temp2

urxc_exit:	out	SREG, i_sreg
		reti

utxc:		in	i_sreg, SREG
		ld	i_temp1,X+
		out	UDR, i_temp1
		dec	uart_cnt
		brne	utxc_90
		cbi	UCSRB, TXCIE		; disable irq
utxc_90:	out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; beeper: timer0 is set to 1�s/count
beep_f1:	ldi	temp4, 200
		ldi	temp2, 80
		rjmp	beep

beep_f2:	ldi	temp4, 180
		ldi	temp2, 100
		rjmp	beep

beep_f3:	ldi	temp4, 160
		ldi	temp2, 120
		rjmp	beep

beep_f4:	ldi	temp4, 100
		ldi	temp2, 200
		rjmp	beep

beep:		clr	temp1
		out	TCNT0, temp1
		BpFET_on		; BpFET on
		AnFET_on		; CnFET on
beep_BpCn10:	in	temp1, TCNT0
		cpi	temp1, 64		; 32�s on
		brne	beep_BpCn10
		BpFET_off		; BpFET off
		AnFET_off		; CnFET off
		ldi	temp3, 16		; 2040�s off
beep_BpCn12:	clr	temp1
		out	TCNT0, temp1
beep_BpCn13:	in	temp1, TCNT0
		cp	temp1, temp4
		brne	beep_BpCn13
		dec	temp3
		brne	beep_BpCn12
		dec	temp2
		brne	beep
		ret

wait30ms:	ldi	temp2, 30
beep_BpCn20:	ldi	temp3, 16
beep_BpCn21:	clr	temp1
		out	TCNT0, temp1
beep_BpCn22:	in	temp1, TCNT0
		cpi	temp1, 255
		brne	beep_BpCn22
		dec	temp3
		brne	beep_BpCn21
		dec	temp2
		brne	beep_BpCn20
		ret

	; 256 periods = 261ms silence
wait260ms:	ldi	temp2, 0	; = 256
beep2_BpCn20:	ldi	temp3, 16
beep2_BpCn21:	clr	temp1
		out	TCNT0, temp1
beep2_BpCn22:	in	temp1, TCNT0
		cpi	temp1, 255
		brne	beep2_BpCn22
		dec	temp3
		brne	beep2_BpCn21
		dec	temp2
		brne	beep2_BpCn20
		ret
;-----bko-----------------------------------------------------------------
tcnt1_to_temp:	ldi	temp4, T1STOP		; stop timer1
		out	TCCR1B, temp4
		ldi	temp4, T1CK8		; preload temp with restart timer1
		in	temp1, TCNT1L		;  - the preload cycle is needed to complete stop operation
		in	temp2, TCNT1H
		out	TCCR1B, temp4
		ret				; !!! ext0int stays disabled - must be enabled again by caller
	; there seems to be only one TEMP register in the AVR
	; if the ext0int interrupt falls between readad LOW value while HIGH value is captured in TEMP and
	; read HIGH value, TEMP register is changed in ext0int routine
;-----bko-----------------------------------------------------------------
evaluate_sys_state:
		cbr	flags1, (1<<EVAL_SYS_STATE)
		sbrs	flags0, T1OVFL_FLAG
		rjmp	eval_sys_s99

	; do it not more often as every 32�s
		cbr	flags0, (1<<T1OVFL_FLAG)

		rjmp	eval_sys_s99		; disabled ;-)

	; control current
eval_sys_i:	rjmp	eval_sys_i_ok

		mov	temp1, current_err
		cpi	temp1, CURRENT_ERR_MAX
		brcc	panic_exit
		inc	current_err
		rjmp	eval_sys_ub

eval_sys_i_ok:	tst	current_err
		breq	eval_sys_ub
		dec	current_err

	; control voltage
eval_sys_ub:	rjmp	eval_sys_ub_ok

		mov	temp1, sys_control
		cpi	temp1, POWER_RANGE
		brcc	eval_sys_s99
		inc	sys_control
		rjmp	eval_sys_s99

eval_sys_ub_ok:	tst	sys_control
		breq	eval_sys_s99
		dec	sys_control
		
eval_sys_s99:	ret

panic_exit:	; !!!!!! OVERCURRENT !!!!!!!!
		cli
		rjmp	reset
;-----bko-----------------------------------------------------------------
set_new_duty:	mov	temp1, ZH
		sub	temp1, sys_control
		brcc	set_new_duty10
		ldi	temp1, MIN_DUTY-1
	; evaluate RPM range
set_new_duty10:	lds	temp2, timing_x
		tst	temp2
		brne	set_new_duty12
		lds	temp2, timing_h	; get actual RPM reference high
		cpi	temp2, PWR_RANGE1	; lower range1 ?
		brcs	set_new_duty20		; on carry - test next range
	; lower as range1
set_new_duty12:	sbr	flags2, (1<<RPM_RANGE1)
		sbr	flags2, (1<<RPM_RANGE2)
		ldi	temp2, PWR_MAX_RPM1	; higher than range1 power max ?
		cp	temp1, temp2
		brcs	set_new_duty40		; on carry - not higher, no restriction
		mov	temp1, temp2		; low (range1) RPM - set PWR_MAX_RPM1
		rjmp	set_new_duty40
	; higher as range1
set_new_duty20:	cpi	temp2, PWR_RANGE2	; lower range2 ?
		brcs	set_new_duty30		; on carry - not lower, no restriction
set_new_duty22:	cbr	flags2, (1<<RPM_RANGE1)
		sbr	flags2, (1<<RPM_RANGE2)
		ldi	temp2, PWR_MAX_RPM2	; higher than range2 power max ?
		cp	temp1, temp2
		brcs	set_new_duty40		; on carry - not higher, no restriction
		mov	temp1, temp2		; low (range2) RPM - set PWR_MAX_RPM2
		rjmp	set_new_duty40
	; higher as range2
set_new_duty30:	cbr	flags2, (1<<RPM_RANGE1)+(1<<RPM_RANGE2)
	; range limits are evaluated - look for STARTUP conditions
set_new_duty40:	sbrs	flags2, STARTUP
		rjmp	set_new_duty50
		ldi	temp3, PWR_STARTUP	; at least PWR_STARTUP ?
		cp	temp1, temp3
		brcc	set_new_duty42		; on no carry - higher than PWR_STARTUP, test PWR_MAX_STARTUP
		ldi	temp1, PWR_STARTUP	; lower - set to PWR_STARTUP
		rjmp	set_new_duty50
set_new_duty42:	ldi	temp3, PWR_MAX_STARTUP	; limit power in startup phase
		cp	temp1, temp3
		brcs	set_new_duty50		; on carry - not higher, test range 2
		mov	temp1, temp3		; set PWR_MAX_STARTUP limit

set_new_duty50:	com	temp1			; down-count to up-count (T0)
		mov	tcnt0_pwron_next, temp1	; save in next
	; tcnt0_power_on is updated to tcnt0_pwron_next in acceptable steps
		ret
;-----bko-----------------------------------------------------------------
evaluate_rpm:	cbr	flags1, (1<<EVAL_RPM)
		lds	temp3, rpm_x
		lds	temp2, rpm_h

		lds	temp1, rpm_l	; subtract 1/256
		sub	temp1, temp2
		sts	rpm_l, temp1
		lds	temp1, rpm_h
		sbc	temp1, temp3
		sts	rpm_h, temp1
		lds	temp1, rpm_x
		sbci	temp1, 0
		sts	rpm_x, temp1

		lds	temp3, timing_acc_x
		lds	temp2, timing_acc_h
		lds	temp1, timing_acc_l
		lsr	temp3		; make one complete commutation cycle
		ror	temp2
		ror	temp1
		lsr	temp3
		ror	temp2
		ror	temp1
	; temp3 is zero now - for sure !!
		sts	timing_acc_x, temp3
		sts	timing_acc_h, temp3
		sts	timing_acc_l, temp3
	; and add the result as 1/256
		lds	temp3, rpm_l
		add	temp3, temp1
		sts	rpm_l, temp3
		lds	temp3, rpm_h
		adc	temp3, temp2
		sts	rpm_h, temp3
		ldi	temp1, 0
		lds	temp3, rpm_x
		adc	temp3, temp1
		sts	rpm_x, temp3

		ret
;-----bko-----------------------------------------------------------------
set_all_timings:
		ldi	YL, low  (timeoutSTART)
		ldi	YH, high (timeoutSTART)
		sts	wt_OCT1_tot_l, YL
		sts	wt_OCT1_tot_h, YH
		ldi	temp3, 0xff
		ldi	temp4, 0x1f
		sts	wt_comp_scan_l, temp3
		sts	wt_comp_scan_h, temp4
		sts	com_timing_l, temp3
		sts	com_timing_h, temp4

set_timing_v:	ldi	ZL, 0x03
		sts	timing_x, ZL
		ldi	temp4, 0xff
		sts	timing_h, temp4
		ldi	temp3, 0xff
		sts	timing_l, temp3

		ret
;-----bko-----------------------------------------------------------------
update_timing:	rcall	tcnt1_to_temp
		sts	tcnt1_sav_l, temp1
		sts	tcnt1_sav_h, temp2
		add	temp1, YL
		adc	temp2, YH
		ldi	temp4, (1<<TOIE1)+(1<<TOIE0)
		out	TIMSK, temp4
		out	OCR1AH, temp2
		out	OCR1AL, temp1
		sbr	flags0, (1<<OCT1_PENDING)
		ldi	temp4, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0) ; enable interrupt again
		out	TIMSK, temp4

	; calculate next waiting times - timing(-l-h-x) holds the time of 4 commutations
		lds	temp1, timing_l
		lds	temp2, timing_h
		lds	ZL, timing_x

		sts	zero_wt_l, temp1	; save for zero crossing timeout
		sts	zero_wt_h, temp2
		tst	ZL
		breq	update_t00
		ldi	temp4, 0xff
		sts	zero_wt_l, temp4	; save for zero crossing timeout
		sts	zero_wt_h, temp4
update_t00:
		lsr	ZL			; build a quarter
		ror	temp2
		ror	temp1

		lsr	ZL
		ror	temp2
		ror	temp1
		lds	temp3, timing_l		; .. and subtract from timing
		lds	temp4, timing_h
		lds	ZL, timing_x
		sub	temp3, temp1
		sbc	temp4, temp2
		sbci	ZL, 0

		lds	temp1, tcnt1_sav_l	; calculate this commutation time
		lds	temp2, tcnt1_sav_h
		lds	YL, last_tcnt1_l
		lds	YH, last_tcnt1_h
		sts	last_tcnt1_l, temp1
		sts	last_tcnt1_h, temp2
		sub	temp1, YL
		sbc	temp2, YH
		sts	last_com_l, temp1
		sts	last_com_h, temp2

		add	temp3, temp1		; .. and add to timing
		adc	temp4, temp2
		ldi	temp2, 0
		adc	ZL, temp2

	; limit RPM to 120.000
		tst	ZL
		brne	update_t90
		tst	temp4
		breq	update_t10
		cpi	temp4, 0x02
		brne	update_t90
		cpi	temp3, 0x98		; 0x298 = 120.000 RPM
		brcc	update_t90
	; set RPM to 120.000
update_t10:	ldi	temp4, 0x02
		ldi	temp3, 0x98
		tst	run_control 
		brne	update_t90		; just active
		ldi	temp1, 0xff		; not active - reactivate
		mov	run_control, temp1

update_t90:	sts	timing_l, temp3
		sts	timing_h, temp4
		sts	timing_x, ZL
		cpi	ZL, 4		; limit range to 0x3ffff
		brcs	update_t99
		rcall	set_timing_v

update_t99:	lds	temp1, timing_acc_l
		add	temp1, temp3
		sts	timing_acc_l, temp1
		lds	temp1, timing_acc_h
		adc	temp1, temp4
		sts	timing_acc_h, temp1
		lds	temp1, timing_acc_x
		adc	temp1, ZL
		sts	timing_acc_x, temp1

		lsr	ZL			; a 16th is the next wait before scan
		ror	temp4
		ror	temp3
		lsr	ZL
		ror	temp4
		ror	temp3
		lsr	ZL
		ror	temp4
		ror	temp3
		lsr	ZL
		ror	temp4
		ror	temp3
		sts	wt_comp_scan_l, temp3
		sts	wt_comp_scan_h, temp4

	; use the same value for commutation timing (15�)
		sts	com_timing_l, temp3
		sts	com_timing_h, temp4

		ret
;-----bko-----------------------------------------------------------------
calc_next_timing:
		lds	YL, wt_comp_scan_l	; holds wait-before-scan value
		lds	YH, wt_comp_scan_h
		rcall	update_timing

		ret

wait_OCT1_tot:	sbrc	flags0, OCT1_PENDING
		rjmp	wait_OCT1_tot

set_OCT1_tot:	cbi	ADCSRA, ADEN		; switch to comparator multiplexed
		in	temp1, SFIOR
		ori	temp1, (1<<ACME)
		out	SFIOR, temp1

		lds	YH, zero_wt_h
		lds	YL, zero_wt_l
		rcall	tcnt1_to_temp
		add	temp1, YL
		adc	temp2, YH
		ldi	temp4, (1<<TOIE1)+(1<<TOIE0)
		out	TIMSK, temp4
		out	OCR1AH, temp2
		out	OCR1AL, temp1
		sbr	flags0, (1<<OCT1_PENDING)
		ldi	temp4, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0)
		out	TIMSK, temp4

		ret
;-----bko-----------------------------------------------------------------
wait_OCT1_before_switch:
		rcall	tcnt1_to_temp
		lds	YL, com_timing_l
		lds	YH, com_timing_h
		add	temp1, YL
		adc	temp2, YH
		ldi	temp3, (1<<TOIE1)+(1<<TOIE0)
		out	TIMSK, temp3
		out	OCR1AH, temp2
		out	OCR1AL, temp1
		sbr	flags0, (1<<OCT1_PENDING)
		ldi	temp3, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0)
		out	TIMSK, temp3

	; don't waste time while waiting - do some controls, if indicated

		sbrc	flags1, EVAL_SYS_STATE
		rcall	evaluate_sys_state

		sbrc	flags1, EVAL_PWM
		rcall	set_new_duty

		sbrc	flags1, EVAL_RPM
		rcall	evaluate_rpm

OCT1_wait:	sbrc	flags0, OCT1_PENDING
		rjmp	OCT1_wait
		ret
;-----bko-----------------------------------------------------------------
start_timeout:	lds	YL, wt_OCT1_tot_l
		lds	YH, wt_OCT1_tot_h
		rcall	update_timing

		in	temp1, TCNT1L
		andi	temp1, 0x0f
		sub	YH, temp1
		cpi	YH, high (timeoutMIN)
		brcc	set_tot2
		ldi	YH, high (timeoutSTART)		
set_tot2:
		sts	wt_OCT1_tot_h, YH

		rcall	sync_with_poweron	; wait at least 100+ microseconds
		rcall	sync_with_poweron	; for demagnetisation - one sync may be added

		ret
;-----bko-----------------------------------------------------------------
send_byte:	sbi	UCSRA,TXC	; clear flag
		out	UDR,temp1
send_b20:	sbis	UCSRA,TXC
		rjmp	send_b20
		ret
;-----bko-----------------------------------------------------------------
switch_power_off:
		ldi	ZH, MIN_DUTY-1		; ZH is new_duty
		ldi	temp1, NO_POWER		; lowest tcnt0_power_on value
		mov	tcnt0_power_on, temp1
		mov	tcnt0_pwron_next, temp1
		ldi	temp1, INIT_PC		; all off
		out	PORTC, temp1
		ldi	temp1, INIT_PD
		out	PORTD, temp1
		ldi	temp1, CHANGE_TIMEOUT	; reset change-timeout
		mov	tcnt0_change_tot, temp1
		sbr	flags1, (1<<POWER_OFF)	; disable power on
		cbr	flags2, (1<<POFF_CYCLE)
		sbr	flags2, (1<<STARTUP)
		ret				; motor is off
;-----bko-----------------------------------------------------------------
wait_if_spike:	ldi	temp1, 8
wait_if_spike2:	dec	temp1
		brne	wait_if_spike2
		ret
;-----bko-----------------------------------------------------------------
sync_with_poweron:
		sbrc	flags0, I_OFF_CYCLE	; first wait for power on
		rjmp	sync_with_poweron
wait_for_poweroff:
		sbrs	flags0, I_OFF_CYCLE	; now wait for power off
		rjmp	wait_for_poweroff
		ret
;-----bko-----------------------------------------------------------------
motor_brake:
.if MOT_BRAKE == 1
		ldi	temp2, 40		; 40 * 0.065ms = 2.6 sec
		ldi	temp1, BRAKE_PB		; all N-FETs on
		out	PORTB, temp1
mot_brk10:	sbrs	flags0, T1OVFL_FLAG
		rjmp	mot_brk10
		cbr	flags0, (1<<T1OVFL_FLAG)
		push	temp2
		rcall	evaluate_rc_puls
		pop	temp2
		cpi	ZH, MIN_DUTY+3		; avoid jitter detect
		brcs	mot_brk20
		rjmp	mot_brk90
mot_brk20:
		dec	temp2
		brne	mot_brk10
mot_brk90:
		ldi	temp1, INIT_PC		; all off
		out	PORTC, temp1
		ldi	temp1, INIT_PD
		out	PORTD, temp1
.endif	; MOT_BRAKE == 1
		ret
;-----bko-----------------------------------------------------------------
; **** startup loop ****
init_startup:	rcall	switch_power_off
		rcall	motor_brake
wait_for_power_on:

    HHDEBUGLED_off

.if	DEBUG_LEDS == 1
		in	temp1, PORTB
		sbr	temp1, (1<<GREEN_LED)+(1<<RED1_LED)
		ldi	temp2, (1<<RED2_LED)
		eor	temp1, temp2		; toggle while waiting for power >= MIN_DUTY
		out	PORTB, temp1
.endif
		cpi	ZH, MIN_DUTY
		brcs	wait_for_power_on

		ldi	temp1, PWR_STARTUP	; begin startup with low power
		com	temp1
		mov	tcnt0_pwron_next, temp1

		cbi	ADCSRA, ADEN		; switch to comparator multiplexed
		in	temp1, SFIOR
		ori	temp1, (1<<ACME)
		out	SFIOR, temp1

		ldi	temp1, INIT_PC		; all off
		out	PORTC, temp1
		ldi	temp1, INIT_PD
		out	PORTD, temp1
		ldi	temp1, 27		; wait about 5mikosec
FETs_off_wt:	dec	temp1
		brne	FETs_off_wt

		rcall	com5com6
		rcall	com6com1

		cbr	flags2, (1<<SCAN_TIMEOUT)
		ldi	temp1, 0
		sts	goodies, temp1

		ldi	temp1, 80	; x 32msec
		mov	t1_timeout, temp1

		rcall	set_all_timings

		rcall	start_timeout

	; fall through start1

;-----bko-----------------------------------------------------------------
; **** start control loop ****

; state 1 = B(p-on) + C(n-choppered) - comparator A evaluated
; out_cA changes from low to high
start1:
.if	DEBUG_LEDS == 1
		in	temp1, PORTB
		sbr	temp1, (1<<RED1_LED)				; red1 = off
		cbr	temp1, (1<<GREEN_LED)+(1<<RED2_LED)		; green+red2 = on
		out	PORTB, temp1
.endif
		sbrs	flags2, COMP_SAVE	; high ?
		rjmp	start1_2		; .. no - loop, while high

start1_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start1_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start1_9
start1_1:	rcall	sync_with_poweron

		sbrc	flags2, COMP_SAVE	; high ?
		rjmp	start1_0		; .. no - loop, while high

; do the special 120� switch
		ldi	temp1, 0
		sts	goodies, temp1
		rcall	com1com2
		rcall	com2com3
		rcall	com3com4
		rcall	start_timeout
		rjmp	start4
	
start1_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start1_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start1_9
start1_3:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE	; high ?
		rjmp	start1_2		; .. no - loop, while low

start1_9:
		rcall	com1com2
		rcall	start_timeout

; state 2 = A(p-on) + C(n-choppered) - comparator B evaluated
; out_cB changes from high to low

start2:		sbrc	flags2, COMP_SAVE
		rjmp	start2_2

start2_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start2_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start2_9
start2_1:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE
		rjmp	start2_0
		rjmp	start2_9

start2_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start2_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start2_9
start2_3:	rcall	sync_with_poweron
		sbrc	flags2, COMP_SAVE
		rjmp	start2_2

start2_9:
		rcall	com2com3
		rcall	start_timeout

; state 3 = A(p-on) + B(n-choppered) - comparator C evaluated
; out_cC changes from low to high

start3:		sbrs	flags2, COMP_SAVE
		rjmp	start3_2

start3_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start3_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start3_9
start3_1:	rcall	sync_with_poweron
		sbrc	flags2, COMP_SAVE
		rjmp	start3_0
		rjmp	start3_9

start3_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start3_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start3_9
start3_3:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE
		rjmp	start3_2

start3_9:
		rcall	com3com4
		rcall	set_new_duty
		rcall	start_timeout

; state 4 = C(p-on) + B(n-choppered) - comparator A evaluated
; out_cA changes from high to low

start4:
.if	DEBUG_LEDS == 1
		in	temp1, PORTB
		sbr	temp1, (1<<RED2_LED)				; red2 = off
		cbr	temp1, (1<<GREEN_LED)+(1<<RED1_LED)		; green+red1 = on
		out	PORTB, temp1
.endif
		sbrc	flags2, COMP_SAVE
		rjmp	start4_2

start4_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start4_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start4_9
start4_1:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE
		rjmp	start4_0
		rjmp	start4_9

start4_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start4_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start4_9
start4_3:	rcall	sync_with_poweron
		sbrc	flags2, COMP_SAVE
		rjmp	start4_2

start4_9:
		rcall	com4com5
		rcall	start_timeout


; state 5 = C(p-on) + A(n-choppered) - comparator B evaluated
; out_cB changes from low to high


start5:		sbrs	flags2, COMP_SAVE
		rjmp	start5_2

start5_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start5_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start5_9
start5_1:	rcall	sync_with_poweron
		sbrc	flags2, COMP_SAVE
		rjmp	start5_0
		rjmp	start5_9

start5_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start5_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start5_9
start5_3:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE
		rjmp	start5_2

start5_9:
		rcall	com5com6
		rcall	evaluate_sys_state
		rcall	start_timeout

; state 6 = B(p-on) + A(n-choppered) - comparator C evaluated
; out_cC changes from high to low

start6:
.if	DEBUG_LEDS == 1
		in	temp1, PORTB
		cbr	temp1, (1<<GREEN_LED)+(1<<RED1_LED)+(1<<RED2_LED)	; all = on
		out	PORTB, temp1
.endif
		sbrc	flags2, COMP_SAVE
		rjmp	start6_2

start6_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start6_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start6_9
start6_1:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE
		rjmp	start6_0
		rjmp	start6_9

start6_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start6_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start6_9
start6_3:	rcall	sync_with_poweron
		sbrc	flags2, COMP_SAVE
		rjmp	start6_2

start6_9:
		rcall	com6com1

		mov	temp1, tcnt0_power_on
		cpi	temp1, NO_POWER
		brne	s6_power_ok
		rjmp	init_startup

s6_power_ok:	tst	control_timeout
		brne	s6_rcp_ok
		rjmp	restart_control

s6_rcp_ok:	tst	t1_timeout
		brne	s6_test_rpm
		rjmp	init_startup		;-) demich
		
s6_test_rpm:	lds	temp1, timing_x
		tst	temp1
		brne	s6_goodies
		lds	temp1, timing_h		; get actual RPM reference high
;		cpi	temp1, PWR_RANGE1
		cpi	temp1, PWR_RANGE2
		brcs	s6_run1

s6_goodies:	lds	temp1, goodies
		sbrc	flags2, SCAN_TIMEOUT
		clr	temp1
		inc	temp1
		sts	goodies,  temp1
		cbr	flags2, (1<<SCAN_TIMEOUT)
		cpi	temp1, ENOUGH_GOODIES
		brcs	s6_start1	

s6_run1:	ldi	temp1, 0xff
		mov	run_control, temp1

		rcall	calc_next_timing
		rcall	set_OCT1_tot

		cbr	flags2, (1<<STARTUP)
		rjmp	run1			; running state begins

s6_start1:	rcall	start_timeout		; need to be here for a correct temp1=comp_state
		rjmp	start1			; go back to state 1

;-----bko-----------------------------------------------------------------
; **** running control loop ****

; run 1 = B(p-on) + C(n-choppered) - comparator A evaluated
; out_cA changes from low to high

run1:
.if	DEBUG_LEDS == 1
		in	temp1, PORTB
		sbr	temp1, (1<<GREEN_LED)+(1<<RED1_LED) ; green+red1 = off
		cbr	temp1, (1<<RED2_LED)		; red2 = on
		out	PORTB, temp1
.endif
 		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		sbr	flags1, (1<<EVAL_RPM)
		rcall	wait_OCT1_before_switch
		rcall	com1com2
		rcall	calc_next_timing
		rcall	wait_OCT1_tot
		
; run 2 = A(p-on) + C(n-choppered) - comparator B evaluated
; out_cB changes from high to low

run2:		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_OCT1_before_switch
		rcall	com2com3
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 3 = A(p-on) + B(n-choppered) - comparator C evaluated
; out_cC changes from low to high

run3:		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		sbr	flags1, (1<<EVAL_PWM)
		rcall	wait_OCT1_before_switch
		rcall	com3com4
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 4 = C(p-on) + B(n-choppered) - comparator A evaluated
; out_cA changes from high to low
run4:
.if	DEBUG_LEDS == 1
		in	temp1, PORTB
		sbr	temp1, (1<<GREEN_LED)+(1<<RED2_LED) ; green+red2 = off
		cbr	temp1, (1<<RED1_LED)		; red1 = on
		out	PORTB, temp1
.endif
		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_OCT1_before_switch
		rcall	com4com5
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 5 = C(p-on) + A(n-choppered) - comparator B evaluated
; out_cB changes from low to high

run5:		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		sbr	flags1, (1<<EVAL_SYS_STATE)
		rcall	wait_OCT1_before_switch
		rcall	com5com6
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 6 = B(p-on) + A(n-choppered) - comparator C evaluated
; out_cC changes from high to low

run6:
.if	DEBUG_LEDS == 1
		in	temp1, PORTB
		sbr	temp1, (1<<GREEN_LED) 			; green = off
		cbr	temp1, (1<<RED2_LED)+(1<<RED1_LED)	; red1+red2 = on
		out	PORTB, temp1
.endif
		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_OCT1_before_switch
		rcall	com6com1
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

		tst	control_timeout
;		breq	restart_control ; IF COMMENTED: LET MOTORS TURN UNBREAKED TILL UNDER 610RPM

		lds	temp1, timing_x
		tst	temp1
		breq	run6_2			; higher than 610 RPM if zero
		dec	temp1
		breq	run6_2			; higher than 610 RPM if equ 1
run_to_start:	sbr	flags2, (1<<STARTUP)
		cbr	flags2, (1<<POFF_CYCLE)
		rjmp	init_startup

run6_2:		cbr	flags2, (1<<POFF_CYCLE)
		tst	run_control		; only once !
		breq	run6_9
		dec	run_control
		breq	run6_3			; poweroff if 0
		mov	temp1, run_control
		cpi	temp1, 1		; poweroff if 1
		breq	run6_3
		cpi	temp1, 2		; poweroff if 2
		brne	run6_9
run6_3:		sbr	flags2, (1<<POFF_CYCLE)

run6_9:
		rjmp	run1			; go back to run 1

restart_control:
		rjmp	init_startup
		cli				; disable all interrupts
		rcall	switch_power_off
		rjmp	reset


;-----bko-----------------------------------------------------------------
; *** scan comparator utilities ***
; 
wait_for_low:	sbrs	flags0, OCT1_PENDING
		ret
		sbis	ACSR, ACO		; low ?
		rjmp	wait_for_low		; .. no - loop, while high
		rcall	wait_if_spike		; .. yes - look for a spike
		sbis	ACSR, ACO		; test again
		rjmp	wait_for_low		; .. is high again, was a spike
		ret

wait_for_high:	sbrs	flags0, OCT1_PENDING
		ret
		sbic	ACSR, ACO		; high ?
		rjmp	wait_for_high		; .. no - loop, while low
		rcall	wait_if_spike		; .. yes - look for a spike
		sbic	ACSR, ACO		; test again
		rjmp	wait_for_high		; .. is low again, was a spike
		ret
;-----bko-----------------------------------------------------------------
; *** commutation utilities ***
com1com2:	BpFET_off		; Bp off
		sbrs	flags1, POWER_OFF
		ApFET_on		; Ap on
		ldi	temp1, mux_b		; set comparator multiplexer to phase B
		out	ADMUX, temp1
		ret

com2com3:	ldi	temp1, (1<<OCIE1A)+(1<<TOIE1) ; stop timer0 interrupt
		out	TIMSK, temp1		;  .. only ONE should change these values at the time
		nop
		cbr	flags0, (1<<A_FET)	; next nFET = BnFET
		cbr	flags0, (1<<C_FET)
		sbrc	flags1, FULL_POWER
		rjmp	c2_switch
		sbrc	flags0, I_OFF_CYCLE	; was power off ?
		rjmp	c2_done			; .. yes - futhermore work is done in timer0 interrupt
c2_switch:	CnFET_off		; Cn off
		sbrs	flags1, POWER_OFF
		BnFET_on		; Bn on
c2_done:	ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0) ; let timer0 do his work again
		out	TIMSK, temp1
		ldi	temp1, mux_c		; set comparator multiplexer to phase C
		out	ADMUX, temp1
		ret

com3com4:	ApFET_off		; Ap off
		sbrs	flags1, POWER_OFF
		CpFET_on		; Cp on
		ldi	temp1, mux_a		; set comparator multiplexer to phase A
		out	ADMUX, temp1
		ret

com4com5:	ldi	temp1, (1<<OCIE1A)+(1<<TOIE1) ; stop timer0 interrupt
		out	TIMSK, temp1		;  .. only ONE should change these values at the time
		nop
		sbr	flags0, (1<<A_FET)	; next nFET = AnFET
		cbr	flags0, (1<<C_FET)
		sbrc	flags1, FULL_POWER
		rjmp	c4_switch
		sbrc	flags0, I_OFF_CYCLE	; was power off ?
		rjmp	c4_done			; .. yes - futhermore work is done in timer0 interrupt
c4_switch:	BnFET_off		; Bn off
		sbrs	flags1, POWER_OFF
		AnFET_on		; An on
c4_done:	ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0) ; let timer0 do his work again
		out	TIMSK, temp1
		ldi	temp1, mux_b		; set comparator multiplexer to phase B
		out	ADMUX, temp1
		ret

com5com6:	CpFET_off		; Cp off
		sbrs	flags1, POWER_OFF
		BpFET_on		; Bp on
		ldi	temp1, mux_c		; set comparator multiplexer to phase C
		out	ADMUX, temp1
		ret

com6com1:	ldi	temp1, (1<<OCIE1A)+(1<<TOIE1) ; stop timer0 interrupt
		out	TIMSK, temp1		;  .. only ONE should change these values at the time
		nop
		cbr	flags0, (1<<A_FET)	; next nFET = CnFET
		sbr	flags0, (1<<C_FET)
		sbrc	flags1, FULL_POWER
		rjmp	c6_switch
		sbrc	flags0, I_OFF_CYCLE	; was power off ?
		rjmp	c6_done			; .. yes - futhermore work is done in timer0 interrupt
c6_switch:	AnFET_off		; An off
		sbrs	flags1, POWER_OFF
		CnFET_on		; Cn on
c6_done:	ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0) ; let timer0 do his work again
		out	TIMSK, temp1
		ldi	temp1, mux_a		; set comparator multiplexer to phase A
		out	ADMUX, temp1
		ret

.exit
