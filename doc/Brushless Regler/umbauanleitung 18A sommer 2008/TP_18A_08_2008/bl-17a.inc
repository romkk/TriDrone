;*********************************************************** 
;* BL-17A                                                  *
;* Dez.2006
;* http://home.versanet.de/~b-konze/                       *
;*********************************************************** 
; minor changes: Oct 2007, Hans Haider, h.haider@comdesign.at - marked with #HH#
;*********************************************************** 

#define Typ "tp-esc18"

.equ CPU_FREQ = 16

; activate Motor brake 

.equ MOT_BRAKE      = 0

.equ	DEBUG_LEDS = 0


;**** **** **** **** ****
; PORT  definitions
;**** **** **** **** ****

;*********************
; PORT D 
;*********************  
.equ	BpFET		= 7	;o
.equ	c_comp		= 6	;i common comparator input (AIN0)
.equ	ApFET		= 5	;o
.equ	CpFET		= 4	;o
.equ	CnFET		= 3	;o
.equ	rcp_in		= 2	;i r/c pulse input
.equ	BnFET		= 1	; #HH#
;.equ			= 0	;
.equ	INIT_PD		= 0b00000001

.equ	DIR_PD		= (1<<BpFET)+(1<<CpFET)+(1<<BnFET)+(1<<CnFET)+(1<<ApFET)
.equ	BRAKE_PD	= (1<<BnFET)+(1<<CnFET)

#define BpFET_on	sbi	PORTD,7
#define BpFET_off	cbi	PORTD,7
#define ApFET_on	sbi	PORTD,5
#define ApFET_off	cbi	PORTD,5
#define CpFET_on	sbi	PORTD,4
#define CpFET_off	cbi	PORTD,4
#define CnFET_on	sbi	PORTD,3
#define CnFET_off	cbi	PORTD,3
#define BnFET_on	sbi	PORTD,1
#define BnFET_off	cbi	PORTD,1
#define AnFET_on	sbi	PORTC,0
#define AnFET_off	cbi	PORTC,0


;*********************
; PORT C definitions *
;*********************
.equ	AnFET		= 0	; o
;.equ	ApFET		= 1	; o 
.equ	mux_a		= 2	; ADC2 phase input
.equ  mux_b  	= 3     ; ADC3 phase input 
.equ	mux_c		= 4	; ADC4 phase input (SDA)
;.equ    	  	= 5	; ADC5 SCL

;*********************
; ADC only
;*********************
;.equ    	  	= 6	; ADC6
.equ 	ACCU_MUX	= 7 	; ADC7 voltage control input 

.equ	INIT_PC		= 0 ; #HH# (1<<AnFET)
.equ	DIR_PC		= (1<<AnFET) ; #HH#
.equ	BRAKE_PC	= (1<<AnFET)


;*********************
; PORT B definitions *
;*********************
;.equ		= 7	
;.equ		= 6	
;.equ		= 5    	; (sck stk200 interface) 
;.equ		= 4	; (miso stk200 interface)
;.equ   SPI_MOSI	= 3	; (mosi stk200 interface) 
;.equ		= 2	; (ss for spi control)
;.equ	SPI_SS	= 1	
;.equ		= 0	; connected with pb4 ???	

#define HHDEBUGLED_on  sbi PORTB,1 ; #HH# LED at pin 13 of atmega8
#define HHDEBUGLED_off cbi PORTB,1 ; #HH# LED at pin 13 of atmega8
.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<2) + (1<<1)	; PB2=0 - whyever ;-)   PB1=debugled #HH#

.equ	BRAKE_PB	= 0

