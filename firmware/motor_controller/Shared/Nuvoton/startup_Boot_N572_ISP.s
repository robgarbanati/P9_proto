;/*---------------------------------------------------------------------------------------------------------*/
;/*																											*/
;/*	Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.											*/
;/*																											*/
;/*---------------------------------------------------------------------------------------------------------*/

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Stack_Size		EQU		0x00000300

				AREA	STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem		SPACE	Stack_Size
__initial_sp

; <h> Heap Configuration
;	<o>	 Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size		EQU		0x00000000

				AREA	HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem		SPACE	Heap_Size
__heap_limit


				PRESERVE8
				THUMB


; Vector Table Mapped to Address 0 at Reset
				AREA	RESET, DATA, READONLY
				EXPORT	__Vectors


__Vectors		DCD		__initial_sp			  ;	Top	of Stack
				DCD		Reset_Handler			  ;	Reset Handler
				DCD		NMI_Wrapper				  ;	NMI	Handler
				DCD		HardFault_Wrapper		  ;	Hard Fault Handler
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		SVC_Wrapper				  ;	SVCall Handler
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		PendSV_Wrapper			  ;	PendSV Handler
				DCD		SysTick_Wrapper			  ;	SysTick	Handler

				; External Interrupts
												  ;	maximum	of 32 External Interrupts are possible
				DCD		WDT_Wrapper	
				DCD		APU_Wrapper	
				DCD		ADC_Wrapper
				DCD		EXINT_Wrapper
				DCD		USB_Wrapper 
				DCD		TMR0_Wrapper
				DCD		TMR1_Wrapper	
				DCD		TMR2_Wrapper	
				DCD		GPAB_Wrapper	
				DCD		SPI0_Wrapper	
				DCD		PWM0_Wrapper	
				DCD		SPI1_Wrapper	
				DCD		TMRF_Wrapper
				DCD		RTC_Wrapper
				DCD		PWRWU_Wrapper

 
; Start of area "EXCEPTION"
				AREA	EXCEPTION, CODE, READONLY			
; Dummy	Exception Handlers (infinite loops which can be	modified)
				
NMI_Wrapper		PROC
				EXPORT	NMI_Wrapper				  [WEAK]
				LDR		R0, =NMI_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP

HardFault_Wrapper\
				PROC
				EXPORT	HardFault_Wrapper		  [WEAK]
				LDR		R0, =HardFault_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP

SVC_Wrapper		PROC
				EXPORT	SVC_Wrapper				  [WEAK]
				LDR		R0, =SVC_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP

PendSV_Wrapper	PROC
				EXPORT	PendSV_Wrapper			  [WEAK]
				LDR		R0, =PendSV_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP

SysTick_Wrapper	PROC
				EXPORT	SysTick_Wrapper			  [WEAK]
				LDR		R0, =SysTick_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP

WDT_Wrapper		PROC
				LDR		R0, =WDT_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP

APU_Wrapper		PROC 
				LDR		R0, =APU_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

ADC_Wrapper		PROC
				LDR		R0, =ADC_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

EXINT_Wrapper	PROC
				LDR		R0, =EXINT_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

USB_Wrapper		PROC
				LDR		R0, =USB_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

TMR0_Wrapper	PROC
				LDR		R0, =TMR0_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

TMR1_Wrapper	PROC
				LDR		R0, =TMR1_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

TMR2_Wrapper	PROC
				LDR		R0, =TMR2_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

GPAB_Wrapper	PROC
				LDR		R0, =GPAB_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

SPI0_Wrapper	PROC
				LDR		R0, =SPI0_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

PWM0_Wrapper	PROC
				LDR		R0, =PWM0_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

SPI1_Wrapper	PROC
				LDR		R0, =SPI1_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

TMRF_Wrapper	PROC
				LDR		R0, =TMRF_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

RTC_Wrapper		PROC
				LDR		R0, =RTC_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP	

PWRWU_Wrapper	PROC
				LDR		R0, =PWRWU_ISR
				LDR		R0, [R0]
				BX		R0
				ENDP

Default_Handler	PROC 				   
				EXPORT	HardFault_Handler		  [WEAK] 
				EXPORT	NMI_Handler				  [WEAK] 
				EXPORT	SVC_Handler				  [WEAK] 
				EXPORT	PendSV_Handler			  [WEAK] 
				EXPORT	SysTick_Handler			  [WEAK]
HardFault_Handler
NMI_Handler
SVC_Handler
PendSV_Handler
SysTick_Handler
				EXPORT	WDT_IRQHandler			  [WEAK]
				EXPORT	APU_IRQHandler			  [WEAK]
				EXPORT	ADC_IRQHandler			  [WEAK]
				EXPORT	EXINT_IRQHandler		  [WEAK]
				EXPORT	USB_IRQHandler			  [WEAK]
				EXPORT	TMR0_IRQHandler			  [WEAK]
				EXPORT	TMR1_IRQHandler			  [WEAK]
				EXPORT	TMR2_IRQHandler			  [WEAK]
				EXPORT	GPAB_IRQHandler			  [WEAK]
				EXPORT	SPI0_IRQHandler			  [WEAK]
				EXPORT	PWM0_IRQHandler			  [WEAK]
				EXPORT	SPI1_IRQHandler			  [WEAK]
				EXPORT	TMRF_IRQHandler			  [WEAK]
				EXPORT	RTC_IRQHandler            [WEAK]
				EXPORT	PWRWU_IRQHandler          [WEAK]
				
WDT_IRQHandler	
APU_IRQHandler	
ADC_IRQHandler
EXINT_IRQHandler
USB_IRQHandler 
TMR0_IRQHandler
TMR1_IRQHandler	
TMR2_IRQHandler	
GPAB_IRQHandler	
SPI0_IRQHandler	
PWM0_IRQHandler	
SPI1_IRQHandler	
TMRF_IRQHandler
RTC_IRQHandler
PWRWU_IRQHandler
				B		.
				ENDP
				ALIGN
; End of area "EXCEPTION"

; Start of area "STARTUP"				 
				AREA	STARTUP, CODE, READONLY		

; Reset	Handler		
				ENTRY		
Reset_Handler	PROC
				EXPORT	Reset_Handler			  [WEAK]
				IMPORT	__main
				
				LDR		R0,	=__main
				BX		R0
				ENDP 
; User Initial Stack & Heap

				IF		:DEF:__MICROLIB
				
				EXPORT	__initial_sp
				EXPORT	__heap_base
				EXPORT	__heap_limit
				
				ELSE
				
				IMPORT	__use_two_region_memory
				EXPORT	__user_initial_stackheap
__user_initial_stackheap

				LDR		R0,	=  Heap_Mem
				LDR		R1,	= (Stack_Mem + Stack_Size)
				LDR		R2,	= (Heap_Mem	+  Heap_Size)
				LDR		R3,	= Stack_Mem
				BX		LR

				ALIGN

				ENDIF

				ALIGN
; End of area "STARTUP"

; Start of area "IRQTABLE"			
				AREA	IRQTABLE, DATA, READWRITE
Reset_ISR		DCD		Reset_Handler
HardFault_ISR	DCD		HardFault_Handler
SVC_ISR			DCD		SVC_Handler
PendSV_ISR		DCD		PendSV_Handler
SysTick_ISR		DCD		SysTick_Handler
NMI_ISR			DCD		NMI_Handler
												  ;	maximum	of 32 External Interrupts are possible
WDT_ISR			DCD		WDT_IRQHandler	
APU_ISR			DCD		APU_IRQHandler	
ADC_ISR			DCD		ADC_IRQHandler
EXINT_ISR		DCD		EXINT_IRQHandler
USB_ISR			DCD		USB_IRQHandler 
TMR0_ISR		DCD		TMR0_IRQHandler
TMR1_ISR		DCD		TMR1_IRQHandler	
TMR2_ISR		DCD		TMR2_IRQHandler	
GPAB_ISR		DCD		GPAB_IRQHandler	
SPI0_ISR		DCD		SPI0_IRQHandler	
PWM0_ISR		DCD		PWM0_IRQHandler	
SPI1_ISR		DCD		SPI1_IRQHandler	
TMRF_ISR		DCD		TMRF_IRQHandler
RTC_ISR			DCD		RTC_IRQHandler
PWRWU_ISR		DCD		PWRWU_IRQHandler
				ALIGN
; End of area "IRQTABLE"
				EXPORT	HardFault_ISR
				EXPORT	SVC_ISR
				EXPORT  PendSV_ISR
				EXPORT	SysTick_ISR
				EXPORT	NMI_ISR
				EXPORT	WDT_ISR
				EXPORT	APU_ISR
				EXPORT	ADC_ISR
				EXPORT	EXINT_ISR
				EXPORT	USB_ISR
				EXPORT	TMR0_ISR
				EXPORT	TMR1_ISR
				EXPORT	TMR2_ISR
				EXPORT	GPAB_ISR
				EXPORT	SPI0_ISR
				EXPORT	PWM0_ISR
				EXPORT	SPI1_ISR
				EXPORT	TMRF_ISR
				EXPORT	RTC_ISR
				EXPORT	PWRWU_ISR

; Start of area "JMPTABLE"            
                AREA    JMPTABLE, DATA, READONLY
appJumpAddress  DCD     Reset_Handler
                ALIGN
; End of area "JMPTABLE"
                EXPORT    appJumpAddress

				END
