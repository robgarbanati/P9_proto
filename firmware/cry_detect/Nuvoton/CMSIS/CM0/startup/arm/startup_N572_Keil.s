;/*---------------------------------------------------------------------------------------------------------*/
;/*																											*/
;/*	Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.											*/
;/*																											*/
;/*---------------------------------------------------------------------------------------------------------*/

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;







Stack_Size		EQU		0x00000400

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
				DCD		NMI_Handler				  ;	NMI	Handler
				DCD		HardFault_Handler		  ;	Hard Fault Handler
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		SVC_Handler				  ;	SVCall Handler
				DCD		0						  ;	Reserved
				DCD		0						  ;	Reserved
				DCD		PendSV_Handler			  ;	PendSV Handler
				DCD		SysTick_Handler			  ;	SysTick	Handler

				; External Interrupts
												  ;	maximum	of 32 External Interrupts are possible
				DCD		WDT_IRQHandler	
				DCD		APU_IRQHandler	
				DCD		ADC_IRQHandler
				DCD		EXINT_IRQHandler
				DCD		USB_IRQHandler 
				DCD		TMR0_IRQHandler
				DCD		TMR1_IRQHandler	
				DCD		TMR2_IRQHandler	
				DCD		GPAB_IRQHandler	
				DCD		SPI0_IRQHandler	
				DCD		PWM0_IRQHandler	
				DCD		SPI1_IRQHandler	
				DCD		TMRF_IRQHandler
				DCD		RTC_IRQHandler
				DCD		PWRWU_IRQHandler


			
				AREA	|.text|, CODE, READONLY
				
				
				
; Reset	Handler	
				
				ENTRY
				
Reset_Handler	PROC
				EXPORT	Reset_Handler			  [WEAK]
				IMPORT	__main
				
				LDR		R0,	=__main
				BX		R0
				ENDP
				
				
; Dummy	Exception Handlers (infinite loops which can be	modified)				 
				
NMI_Handler		PROC
				EXPORT	NMI_Handler				  [WEAK]
				B		.
				ENDP
HardFault_Handler\
				PROC
				EXPORT	HardFault_Handler		  [WEAK]
				B		.
				ENDP
SVC_Handler		PROC
				EXPORT	SVC_Handler				  [WEAK]
				B		.
				ENDP
PendSV_Handler	PROC
				EXPORT	PendSV_Handler			  [WEAK]
				B		.
				ENDP
SysTick_Handler	PROC
				EXPORT	SysTick_Handler			  [WEAK]
				B		.
				ENDP

Default_Handler	PROC

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


				END
