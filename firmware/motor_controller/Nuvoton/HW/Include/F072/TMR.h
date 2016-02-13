/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef __TMR_H__
#define __TMR_H__

/*----------------------------- Timer Controller -----------------------------*/
// TCSR(Timer Control and Status Register)
#define TMR_TCSR_PRESCALE		BIT(7,0)				// Timer clock prescaler
#define TMR_TCSR_Reserved		BIT(24,8)				// Reserved
#define TMR_TCSR_CACT			BIT25					// Timer is in Active
#define TMR_TCSR_CRST			BIT26					// Counter Reset
#define TMR_TCSR_MODE			BIT(28,27)				// Timer Operating Mode
#define TMR_TCSR_TIE			BIT29					// Interrupt Enable Bit for 072 name is TMIE
#define TMR_TCSR_CEN			BIT30					// Counter Enable Bit for 072 name is TCEN

// TCMPR(Timer Compare Register)
#define TMR_TCMPR_TCMP			BIT(15,0)				// Timer Compare Value

// TISR(Timer Interrupt Status Register)
#define TMR_TISR_TIF			BIT0					// Timer Interrupt Flag (Read Only) for 072 name is TMIF

// TDR(Timer Data Register)
#define TMR_TDR_TDR				BIT1					// Timer Data Register

// TFIR(TimerF Interrupt Register )
#define TMR_TFIR_TFIF			BIT0					// TimerF Interrupt Flag
#define TMR_TFIR_TFIE			BIT1					// TimerF Interrupt Enable

// IRCAR(IR Carrier Output Control Register)
#define TMR_IRCAR_NONCS			BIT0					// Non-carrier state
#define TMR_IRCAR_IRCEN			BIT1					// IR carrier output enable

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
	    __IO uint32_t  PRESCALE:8;						// Timer clock prescaler
	    __I  uint32_t  RESERVE0:17;						// Reserved
	    __IO uint32_t  CACT:1;							// Timer is in Active
	    __IO uint32_t  CRST:1;							// Counter Reset
	    __IO uint32_t  MODE:2;							// Timer Operating Mode
	    __IO uint32_t  TIE:1;							// Interrupt Enable Bit for 072 name is TMIE
	    __IO uint32_t  CEN:1;							// Counter Enable Bit for 072 name is TCEN
    };
} TMR_TCSR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  TCMP:16;							// Timer Compare Value
	};
} TMR_TCMPR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
	    __IO uint32_t  TIF:1;							// Timer Interrupt Flag for 072 name is TMIF
    };
} TMR_TISR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
	    __IO uint32_t  TDR:16;							// Timer Data Register
    };
} TMR_TDR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
	    __IO uint32_t  TFIF:1;							// TimerF Interrupt Flag
	    __IO uint32_t  TFIE:1;							// TimerF Interrupt Enable
    };
} TMR_TFIR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
	    __IO uint32_t  NONCS:1;							// Non-carrier state
	    __IO uint32_t  IRCEN:1;							// IR carrier output enable
    };
} TMR_IRCAR_T;


typedef struct
{
	TMR_TCSR_T		TCSR;								// Timer Control and Status Register
	TMR_TCMPR_T		TCMPR;								// Timer Compare Register
	TMR_TISR_T		TISR;								// Timer Interrupt Status Register
	TMR_TDR_T		TDR;								// Timer Data Register
} TMR_T;


typedef struct
{
	TMR_IRCAR_T		IRCAR;								// IR Carrier Output Control Register
}TMR_IR_T;


typedef struct
{
	TMR_TFIR_T		TFIR;								// TimerF Interrupt Register
} TMRF_T;

#endif
