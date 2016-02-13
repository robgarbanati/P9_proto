/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef __SYSINT_H__
#define __SYSINT_H__

/*---------------------------- Interrupt Controller ------------------------------*/
// IRQN_SRC(Interrupt Source Identify Control Register)
#define SYSINT_IRQN_SRC_SYSINT_SRC	BIT(2,0)			// interrupt source identify

// NMI_SEL(NMI Interrupt Source Select Control Register)
#define SYSINT_NMI_SEL_NMI_SEL		BIT(3,0)			// The NMI interrupt to Cortex-M0 can be selected from one of 16 interrupts.
#define SYSINT_NMI_SEL_INT_TEST		BIT7				// Interrupt test mode enable

// MCU_IRQ(MCU Interrupt Request Source Register)
#define SYSINT_MCU_IRQ_MCU_IRQ		BIT(15,0)			// MCU IRQ Source Register

typedef union
{
	__I  uint32_t  u32Reg;
	struct
	{
		__I  uint32_t  INT_SRC:3;						// interrupt source identify
	};
} SYSINT_IRQN_SRC_T;

typedef union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  NMI_SEL:4;						// The NMI interrupt to Cortex-M0 can be selected from one of 16 interrupts.
		__I  uint32_t  RESERVE:3;
		__IO uint32_t  INT_TEST:1;						// Interrupt test mode enable
	};
} SYSINT_NMI_SEL_T;

typedef union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  MCU_IRQ:16;						// MCU IRQ Source Register
	};
} SYSINT_MCU_IRQ_T;

typedef struct
{
	SYSINT_IRQN_SRC_T	IRQN_SRC[15];					// Interrupt Source Identify Control Register
	__I  uint32_t		RESERVE[17];
	SYSINT_NMI_SEL_T	NMI_SEL;						// NMI Interrupt Source Select Control Register
	SYSINT_MCU_IRQ_T	MCU_IRQ;   						// MCU Interrupt Request Source Register
} SYSINT_T;

#endif
