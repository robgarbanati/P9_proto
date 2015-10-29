/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef __APU_H__
#define __APU_H__

/*--------------------------- Audio Processing Unit Controller --------------------------*/
// APUCR(APU Control Register)
#define APU_APUCR_TSHD				BIT(2,0)			// APU Interrupt Threshold
#define APU_APUCR_APUIS				BIT5				// APU Interrupt Status
#define APU_APUCR_APUIE				BIT6				// APU Interrupt Enable
#define APU_APUCR_PAMPE				BIT7				// Power Amplifier Enable
#define APU_APUCR_DACE				BIT8				// DAC Enable
#define APU_APUCR_BPPAM				BIT9				// Bypass Power Amplifier, DAC output to Pin
#define APU_APUCR_DACGN				BIT13				// DAC Output Current Control

// APUVM(APU Volume Control Register)
#define APU_APUVM_VOLUM				BIT(2,0)			// APU Volume Adjustment

// DACCH1(DAC channel 1 Data Buffer Register)
#define APU_DACCH1_PCM				BIT(15,0)			// PCM Data for DAC

// APUDRx(DAC channel 0 Data Buffer Register 0~7)
#define APU_APUDR_PCM				BIT(15,0)			// PCM Data for DAC


typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  TSHD:3;			// APU Interrupt Threshold
		__I  uint32_t  RESERVE0:2;
		__IO uint32_t  APUIS:1;			// APU Interrupt Status
		__IO uint32_t  APUIE:1;			// APU Interrupt Enable
		__IO uint32_t  PAMPE:1;			// Power Amplifier Enable
		__IO uint32_t  DACE:1;			// DAC Enable
		__IO uint32_t  BPPAM:1;			// Bypass Power Amplifier, DAC output to Pin
		__I  uint32_t  RESERVE1:3;
		__IO uint32_t  DACGN:1;			// DAC Output Current Control
	};
} APUCR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  VOLUM:3;			// APU Volume Adjustment
	};
} APUVM_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  PCM:16;			// PCM Data for DAC
	};
} DACCH1_T;

typedef DACCH1_T	APUDR_T;

typedef struct
{
	APUCR_T			APUCR;				// APU Control Register
	APUVM_T			APUVM;				// APU Volume Control Register
	__I	 uint32_t	RESERVE0;
	DACCH1_T		DACCH1;				// DAC channel 1 Data Buffer Register
	APUDR_T			APUDR[8];			// DAC channel 0 Data Buffer Register 0~7
} APU_T;

#endif
