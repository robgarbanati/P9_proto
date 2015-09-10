/*---------------------------------------------------------------------------------------------------------*/
/*																										   */
/* Copyright (c) Nuvoton Technology	Corp. All rights reserved.											   */
/*																										   */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef	__FMC_H__
#define	__FMC_H__

/*-------------------------- FLASH Memory Controller -------------------------*/
// ISPCON(ISP Control Register)
#define	FMC_ISPCON_ISPEN		BIT0			// ISP Enable
#define	FMC_ISPCON_EWEN			BIT1			// Enable Erase/Write of ISP function
#define	FMC_ISPCON_CFUEN		BIT4			// Config0 Update Enable
#define	FMC_ISPCON_ISPFF		BIT6			// ISP Fail	Flag
#define	FMC_ISPCON_PT			BIT(10,8)		// Flash Program Time
#define	FMC_ISPCON_ET			BIT(14,12)	// Flash Erase Time

// ISPCMD(ISP Address Register)
#define FMC_ISPCMD_FCTRL		BIT(3,0)		// ISP control
#define FMC_ISPCMD_FCEN			BIT4			// FCEN: Flash Chip Enable
#define FMC_ISPCMD_FOEN			BIT5			// FOEN: Flash Output Enable

// ISPTRG(ISP Trigger Register)
#define FMC_ISPTRG_ISPGO		BIT0			// ISP start trigger

// ICPCON(ICP Control Register)
#define FMC_ICPCON_ICPEN		BIT0			// ICP Enable

// RMPCON(ROMMAP Control Register)
#define FMC_RMPCON_ICPRMP		BIT0			// ICP ROMMAP Flag

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  ISPEN:1;					// ISP Enable
		__IO uint32_t  EWEN:1;					// Enable Erase/Write of ISP function
		__I	 uint32_t  RESERVE0:2;
		__IO uint32_t  CFUEN:1;					// Config0 Update Enable
		__I	 uint32_t  RESERVE1:1;
		__IO uint32_t  ISPFF:1;					// ISP Fail	Flag
		__I	 uint32_t  RESERVE2:1;
		__IO uint32_t  PT:3;					  // Flash Program Time
		__I	 uint32_t  RESERVE3:1;
		__IO uint32_t  ET:3;					  // Flash Erase Time
		__I	 uint32_t  RESERVE4:17;
	};
} FMC_ISPCON_T;

typedef	__IO uint32_t FMC_ISPADR_T;
typedef	__IO uint32_t FMC_ISPDAT_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  FCTRL:4;					// ISP control
		__IO uint32_t  FCEN:1;					// FCEN: Flash Chip Enable
		__IO uint32_t  FOEN:1;					// FOEN: Flash Output Enable
	};
} FMC_ISPCMD_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  ISPGO:1;					// ISP start trigger
		__I	 uint32_t  RESERVE:31;
	};
} FMC_ISPTRG_T;

typedef	__I	uint32_t FMC_DFBADR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  FPSEN:1;					// Flash Power Save Enable
		__IO uint32_t  FATS:3;					// Flash Access Time Window Select
		__I	 uint32_t  RESERVE:28;
	};
} FMC_FATCON_T;

typedef	struct
{
	FMC_ISPCON_T  ISPCON;				// ISP Control Register
	FMC_ISPADR_T  ISPADR;				// ISP Address Register
	FMC_ISPDAT_T  ISPDAT;				// ISP Data	Register
	FMC_ISPCMD_T  ISPCMD;				// ISP Command Register
	FMC_ISPTRG_T  ISPTRG;				// ISP Trigger Register
} FMC_T;
#endif
