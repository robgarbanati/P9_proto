/*---------------------------------------------------------------------------------------------------------*/
/*																										   */
/* Copyright (c) Nuvoton Technology	Corp. All rights reserved.											   */
/*																										   */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef	__WDT_H__
#define	__WDT_H__

/*----------------------------- WDT Controller -----------------------------*/
// WTCR(Watchdog Timer Control Register)
#define WDT_WTCR_CLRWT			BIT0					// WDT Clear
#define WDT_WTCR_WTRE			BIT1					// WDT Reset Enable
#define WDT_WTCR_WTRF			BIT2					// WDT Reset Flag
#define WDT_WTCR_WTIF			BIT3					// WDT Interrupt Flag
#define WDT_WTCR_WTWKE			BIT4					// WDT Wakeup Function Enable bit
#define WDT_WTCR_WTWKF			BIT5					// WDT Wakeup Flag
#define WDT_WTCR_WTIE			BIT6					// WDT Interrupt Enable
#define WDT_WTCR_WTE			BIT7					// WDT Enable
#define WDT_WTCR_WTIS			BIT(10,8)				// WDT Interval Select


typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  CLRWT:1;							// WDT Clear
		__IO uint32_t  WTRE:1;							// WDT Reset Enable
		__IO uint32_t  WTRF:1;							// WDT Reset Flag
		__IO uint32_t  WTIF:1;							// WDT Interrupt Flag
		__IO uint32_t  WTWKE:1;							// WDT Wakeup Function Enable bit
		__IO uint32_t  WTWKF:1;							// WDT Wakeup Flag
		__IO uint32_t  WTIE:1;							// WDT Interrupt Enable
		__IO uint32_t  WTE:1;							// WDT Enable
		__IO uint32_t  WTIS:3;							// WDT Interval Select
	};
} WDT_WTCR_T;

typedef struct
{
	WDT_WTCR_T     WTCR;
} WDT_T;
 
 #endif
