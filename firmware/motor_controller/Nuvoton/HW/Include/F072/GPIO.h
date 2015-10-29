/****************************************************************
 *																*
 * Copyright (c) Nuvoton Technology	Corp. All rights reserved.	*
 *																*
 ****************************************************************/
#ifndef	__GPIO_H__
#define	__GPIO_H__

/*---------------------	General	Purpose	Input and Ouptut ---------------------*/
// GPIO_PMD(GPIO Port Bit Mode Control)
#define	GPIO_PMD_PMD0		BIT(1, 0)				// GPIO I/O Pin0 Mode Control 
#define	GPIO_PMD_PMD1		BIT(3, 2)				// GPIO I/O Pin1 Mode Control 
#define	GPIO_PMD_PMD2		BIT(5, 4)				// GPIO I/O Pin2 Mode Control 
#define	GPIO_PMD_PMD3		BIT(7, 6)				// GPIO I/O Pin3 Mode Control 
#define	GPIO_PMD_PMD4		BIT(9, 8)				// GPIO I/O Pin4 Mode Control 
#define	GPIO_PMD_PMD5		BIT(11,10)				// GPIO I/O Pin5 Mode Control 
#define	GPIO_PMD_PMD6		BIT(13,12)				// GPIO I/O Pin6 Mode Control 
#define	GPIO_PMD_PMD7		BIT(15,14)				// GPIO I/O Pin7 Mode Control 
#define	GPIO_PMD_PMD8		BIT(17,16)				// GPIO I/O Pin8 Mode Control 
#define	GPIO_PMD_PMD9		BIT(19,18)				// GPIO I/O Pin9 Mode Control 
#define	GPIO_PMD_PMD10		BIT(21,20)				// GPIO I/O Pin10 Mode Control 
#define	GPIO_PMD_PMD11		BIT(23,22)				// GPIO I/O Pin11 Mode Control 
#define	GPIO_PMD_PMD12		BIT(25,24)				// GPIO I/O Pin12 Mode Control 
#define	GPIO_PMD_PMD13		BIT(27,26)				// GPIO I/O Pin13 Mode Control 
#define	GPIO_PMD_PMD14		BIT(29,28)				// GPIO I/O Pin14 Mode Control 
#define	GPIO_PMD_PMD15		BIT(31,30)				// GPIO I/O Pin15 Mode Control 

// GPIO_OFFD(GPIO Port Bit OFF Digital Enable)
#define GPIO_OFFD_OFFD0		BIT16				// GPIO Pin0 OFF digital input path enable
#define GPIO_OFFD_OFFD1		BIT17				// GPIO Pin1 OFF digital input path enable
#define GPIO_OFFD_OFFD2		BIT18				// GPIO Pin2 OFF digital input path enable
#define GPIO_OFFD_OFFD3		BIT19				// GPIO Pin3 OFF digital input path enable
#define GPIO_OFFD_OFFD4		BIT20				// GPIO Pin4 OFF digital input path enable
#define GPIO_OFFD_OFFD5		BIT21				// GPIO Pin5 OFF digital input path enable
#define GPIO_OFFD_OFFD6		BIT22				// GPIO Pin6 OFF digital input path enable
#define GPIO_OFFD_OFFD7		BIT23				// GPIO Pin7 OFF digital input path enable
#define GPIO_OFFD_OFFD8		BIT24				// GPIO Pin8 OFF digital input path enable
#define GPIO_OFFD_OFFD9		BIT25				// GPIO Pin9 OFF digital input path enable
#define GPIO_OFFD_OFFD10	BIT26				// GPIO Pin10 OFF digital input path enable
#define GPIO_OFFD_OFFD11	BIT27				// GPIO Pin11 OFF digital input path enable
#define GPIO_OFFD_OFFD12	BIT28				// GPIO Pin12 OFF digital input path enable
#define GPIO_OFFD_OFFD13	BIT29				// GPIO Pin13 OFF digital input path enable
#define GPIO_OFFD_OFFD14	BIT30				// GPIO Pin14 OFF digital input path enable
#define GPIO_OFFD_OFFD15	BIT31				// GPIO Pin15 OFF digital input path enable

// GPIO_DOUT(GPIO Port Data Output Value)
#define GPIO_DOUT_DOUT0		BIT0				// GPIO Pin0 Output Value
#define GPIO_DOUT_DOUT1		BIT1				// GPIO Pin1 Output Value
#define GPIO_DOUT_DOUT2		BIT2				// GPIO Pin2 Output Value
#define GPIO_DOUT_DOUT3		BIT3				// GPIO Pin3 Output Value
#define GPIO_DOUT_DOUT4		BIT4				// GPIO Pin4 Output Value
#define GPIO_DOUT_DOUT5		BIT5				// GPIO Pin5 Output Value
#define GPIO_DOUT_DOUT6		BIT6				// GPIO Pin6 Output Value
#define GPIO_DOUT_DOUT7		BIT7				// GPIO Pin7 Output Value
#define GPIO_DOUT_DOUT8		BIT8				// GPIO Pin8 Output Value
#define GPIO_DOUT_DOUT9		BIT9				// GPIO Pin9 Output Value
#define GPIO_DOUT_DOUT10	BIT10				// GPIO Pin10 Output Value
#define GPIO_DOUT_DOUT11	BIT11				// GPIO Pin11 Output Value
#define GPIO_DOUT_DOUT12	BIT12				// GPIO Pin12 Output Value
#define GPIO_DOUT_DOUT13	BIT13				// GPIO Pin13 Output Value
#define GPIO_DOUT_DOUT14	BIT14				// GPIO Pin14 Output Value
#define GPIO_DOUT_DOUT15	BIT15				// GPIO Pin15 Output Value

// GPIO_DMASK(GPIO Port Data Output Write Mask)
#define GPIO_DMASK_DMASK0	BIT0				// GPIO Pin0 Data Output Write Mask
#define GPIO_DMASK_DMASK1	BIT1				// GPIO Pin1 Data Output Write Mask
#define GPIO_DMASK_DMASK2	BIT2				// GPIO Pin2 Data Output Write Mask
#define GPIO_DMASK_DMASK3	BIT3				// GPIO Pin3 Data Output Write Mask
#define GPIO_DMASK_DMASK4	BIT4				// GPIO Pin4 Data Output Write Mask
#define GPIO_DMASK_DMASK5	BIT5				// GPIO Pin5 Data Output Write Mask
#define GPIO_DMASK_DMASK6	BIT6				// GPIO Pin6 Data Output Write Mask
#define GPIO_DMASK_DMASK7	BIT7				// GPIO Pin7 Data Output Write Mask
#define GPIO_DMASK_DMASK8	BIT8				// GPIO Pin8 Data Output Write Mask
#define GPIO_DMASK_DMASK9	BIT9				// GPIO Pin9 Data Output Write Mask
#define GPIO_DMASK_DMASK10	BIT10				// GPIO Pin10 Data Output Write Mask
#define GPIO_DMASK_DMASK11	BIT11				// GPIO Pin11 Data Output Write Mask
#define GPIO_DMASK_DMASK12	BIT12				// GPIO Pin12 Data Output Write Mask
#define GPIO_DMASK_DMASK13	BIT13				// GPIO Pin13 Data Output Write Mask
#define GPIO_DMASK_DMASK14	BIT14				// GPIO Pin14 Data Output Write Mask
#define GPIO_DMASK_DMASK15	BIT15				// GPIO Pin15 Data Output Write Mask

// GPIO_PIN(GPIO Port Pin Value)	
#define GPIO_PIN_PIN0		BIT0				// GPIO Pin0 Input Value
#define GPIO_PIN_PIN1		BIT1				// GPIO Pin1 Input Value
#define GPIO_PIN_PIN2		BIT2				// GPIO Pin2 Input Value
#define GPIO_PIN_PIN3		BIT3				// GPIO Pin3 Input Value
#define GPIO_PIN_PIN4		BIT4				// GPIO Pin4 Input Value
#define GPIO_PIN_PIN5		BIT5				// GPIO Pin5 Input Value
#define GPIO_PIN_PIN6		BIT6				// GPIO Pin6 Input Value
#define GPIO_PIN_PIN7		BIT7				// GPIO Pin7 Input Value
#define GPIO_PIN_PIN8		BIT8				// GPIO Pin8 Input Value
#define GPIO_PIN_PIN9		BIT9				// GPIO Pin9 Input Value
#define GPIO_PIN_PIN10		BIT10				// GPIO Pin10 Input Value
#define GPIO_PIN_PIN11		BIT11				// GPIO Pin11 Input Value
#define GPIO_PIN_PIN12		BIT12				// GPIO Pin12 Input Value
#define GPIO_PIN_PIN13		BIT13				// GPIO Pin13 Input Value
#define GPIO_PIN_PIN14		BIT14				// GPIO Pin14 Input Value
#define GPIO_PIN_PIN15		BIT15				// GPIO Pin15 Input Value

// GPIO_IMD(GPIO Port Interrupt Mode Control)
#define GPIO_IMD_IMD0		BIT0				// GPIO Pin0 Interrupt Mode Control
#define GPIO_IMD_IMD1		BIT1				// GPIO Pin1 Interrupt Mode Control
#define GPIO_IMD_IMD2		BIT2				// GPIO Pin2 Interrupt Mode Control
#define GPIO_IMD_IMD3		BIT3				// GPIO Pin3 Interrupt Mode Control
#define GPIO_IMD_IMD4		BIT4				// GPIO Pin4 Interrupt Mode Control
#define GPIO_IMD_IMD5		BIT5				// GPIO Pin5 Interrupt Mode Control
#define GPIO_IMD_IMD6		BIT6				// GPIO Pin6 Interrupt Mode Control
#define GPIO_IMD_IMD7		BIT7				// GPIO Pin7 Interrupt Mode Control
#define GPIO_IMD_IMD8		BIT8				// GPIO Pin8 Interrupt Mode Control
#define GPIO_IMD_IMD9		BIT9				// GPIO Pin9 Interrupt Mode Control
#define GPIO_IMD_IMD10		BIT10				// GPIO Pin10 Interrupt Mode Control
#define GPIO_IMD_IMD11		BIT11				// GPIO Pin11 Interrupt Mode Control
#define GPIO_IMD_IMD12		BIT12				// GPIO Pin12 Interrupt Mode Control
#define GPIO_IMD_IMD13		BIT13				// GPIO Pin13 Interrupt Mode Control
#define GPIO_IMD_IMD14		BIT14				// GPIO Pin14 Interrupt Mode Control
#define GPIO_IMD_IMD15		BIT15				// GPIO Pin15 Interrupt Mode Control

// GPIO_IEN(GPIO Port Interrupt Enable)	
#define GPIO_IEN_IF_EN0		BIT0				// GPIO Pin0 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN1		BIT1				// GPIO Pin1 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN2		BIT2				// GPIO Pin2 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN3		BIT3				// GPIO Pin3 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN4		BIT4				// GPIO Pin4 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN5		BIT5				// GPIO Pin5 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN6		BIT6				// GPIO Pin6 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN7		BIT7				// GPIO Pin7 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN8		BIT8				// GPIO Pin8 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN9		BIT9				// GPIO Pin9 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN10	BIT10				// GPIO Pin10 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN11	BIT11				// GPIO Pin11 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN12	BIT12				// GPIO Pin12 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN13	BIT13				// GPIO Pin13 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN14	BIT14				// GPIO Pin14 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IF_EN15	BIT15				// GPIO Pin15 Interrupt Enable by Input Falling Edge or Input Level Low
#define GPIO_IEN_IR_EN0		BIT16				// GPIO Pin0 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN1		BIT17				// GPIO Pin1 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN2		BIT18				// GPIO Pin2 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN3		BIT19				// GPIO Pin3 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN4		BIT20				// GPIO Pin4 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN5		BIT21				// GPIO Pin5 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN6		BIT22				// GPIO Pin6 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN7		BIT23				// GPIO Pin7 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN8		BIT24				// GPIO Pin8 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN9		BIT25				// GPIO Pin9 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN10	BIT26				// GPIO Pin10 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN11	BIT27				// GPIO Pin11 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN12	BIT28				// GPIO Pin12 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN13	BIT29				// GPIO Pin13 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN14	BIT30				// GPIO Pin14 Interrupt Enable by Input Rising Edge or Input Level High
#define GPIO_IEN_IR_EN15	BIT31				// GPIO Pin15 Interrupt Enable by Input Rising Edge or Input Level High

// GPIO_ISRC(GPIO Port Interrupt Source Flag)
#define GPIO_ISRC_ISRC0		BIT0				// GPIO Pin0 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC1		BIT1				// GPIO Pin1 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC2		BIT2				// GPIO Pin2 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC3		BIT3				// GPIO Pin3 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC4		BIT4				// GPIO Pin4 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC5		BIT5				// GPIO Pin5 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC6		BIT6				// GPIO Pin6 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC7		BIT7				// GPIO Pin7 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC8		BIT8				// GPIO Pin8 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC9		BIT9				// GPIO Pin9 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC10	BIT10				// GPIO Pin10 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC11	BIT11				// GPIO Pin11 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC12	BIT12				// GPIO Pin12 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC13	BIT13				// GPIO Pin13 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC14	BIT14				// GPIO Pin14 Interrupt Trigger Source Indicator
#define GPIO_ISRC_ISRC15	BIT15				// GPIO Pin15 Interrupt Trigger Source Indicator

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t PMD0:2;						// GPIO I/O Pin0 Mode Control 
		__IO uint32_t PMD1:2;						// GPIO I/O Pin1 Mode Control 
		__IO uint32_t PMD2:2;						// GPIO I/O Pin2 Mode Control 
		__IO uint32_t PMD3:2;						// GPIO I/O Pin3 Mode Control 
		__IO uint32_t PMD4:2;						// GPIO I/O Pin4 Mode Control 
		__IO uint32_t PMD5:2;						// GPIO I/O Pin5 Mode Control 
		__IO uint32_t PMD6:2;						// GPIO I/O Pin6 Mode Control 
		__IO uint32_t PMD7:2;						// GPIO I/O Pin7 Mode Control 
		__IO uint32_t PMD8:2;						// GPIO I/O Pin8 Mode Control 
		__IO uint32_t PMD9:2;						// GPIO I/O Pin9 Mode Control 
		__IO uint32_t PMD10:2;						// GPIO I/O Pin10 Mode Control 
		__IO uint32_t PMD11:2;						// GPIO I/O Pin11 Mode Control 
		__IO uint32_t PMD12:2;						// GPIO I/O Pin12 Mode Control 
		__IO uint32_t PMD13:2;						// GPIO I/O Pin13 Mode Control 
		__IO uint32_t PMD14:2;						// GPIO I/O Pin14 Mode Control 
		__IO uint32_t PMD15:2;						// GPIO I/O Pin15 Mode Control 
	};
} GPIO_PMD_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__I  uint32_t RESERVE:16;
		__IO uint32_t OFFD0:1;						// GPIO Pin0 OFF digital input path enable
		__IO uint32_t OFFD1:1;						// GPIO Pin1 OFF digital input path enable
		__IO uint32_t OFFD2:1;						// GPIO Pin2 OFF digital input path enable
		__IO uint32_t OFFD3:1;						// GPIO Pin3 OFF digital input path enable
		__IO uint32_t OFFD4:1;						// GPIO Pin4 OFF digital input path enable
		__IO uint32_t OFFD5:1;						// GPIO Pin5 OFF digital input path enable
		__IO uint32_t OFFD6:1;						// GPIO Pin6 OFF digital input path enable
		__IO uint32_t OFFD7:1;						// GPIO Pin7 OFF digital input path enable
		__IO uint32_t OFFD8:1;						// GPIO Pin8 OFF digital input path enable
		__IO uint32_t OFFD9:1;						// GPIO Pin9 OFF digital input path enable
		__IO uint32_t OFFD10:1;						// GPIO Pin10 OFF digital input path enable
		__IO uint32_t OFFD11:1;						// GPIO Pin11 OFF digital input path enable
		__IO uint32_t OFFD12:1;						// GPIO Pin12 OFF digital input path enable
		__IO uint32_t OFFD13:1;						// GPIO Pin13 OFF digital input path enable
		__IO uint32_t OFFD14:1;						// GPIO Pin14 OFF digital input path enable
		__IO uint32_t OFFD15:1;						// GPIO Pin15 OFF digital input path enable
	};
} GPIO_OFFD_T;


typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t DOUT0:1;						// GPIO Pin0 Output Value
		__IO uint32_t DOUT1:1;						// GPIO Pin1 Output Value
		__IO uint32_t DOUT2:1;						// GPIO Pin2 Output Value
		__IO uint32_t DOUT3:1;						// GPIO Pin3 Output Value
		__IO uint32_t DOUT4:1;						// GPIO Pin4 Output Value
		__IO uint32_t DOUT5:1;						// GPIO Pin5 Output Value
		__IO uint32_t DOUT6:1;						// GPIO Pin6 Output Value
		__IO uint32_t DOUT7:1;						// GPIO Pin7 Output Value
		__IO uint32_t DOUT8:1;						// GPIO Pin8 Output Value
		__IO uint32_t DOUT9:1;						// GPIO Pin9 Output Value
		__IO uint32_t DOUT10:1;						// GPIO Pin10 Output Value
		__IO uint32_t DOUT11:1;						// GPIO Pin11 Output Value
		__IO uint32_t DOUT12:1;						// GPIO Pin12 Output Value
		__IO uint32_t DOUT13:1;						// GPIO Pin13 Output Value
		__IO uint32_t DOUT14:1;						// GPIO Pin14 Output Value
		__IO uint32_t DOUT15:1;						// GPIO Pin15 Output Value
		__I  uint32_t RESERVE:16;
	};
} GPIO_DOUT_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t DMASK0:1;						// GPIO Pin0 Data Output Write Mask
		__IO uint32_t DMASK1:1;						// GPIO Pin1 Data Output Write Mask
		__IO uint32_t DMASK2:1;						// GPIO Pin2 Data Output Write Mask
		__IO uint32_t DMASK3:1;						// GPIO Pin3 Data Output Write Mask
		__IO uint32_t DMASK4:1;						// GPIO Pin4 Data Output Write Mask
		__IO uint32_t DMASK5:1;						// GPIO Pin5 Data Output Write Mask
		__IO uint32_t DMASK6:1;						// GPIO Pin6 Data Output Write Mask
		__IO uint32_t DMASK7:1;						// GPIO Pin7 Data Output Write Mask
		__IO uint32_t DMASK8:1;						// GPIO Pin8 Data Output Write Mask
		__IO uint32_t DMASK9:1;						// GPIO Pin9 Data Output Write Mask
		__IO uint32_t DMASK10:1;					// GPIO Pin10 Data Output Write Mask
		__IO uint32_t DMASK11:1;					// GPIO Pin11 Data Output Write Mask
		__IO uint32_t DMASK12:1;					// GPIO Pin12 Data Output Write Mask
		__IO uint32_t DMASK13:1;					// GPIO Pin13 Data Output Write Mask
		__IO uint32_t DMASK14:1;					// GPIO Pin14 Data Output Write Mask
		__IO uint32_t DMASK15:1;					// GPIO Pin15 Data Output Write Mask
		__I  uint32_t RESERVE:16;
	};
} GPIO_DMASK_T;

typedef	union
{
	__I  uint32_t  u32Reg;
	struct
	{
		__I  uint32_t PIN0:1;						// GPIO Pin0 Input Value
		__I  uint32_t PIN1:1;						// GPIO Pin1 Input Value
		__I  uint32_t PIN2:1;						// GPIO Pin2 Input Value
		__I  uint32_t PIN3:1;						// GPIO Pin3 Input Value
		__I  uint32_t PIN4:1;						// GPIO Pin4 Input Value
		__I  uint32_t PIN5:1;						// GPIO Pin5 Input Value
		__I  uint32_t PIN6:1;						// GPIO Pin6 Input Value
		__I  uint32_t PIN7:1;						// GPIO Pin7 Input Value
		__I  uint32_t PIN8:1;						// GPIO Pin8 Input Value
		__I  uint32_t PIN9:1;						// GPIO Pin9 Input Value
		__I  uint32_t PIN10:1;						// GPIO Pin10 Input Value
		__I  uint32_t PIN11:1;						// GPIO Pin11 Input Value
		__I  uint32_t PIN12:1;						// GPIO Pin12 Input Value
		__I  uint32_t PIN13:1;						// GPIO Pin13 Input Value
		__I  uint32_t PIN14:1;						// GPIO Pin14 Input Value
		__I  uint32_t PIN15:1;						// GPIO Pin15 Input Value
		__I  uint32_t RESERVE:16;
	};
}  GPIO_PIN_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t IMD0:1;						// GPIO Pin0 Interrupt Mode Control
		__IO uint32_t IMD1:1;						// GPIO Pin1 Interrupt Mode Control
		__IO uint32_t IMD2:1;						// GPIO Pin2 Interrupt Mode Control
		__IO uint32_t IMD3:1;						// GPIO Pin3 Interrupt Mode Control
		__IO uint32_t IMD4:1;						// GPIO Pin4 Interrupt Mode Control
		__IO uint32_t IMD5:1;						// GPIO Pin5 Interrupt Mode Control
		__IO uint32_t IMD6:1;						// GPIO Pin6 Interrupt Mode Control
		__IO uint32_t IMD7:1;						// GPIO Pin7 Interrupt Mode Control
		__IO uint32_t IMD8:1;						// GPIO Pin8 Interrupt Mode Control
		__IO uint32_t IMD9:1;						// GPIO Pin9 Interrupt Mode Control
		__IO uint32_t IMD10:1;						// GPIO Pin10 Interrupt Mode Control
		__IO uint32_t IMD11:1;						// GPIO Pin11 Interrupt Mode Control
		__IO uint32_t IMD12:1;						// GPIO Pin12 Interrupt Mode Control
		__IO uint32_t IMD13:1;						// GPIO Pin13 Interrupt Mode Control
		__IO uint32_t IMD14:1;						// GPIO Pin14 Interrupt Mode Control
		__IO uint32_t IMD15:1;						// GPIO Pin15 Interrupt Mode Control
		__I  uint32_t RESERVE:16;
	};
} GPIO_IMD_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t IF_EN0:1;						// GPIO Pin0 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN1:1;						// GPIO Pin1 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN2:1;						// GPIO Pin2 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN3:1;						// GPIO Pin3 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN4:1;						// GPIO Pin4 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN5:1;						// GPIO Pin5 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN6:1;						// GPIO Pin6 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN7:1;						// GPIO Pin7 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN8:1;						// GPIO Pin8 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN9:1;						// GPIO Pin9 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN10:1;					// GPIO Pin10 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN11:1;					// GPIO Pin11 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN12:1;					// GPIO Pin12 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN13:1;					// GPIO Pin13 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN14:1;					// GPIO Pin14 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IF_EN15:1;					// GPIO Pin15 Interrupt Enable by Input Falling Edge or Input Level Low
		__IO uint32_t IR_EN0:1;						// GPIO Pin0 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN1:1;						// GPIO Pin1 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN2:1;						// GPIO Pin2 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN3:1;						// GPIO Pin3 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN4:1;						// GPIO Pin4 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN5:1;						// GPIO Pin5 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN6:1;						// GPIO Pin6 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN7:1;						// GPIO Pin7 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN8:1;						// GPIO Pin8 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN9:1;						// GPIO Pin9 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN10:1;					// GPIO Pin10 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN11:1;					// GPIO Pin11 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN12:1;					// GPIO Pin12 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN13:1;					// GPIO Pin13 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN14:1;					// GPIO Pin14 Interrupt Enable by Input Rising Edge or Input Level High
		__IO uint32_t IR_EN15:1;					// GPIO Pin15 Interrupt Enable by Input Rising Edge or Input Level High
	};
} GPIO_IEN_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t ISRC0:1;						// GPIO Pin0 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC1:1;						// GPIO Pin1 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC2:1;						// GPIO Pin2 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC3:1;						// GPIO Pin3 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC4:1;						// GPIO Pin4 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC5:1;						// GPIO Pin5 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC6:1;						// GPIO Pin6 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC7:1;						// GPIO Pin7 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC8:1;						// GPIO Pin8 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC9:1;						// GPIO Pin9 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC10:1;						// GPIO Pin10 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC11:1;						// GPIO Pin11 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC12:1;						// GPIO Pin12 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC13:1;						// GPIO Pin13 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC14:1;						// GPIO Pin14 Interrupt Trigger Source Indicator
		__IO uint32_t ISRC15:1;						// GPIO Pin15 Interrupt Trigger Source Indicator
		__I  uint32_t RESERVE:16;
	};
} GPIO_ISRC_T;

typedef	struct
{
	GPIO_PMD_T		PMD;							// GPIO Port Bit Mode Control
	GPIO_OFFD_T		OFFD;							// GPIO Port Bit OFF Digital enable
	GPIO_DOUT_T		DOUT;							// GPIO Port Data Output Value
	GPIO_DMASK_T	DMASK;							// GPIO Port Data Output Write Mask
	GPIO_PIN_T		PIN;							// GPIO Port Pin Value
	__I uint32_t	RESERVE;
	GPIO_IMD_T		IMD;							// GPIO Port Interrupt Mode Control
	GPIO_IEN_T		IEN;							// GPIO Port Interrupt Enable
	GPIO_ISRC_T		ISRC;							// GPIO Port Interrupt Source Flag
} GPIO_T;

#endif
