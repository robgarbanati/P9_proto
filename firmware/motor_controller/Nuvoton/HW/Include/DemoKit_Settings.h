/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __DEMOKIT_SETTINGS_H__
#define __DEMOKIT_SETTINGS_H__


/************************************************************************************************************
 * Description:
 *	GPIO bit mapping in demo board V1.1 and V1.2 
 *
 *
 ************************************************************************************************************/
#ifdef __DEMOKIT_V1_1
	#define DEMOKIT_VKEY0BIT	BIT14
	#define DEMOKIT_VKEY1BIT	BIT15
#elif defined __DEMOKIT_V1_2
	#define DEMOKIT_VKEY0BIT	BIT6
	#define DEMOKIT_VKEY1BIT	BIT7
#endif	//#if   __DEMOKIT_V1_1

	/* GPB 12~ 15 */
	#define DEMOKIT_VKEY2BIT	BIT12
	#define DEMOKIT_VKEY3BIT	BIT13
	#define DEMOKIT_VKEY4BIT	BIT14
	#define DEMOKIT_VKEY5BIT	BIT15

		
	#define DEMOKIT_SWITCH0BIT	BIT8
	#define DEMOKIT_SWITCH1BIT	BIT9
	#define DEMOKIT_SWITCH2BIT	BIT10
	#define DEMOKIT_SWITCH3BIT	BIT11	

	
	#define DEMOKIT_LED0BIT	BIT6
	#define DEMOKIT_LED1BIT	BIT7
	#define DEMOKIT_LED2BIT	BIT8
	#define DEMOKIT_LED3BIT	BIT9
	#define DEMOKIT_LED4BIT	BIT10
	#define DEMOKIT_LED5BIT	BIT11
	
#ifdef __DEMOKIT_V1_2
	#define DEMOKIT_EXLDOBITS_ENABLE		(BIT15)
	#define DEMOKIT_EXLDOBITS_DISABLE		(BIT14|BIT15)
#endif
	
/************************************************************************************************************
 * Description:
 *	GPIO port mapping in demo board V1.1 and V1.2 
 *
 *
 ************************************************************************************************************/
	#define DEMOKIT_VKEY0PORT	GPIOA
	#define DEMOKIT_VKEY1PORT	GPIOA
	#define DEMOKIT_VKEY2PORT	GPIOB
	#define DEMOKIT_VKEY3PORT	GPIOB
	#define DEMOKIT_VKEY4PORT	GPIOB
	#define DEMOKIT_VKEY5PORT	GPIOB

		
	#define DEMOKIT_SWITCH0PORT	GPIOA
	#define DEMOKIT_SWITCH1PORT	GPIOA
	#define DEMOKIT_SWITCH2PORT	GPIOA
	#define DEMOKIT_SWITCH3PORT	GPIOA

		
#ifdef  __DEMOKIT_V1_1
	#define DEMOKIT_LED0PORT	GPIOA
	#define DEMOKIT_LED1PORT	GPIOA			
#elif defined __DEMOKIT_V1_2
	#define DEMOKIT_LED0PORT	GPIOB
	#define DEMOKIT_LED1PORT	GPIOB
#endif	//#if   __DEMOKIT_V1_1
	
	#define DEMOKIT_LED2PORT	GPIOB
	#define DEMOKIT_LED3PORT	GPIOB
	#define DEMOKIT_LED4PORT	GPIOB
	#define DEMOKIT_LED5PORT	GPIOB
	
/************************************************************************************************************
 * Description:
 *	GPIO mode mapping in demo board V1.1 and V1.2 
 *
 *
 ************************************************************************************************************/

#ifdef  __DEMOKIT_V1_1
	#define DEMOKIT_VKEY0MODE_QUASI	DRVGPIO_IOMODE_PIN14_QUASI
	#define DEMOKIT_VKEY1MODE_QUASI	DRVGPIO_IOMODE_PIN15_QUASI
#elif defined __DEMOKIT_V1_2
	#define DEMOKIT_VKEY0MODE_QUASI	DRVGPIO_IOMODE_PIN6_QUASI
	#define DEMOKIT_VKEY1MODE_QUASI	DRVGPIO_IOMODE_PIN7_QUASI
#endif //#if   __DEMOKIT_V1_1

#define DEMOKIT_VKEY2MODE_QUASI	DRVGPIO_IOMODE_PIN12_QUASI
#define DEMOKIT_VKEY3MODE_QUASI	DRVGPIO_IOMODE_PIN13_QUASI
#define DEMOKIT_VKEY4MODE_QUASI	DRVGPIO_IOMODE_PIN14_QUASI
#define DEMOKIT_VKEY5MODE_QUASI	DRVGPIO_IOMODE_PIN15_QUASI

#define DEMOKIT_SWITCH0MODE_QUASI	DRVGPIO_IOMODE_PIN8_QUASI
#define DEMOKIT_SWITCH1MODE_QUASI	DRVGPIO_IOMODE_PIN9_QUASI
#define DEMOKIT_SWITCH2MODE_QUASI	DRVGPIO_IOMODE_PIN10_QUASI
#define DEMOKIT_SWITCH3MODE_QUASI	DRVGPIO_IOMODE_PIN11_QUASI


#define DEMOKIT_LED0MODE_OUT	DRVGPIO_IOMODE_PIN6_OUT
#define DEMOKIT_LED1MODE_OUT	DRVGPIO_IOMODE_PIN7_OUT
#define DEMOKIT_LED2MODE_OUT	DRVGPIO_IOMODE_PIN8_OUT
#define DEMOKIT_LED3MODE_OUT	DRVGPIO_IOMODE_PIN9_OUT
#define DEMOKIT_LED4MODE_OUT	DRVGPIO_IOMODE_PIN10_OUT
#define DEMOKIT_LED5MODE_OUT	DRVGPIO_IOMODE_PIN11_OUT

#ifdef  __DEMOKIT_V1_2
	#define DEMOKIT_EXLDOMODE_ENABLE	(DRVGPIO_IOMODE_PIN14_IN|DRVGPIO_IOMODE_PIN15_OUT)
	#define DEMOKIT_EXLDOMODE_DISABLE	(DRVGPIO_IOMODE_PIN14_OUT|DRVGPIO_IOMODE_PIN15_OUT)
#endif

#endif	//#ifndef __DEMOKIT_SETTINGS_H__

