/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved. 		                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __EVB_SETTINGS_H__
#define __EVB_SETTINGS_H__


/************************************************************************************************************
 * Description:
 *	GPIO bit mapping in demo board V1.0
 *
 *
 ************************************************************************************************************/
#if (!defined(__EVB_V1_0__))
//#warning "Select EVB1.0 as the default EVB!. Please define __EVB_V1_0__ to select correct EVB!"
#define __EVB_V1_0__
#endif

#define EVB_VKEY0BIT	BIT6
#define EVB_VKEY1BIT	BIT7
#define EVB_VKEY2BIT	BIT12
#define EVB_VKEY3BIT	BIT13
#define EVB_VKEY4BIT	BIT14
#define EVB_VKEY5BIT	BIT15

	
#define EVB_SWITCH0BIT	BIT8
#define EVB_SWITCH1BIT	BIT9
#define EVB_SWITCH2BIT	BIT10
#define EVB_SWITCH3BIT	BIT11	

#define EVB_LED0BIT	BIT6
#define EVB_LED1BIT	BIT7
#define EVB_LED2BIT	BIT8
#define EVB_LED3BIT	BIT9
#define EVB_LED4BIT	BIT10
#define EVB_LED5BIT	BIT11
	
/************************************************************************************************************
 * Description:
 *	GPIO port mapping in demo board V1.0
 *
 *
 ************************************************************************************************************/
#define EVB_VKEY0PORT	GPIOA
#define EVB_VKEY1PORT	GPIOA
#define EVB_VKEY2PORT	GPIOB
#define EVB_VKEY3PORT	GPIOB
#define EVB_VKEY4PORT	GPIOB
#define EVB_VKEY5PORT	GPIOB

#define EVB_SWITCH0PORT	GPIOA
#define EVB_SWITCH1PORT	GPIOA
#define EVB_SWITCH2PORT	GPIOA
#define EVB_SWITCH3PORT	GPIOA

#define EVB_LED0PORT	GPIOB
#define EVB_LED1PORT	GPIOB
#define EVB_LED2PORT	GPIOB
#define EVB_LED3PORT	GPIOB
#define EVB_LED4PORT	GPIOB
#define EVB_LED5PORT	GPIOB
	
/************************************************************************************************************
 * Description:
 *	GPIO mode mapping in demo board V1.0
 *
 *
 ************************************************************************************************************/
#define EVB_VKEY0MODE_QUASI		DRVGPIO_IOMODE_PIN6_QUASI
#define EVB_VKEY1MODE_QUASI		DRVGPIO_IOMODE_PIN7_QUASI
#define EVB_VKEY2MODE_QUASI		DRVGPIO_IOMODE_PIN12_QUASI
#define EVB_VKEY3MODE_QUASI		DRVGPIO_IOMODE_PIN13_QUASI
#define EVB_VKEY4MODE_QUASI		DRVGPIO_IOMODE_PIN14_QUASI
#define EVB_VKEY5MODE_QUASI		DRVGPIO_IOMODE_PIN15_QUASI

#define EVB_SWITCH0MODE_QUASI	DRVGPIO_IOMODE_PIN8_QUASI
#define EVB_SWITCH1MODE_QUASI	DRVGPIO_IOMODE_PIN9_QUASI
#define EVB_SWITCH2MODE_QUASI	DRVGPIO_IOMODE_PIN10_QUASI
#define EVB_SWITCH3MODE_QUASI	DRVGPIO_IOMODE_PIN11_QUASI

#define EVB_LED0MODE_OUT		DRVGPIO_IOMODE_PIN6_OUT
#define EVB_LED1MODE_OUT		DRVGPIO_IOMODE_PIN7_OUT
#define EVB_LED2MODE_OUT		DRVGPIO_IOMODE_PIN8_OUT
#define EVB_LED3MODE_OUT		DRVGPIO_IOMODE_PIN9_OUT
#define EVB_LED4MODE_OUT		DRVGPIO_IOMODE_PIN10_OUT
#define EVB_LED5MODE_OUT		DRVGPIO_IOMODE_PIN11_OUT

#endif	//#ifndef __EVB_SETTINGS_H__

