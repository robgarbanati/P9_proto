#include "Platform.h"
#include "Driver/DrvGPIO.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvTimer.h"
#include "app_SoftPWM.h"

uint16_t	RGB_pwm_cnt = 0;
uint8_t		RGB_R_pwm 	= 0;
uint8_t		RGB_G_pwm 	= 0;
uint8_t		RGB_B_pwm 	= 0;

void RGB_init(void)
{
	DrvCLK_SetClkSrcTmr2(eDRVCLK_TIMERSRC_48M);
	
	// Configure timer 2 for RGB at ~50Hz -> 50Hz * 64 = 3200Hz
	DrvTimer_OpenTmr2(DRVTIMER_START_COUNTING | DRVTIMER_ENABLE_INTERRUPT | DRVTIMER_PERIODIC_MODE | 20, 700);
	
	// Enable timer interrupt.
	DrvTimer_EnableIntTmr2();
	NVIC_EnableIRQ(TMR2_IRQn);
	
	// Enable the timer.
	DrvTimer_EnableTmr2();
}

void RGB_handle(void)
{
	if (RGB_pwm_cnt >= LED_PWM_PERIOD)
	{
		RGB_pwm_cnt = 0;
	}
	
	if (RGB_pwm_cnt == 0)
	{
		DrvGPIO_ClearOutputBit(&GPIOA, DRVGPIO_PIN_9);	
		DrvGPIO_ClearOutputBit(&GPIOA, DRVGPIO_PIN_10);	
		DrvGPIO_ClearOutputBit(&GPIOA, DRVGPIO_PIN_11);	
	}
	
	if (RGB_pwm_cnt >= RGB_R_pwm)
	{
		DrvGPIO_SetOutputBit(&GPIOA, DRVGPIO_PIN_9);
	}	
	if (RGB_pwm_cnt >= RGB_G_pwm)
	{
		DrvGPIO_SetOutputBit(&GPIOA, DRVGPIO_PIN_10);
	}
	if (RGB_pwm_cnt >= RGB_B_pwm)
	{
		DrvGPIO_SetOutputBit(&GPIOA, DRVGPIO_PIN_11);
	}
	
	RGB_pwm_cnt++;
}

void RGB_set(uint8_t R, uint8_t G, uint8_t B)
{
	RGB_R_pwm = R;
	RGB_G_pwm = G;
	RGB_B_pwm = B;
}
