#include <stdio.h>
#include <string.h>
#include "Driver/DrvGPIO.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvTimer.h"
#include "Driver/DrvSPI.h"
#include "Driver/DrvPWM.h"
#include "Driver/DrvSYS.h"
#include "Motor.h"
#include "Main.h"
#include "PWM.h"
#include "Spi.h"


//
// Global Variables
//



//
// Local Defines
//

//
// Local Functions
//


//
// Global Functions
//

void Start_Timer0(void)
{
	DrvTimer_OpenTmr0(DRVTIMER_START_COUNTING | DRVTIMER_ENABLE_INTERRUPT | DRVTIMER_PERIODIC_MODE | 50, (UINT16) MOTOR_TIMER_1ms);
}

void Motor_Init(void)
{
	// Enable interrupt on Homing_Sensor falling or rising.
//	DrvGPIO_SetFallingInt(&HOMING_SENSOR_GPIO, HOMING_SENSOR_PIN, TRUE);
//	DrvGPIO_SetRisingInt(&HOMING_SENSOR_GPIO, HOMING_SENSOR_PIN, TRUE);
	
	// Set the timer 0 clock source.
	DrvCLK_SetClkSrcTmr0(eDRVCLK_TIMERSRC_HCLK);
	
	// Configure timer 0 with a compare compare value for 1 second .
	Start_Timer0();
	
	// Enable timer interrupt.
	DrvTimer_EnableIntTmr0();
	NVIC_EnableIRQ(TMR0_IRQn);
	
	// Enable the timer.
	DrvTimer_EnableTmr0();	
}

