#include <string.h>
#include "Driver/DrvGPIO.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvTimer.h"
#include "Driver/DrvSPI.h"
#include "Driver/DrvPWM.h"
#include "Driver/DrvSYS.h"
#include "Motor.h"
#include "Spi.h"
#include "Debug.h"
#include "PWM.h"
#include "Main.h"

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

#define PWM_MODE_TOGGLE 1
#define PWM_MODE_ONESHOT 0

void TestFunc(void)
{
}

void PWM_Init(void)
{	
	UINT16	u16PreScale = 4;
	
	E_DRVPWM_CLOCKDIV	eClockDiv = eDRVPWM_CLOCK_DIV_2;
	
	DrvCLK_SetClkSrcPwm(eDRVCLK_PWMSRC_48M);	
	
	DrvPWM_DisablePwmInverter();		

	// Enable Output for PWM match 0
	DrvPWM_EnablePwmOutput0();
	
	// Enable Output for PWM match 1
	DrvPWM_EnablePwmOutput1();
	
	// Enable Output for PWM match 2
	DrvPWM_EnablePwmOutput2();
	
	
	
	DrvPWM_DisableDeadZone0();
	DrvPWM_DisableDeadZone1(0);
	
							
  DrvPWM_StartPwmTimer(
						PWM_MODE_TOGGLE,			// PWM mode, auto-reload or one shot
						u16PreScale-1,				// prescale
						eClockDiv,						//clock divider
						MOTOR_PWM_PERIOD-1);	// set timer period		

	PWM_set_output0(0);
	PWM_set_output1(0);
	PWM_set_output2(0);
}

void PWM_set_output0(uint16_t Value)
{
	DrvPWM_SetComparatorPwm0(MOTOR_PWM_PERIOD - Value);
}

void PWM_set_output1(uint16_t Value)
{
	DrvPWM_SetComparatorPwm1(MOTOR_PWM_PERIOD - Value);
}

void PWM_set_output2(uint16_t Value)
{
	DrvPWM_SetComparatorPwm2(MOTOR_PWM_PERIOD - Value);
}
