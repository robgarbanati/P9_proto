#include <stdio.h>
#include <string.h>
#include "Platform.h"
#include "Driver/DrvGPIO.h"
#include "Driver/DrvSYS.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvSPI.h"
#include "Driver/DrvPWM.h"
#include "Driver/DrvTimer.h"
#include "Adc.h"
#include "SysClkConfig.h"
#include "Main.h"
#include "Spi.h"
#include "PWM.h"
#include "app_SinDrive.h"
#include "app_drv8301.h"
#include "app_SoftPWM.h"
#include "app_OptoSensor.h"

#define ACTIVITY_BUTTON_SPI_SHIFT_AMOUNT	7
#define HIGH_STATE	1
#define LOW_STATE	0


#include <stdio.h>	  
#define  SMPLPWM_TESTCAP 0
 
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
						 
UINT16	g_u16Frequency;
UINT32	g_bCapInt = 0;

UINT32	g_u32CMR,g_u32CNR;
BOOL	g_bLEDH=TRUE,g_LED_S=FALSE;


volatile UINT32	blue_duty_cycle, green_duty_cycle, red_duty_cycle;
extern volatile int current_sensed;

volatile UINT32 TO_counter = 0;
volatile UINT8 speed_flag;
UINT8 shifted_activity_button = 0;
extern UINT8 activity_button_pressed_flag;
static UINT8 activity_button_is_being_pressed;

volatile UINT16 Homing_pin_state;

int is_left_safety_clip_in(void)
{
	return DrvGPIO_GetInputPinValue(&SAFETY_CLIP_GPIO, LEFT_SAFETY_CLIP) >> 15;
}

int is_right_safety_clip_in(void)
{
	return DrvGPIO_GetInputPinValue(&SAFETY_CLIP_GPIO, RIGHT_SAFETY_CLIP) >> 14;
}

int get_activity_button_state(void)
{
	return activity_button_is_being_pressed;
}

void GPAB_IRQHandler(void)
{	
	// Is this from SPI CS?
	if(DrvGPIO_GetIntFlag(&SPI_SLAVE_GPIO, SPI_SLAVE_CS_PIN))
	{
		// Handle send/receive of packets from N3290 (State Machine Chip) and Cry Detect Board.
		read_and_write_SPI();

		// Clear the SPI CS interrupt.
		DrvGPIO_ClearIntFlag(&SPI_SLAVE_GPIO, SPI_SLAVE_CS_PIN);
	}
	
	// Is this from Activity Button?
	if(DrvGPIO_GetIntFlag(&ACTIVITY_BUTTON_GPIO, ACTIVITY_BUTTON_PIN))
	{
		activity_button_is_being_pressed = 1;

		// Only continue if pin is high.
		DrvGPIO_ClearIntFlag(&ACTIVITY_BUTTON_GPIO, ACTIVITY_BUTTON_PIN);
		if (DrvGPIO_GetInputPinValue(&ACTIVITY_BUTTON_GPIO, ACTIVITY_BUTTON_PIN))
		{
			activity_button_is_being_pressed = 0;
			activity_button_pressed_flag = 1 << ACTIVITY_BUTTON_SPI_SHIFT_AMOUNT;
			
			// Clear the activity button interrupt.
			DrvGPIO_ClearIntFlag(&ACTIVITY_BUTTON_GPIO, ACTIVITY_BUTTON_PIN);
		}
	}	
	
	// Is this from homing sensor?
	if(DrvGPIO_GetIntFlag(&HOMING_SENSOR_GPIO, HOMING_SENSOR_PIN))
	{
		// grab homing sensor state and process it...
		// returned state is not 0 or 1, it's actually 0 or shifted value (1 << PIN_POS) to reflect the actuall pin!!!
		Homing_pin_state = DrvGPIO_GetInputPinValue(&HOMING_SENSOR_GPIO, HOMING_SENSOR_PIN);
		
		if (Homing_pin_state) {
			// TEST - visualizing opto. functino
			//DrvGPIO_ClearOutputBit(&GPIOB, DRVGPIO_PIN_6);
			OptoSensor_process_transition(HIGH);
		} else {
			// TEST - visualizing opto. functino
			//DrvGPIO_SetOutputBit(&GPIOB, DRVGPIO_PIN_6);
			OptoSensor_process_transition(LOW);
		}
			
		// Clear the homing interrupt.
		DrvGPIO_ClearIntFlag(&HOMING_SENSOR_GPIO, HOMING_SENSOR_PIN);
	}
}

uint8_t Update = 0;

// Motor driver timer
void TMR0_IRQHandler(void)
{
	// Clear the timer interrupt flag.
	DrvTimer_ClearIntFlagTmr0();
		
	Update = 1;
	
	// TEST - for measuring processor load
	//DrvGPIO_SetOutputBit(&GPIOB, DRVGPIO_PIN_11);
	
	// Increment real-time counter
	TO_counter++;
}


// Soft PWM timer
void TMR2_IRQHandler(void)
{
	// Clear the timer interrupt flag.
	DrvTimer_ClearIntFlagTmr2();
	
	RGB_handle();
}

// Initialize interrupt priorities.
void priorityInit(void)
{
	// Set the SPI slave interrupt priority high.
	NVIC_SetPriority(SPI1_IRQn, 0);

	// Set the GPIO interrupt priority high.
	NVIC_SetPriority(GPAB_IRQn, 0);

	// Set the ADC interrupt lower than SPI and GPIO.
	NVIC_SetPriority(ADC_IRQn, 1);
	
	//********** What is going on here?*********
	NVIC_SetPriority(TMR0_IRQn, 3);
	NVIC_SetPriority(TMR2_IRQn, 1);
}

// Initialize system clock.
void clkInit(void)
{
// 	UINT32 HCLK;

	// Unlock protected registers.
	DrvSYS_UnlockKeyReg();

	// Configure the system clock source.  See SysClkConfig.h.
	_SYSCLKCONFIG();
	
	// Set HCLK divider
	DrvCLK_SetClkDividerHclk(15);
	
// 	HCLK = DrvCLK_GetHclk();
//	printf("HCLK is %d.\n",HCLK);

	// Enable LDO33.
	DrvCLK_EnableLDO30_P();
	
	init_ADC_clock();
	
	DrvPWM_Open();

	// Disable low voltage reset
	DrvSYS_DisableLowVoltageReset_P();
	
	// Enable power on reset (assert reset when power first comes on).
	DrvSYS_ClearPORDisableCode_P();

	// Lock protected registers.
	DrvSYS_LockKeyReg();
}

void gpioInit(void)
{
	// Configure GPIO A special functions.
	DrvSYS_EnableMultifunctionGpioa(
		DRVSYS_GPIOA_MF1_SPI0_1ST_CHIP_SEL_OUT |
		DRVSYS_GPIOA_MF2_SPI0_CLOCK_OUT |
		DRVSYS_GPIOA_MF3_SPI0_DATA_IN |
		DRVSYS_GPIOA_MF4_SPI0_DATA_OUT |
		DRVSYS_GPIOA_MF8_ADC_CHANNEL0_IN  	// Current sensor
	);

	// Configure GPIO B special functions.
	DrvSYS_EnableMultifunctionGpiob(
		DRVSYS_GPIOB_MF1_SPI1_1ST_CHIP_SEL_OUT	|	// Slave SPI select input
		DRVSYS_GPIOB_MF2_SPI1_CLOCK_OUT 		|	// Slave SPI clock input
		DRVSYS_GPIOB_MF3_SPI1_DATA_IN 			|	// Slave SPI data output
		DRVSYS_GPIOB_MF4_SPI1_DATA_OUT 			|	// Slave SPI data input
		DRVSYS_GPIOB_MF8_PWM_OUT_0		|	  // PWM Motor
		DRVSYS_GPIOB_MF9_PWM_OUT_1		|	  // old PWM Blue LED
		DRVSYS_GPIOB_MF10_PWM_OUT_2		//|	// old PWM Green LED
		//DRVSYS_GPIOB_MF11_PWM_OUT_3			// old PWM Red LED
	);
	
	// Configure GPIO port A pins.
	DrvGPIO_SetIOMode(&GPIOA,
		DRVGPIO_IOMODE_PIN1_IN		|	// Slave SPI select input
		
		DRVGPIO_IOMODE_PIN10_OUT   | // softPWM LED R
		DRVGPIO_IOMODE_PIN11_OUT   | // softPWM LED G
		DRVGPIO_IOMODE_PIN12_OUT     // softPWM LED B
	);
	
	DrvGPIO_EnableInputPin(&GPIOB,
		DRVGPIO_PIN_12 |
		DRVGPIO_PIN_13 |
		DRVGPIO_PIN_14 |
		DRVGPIO_PIN_15
	);

	// Configure GPIO port B pins.
	DrvGPIO_SetIOMode(&GPIOB,
	
		DRVGPIO_IOMODE_PIN6_OUT		|
	
		DRVGPIO_IOMODE_PIN8_OUT		|	// PWM Motor
		DRVGPIO_IOMODE_PIN9_OUT		|	// PWM Motor
		DRVGPIO_IOMODE_PIN10_OUT	|	// PWM Motor
		DRVGPIO_IOMODE_PIN11_OUT	|	// busy LED			
		
		DRVGPIO_IOMODE_PIN12_IN		|	// Read activity_Button
		DRVGPIO_IOMODE_PIN13_IN		|	// Read Homing_Sensor
		DRVGPIO_IOMODE_PIN14_IN		|	// Read Safety_Clip_R
		DRVGPIO_IOMODE_PIN15_IN			// Read Safety_Clip_L
	);

//	// Enable interupt on activity button released.
//	DrvGPIO_SetRisingInt(&ACTIVITY_BUTTON_GPIO, ACTIVITY_BUTTON_PIN, TRUE);
//	DrvGPIO_EnableFallingLowInt(&ACTIVITY_BUTTON_GPIO, ACTIVITY_BUTTON_PIN);

	// Enable interupt on interrupter change
	DrvGPIO_SetRisingInt(&HOMING_SENSOR_GPIO, HOMING_SENSOR_PIN, TRUE);
	DrvGPIO_SetFallingInt(&HOMING_SENSOR_GPIO, HOMING_SENSOR_PIN, TRUE);

	// Enable GPIO interrupt routine.
	NVIC_EnableIRQ(GPAB_IRQn);    
}

uint16_t res;
int main(void)
{

	
	clkInit();

	gpioInit();

//	init_ADC();
	
//	spiSlave_Init();
	spiMaster_Init();
	
	
	// motor driver IC initialization
	init_DRV8301();	
	// motor PWM module initialization
	PWM_Init();		
	
	// test
	//PWM_set_output0(16);
	//while(1);
	
	
	// BLDC motor control initialization
	SineDrive_init();	
	
	// software PWM for RGB initializaiton
	RGB_init(); // place it after sindrive init so it doesn't slow down initial calculation
	RGB_set(RGB_YELLOW);
	
	// DEMO - starting motor movement
	SineDrive_setMotorMovement(2.0, 0.25, 0.70, 3000);
	
	for (;;)
	{
		// DEMO - timer interrupt sets this Update flag
		//        when new update is up, just call the SineDrive_do() function...
		//        SineDrive_do() takes care of everything, accelerating, decellerating, swithing states, moving motor, sinchronizing, etc...
		if (Update) {
			Update = 0;
			SineDrive_do();	
			
			// TEST - for measuring processor load
			//DrvGPIO_ClearOutputBit(&GPIOB, DRVGPIO_PIN_11);			
		}
		
		// DEMO - at 10th second since restart, shange the motor movement
		//        observe the gracefull change by first reducing oscillation amplitude, then restarting
		//        with new movement by gradualy increasing amplitude
		if (TO_counter == 10000)
		{
				//SineDrive_setMotorMovement(2.0, 0.25, 0.70, 3000);
			SineDrive_setMotorMovement(0.25, 1.0, 0.70, 3000);
		}
		
		// DEMO - same here, just starting drom 20th second...
		if (TO_counter == 20000)
		{
				//SineDrive_setMotorMovement(2.0, 0.25, 0.70, 3000);
			SineDrive_setMotorMovement(3.5, 0.25, 0.70, 3000);
		}

	}
}
