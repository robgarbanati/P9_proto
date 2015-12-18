#include <string.h>
#include <stdio.h>
#include "Driver/DrvGPIO.h"
#include "Driver/DrvSPI.h"
#include "Driver/DrvSYS.h"
#include "Spi.h"
#include "Motor.h"
#include "PWM.h"
#include "Main.h"
#include "app_SoftPWM.h"

//
// Global Variables
//
#define SPI_MAX_VALUE	0xFF
#define MOTOR_SPEED_DIVIDER 4  // We have a resolution of 0.25 hz
volatile UINT16 activity_button_pressed_flag = 0;
volatile UINT16 power_down_flag = 0;

//
// Local Defines
//

static UINT8 sway_state, motor_PWM;
static UINT16 motor_freq;

#define STARTUP		    0
#define STARTUP_BOOST	1
#define UP1		        2
#define UP2		        3
#define UP3	            4
#define UP3_BOOST       5
#define PRETIMEOUT      6
#define DOWN3	        7
#define DOWN2	        8
#define DOWN1	        9
#define HOME_BOOST	    10
#define HOME	        11
#define TIMEOUT_STATE	12
#define ONLINE_STATE	13
#define NO_STATE	    14


//
// Global Functions
//

float get_frequency(void)
{
	float return_value = (float) motor_freq * 0.05;
//	printf("mf is %d, gf returns %f\n", motor_freq, return_value);
	
	// Make sure 0 Hz is never returned, because that breaks SineDrive_setMotorMovement.
	if(return_value < 0.50)
		return 0.50;
	else
		return return_value;
}

float get_amplitude_from_state(void)
{
	if(motor_freq == 0)
		return 0.0;
	switch(sway_state)
	{
		case ONLINE_STATE:
			return 0.0;
		case HOME:
		case HOME_BOOST:
		case STARTUP:
		case STARTUP_BOOST:
			return 0.6808;
		case UP1:
		case DOWN1:
			return 0.4357;
		case UP2:
		case DOWN2:
			return 0.2713;
		case UP3:
		case DOWN3:
		case UP3_BOOST:
		case PRETIMEOUT:
			return 0.2100;
		default:
			return 0.0;
	}
}

void set_led_color_from_state(void)
{
	switch(sway_state)
	{
		case ONLINE_STATE:
			RGB_set(RGB_WHITE);
			break;
		case HOME:
		case HOME_BOOST:
		case STARTUP:
		case STARTUP_BOOST:
			RGB_set(RGB_MAGENTA);
			break;
		case UP1:
		case DOWN1:
			RGB_set(RGB_BLUE);	
			break;
		case UP2:
		case DOWN2:
			RGB_set(RGB_GREEN);
			break;
		case UP3:
		case DOWN3:
		case UP3_BOOST:
		case PRETIMEOUT:
			RGB_set(RGB_PINK);
			break;
		default:
			RGB_set(RGB_RED);
			break;
	}
	return;
}


float get_motor_PWM(void)
{
	return ((float) motor_PWM)/100;
}

UINT16 get_safety_clip_flags(void)
{
//	printf("%d %d\n", is_left_safety_clip_in(), is_right_safety_clip_in());
	return (is_left_safety_clip_in() << LEFT_SAFETY_CLIP_FLAG_POSITION) | (is_right_safety_clip_in() << RIGHT_SAFETY_CLIP_FLAG_POSITION);
}

UINT8 get_sway_state(void)
{
	return (UINT8) sway_state;
}

//
// Local Functions
//

void spiSlave_Init(void)
{
	// Open the SPI driver.
	DrvSPI_Open(SPI_SLAVE_HANDLER, SPI_SLAVE_OPEN_FLAGS, SPI_SLAVE_DIVIDER);

	// Configure for slave mode.
	DrvSPI_SPI1_SetSlaveMode(TRUE);

	// Level trigger for slave mode.
	DrvSPI_SPI1_LevelTriggerInSlave(TRUE);

	// Set the zero status byte to shift out.
	DrvSPI_SingleWriteData0(SPI_SLAVE_HANDLER, (UINT32) 0x00);

	// Initiate the SPI transaction.
	DrvSPI_SetGo(SPI_SLAVE_HANDLER);

	// Enable interupt on SPI CS falling.
	DrvGPIO_SetFallingInt(&SPI_SLAVE_GPIO, SPI_SLAVE_CS_PIN, TRUE);
	
	sway_state = NO_STATE;
}

void spiSlave_Write(UINT32 value)
{
	// Set the data to shift out.
	DrvSPI_SingleWriteData0(SPI_SLAVE_HANDLER, (UINT32) value);

	// Initiate the next SPI transaction.
	DrvSPI_SetGo(SPI_SLAVE_HANDLER);
}

// Init master to talk to DRV8301
void spiMaster_Init_Motor(void) {
	// Open the SPI driver.
	DrvSPI_Open(SPI_MASTER_HANDLER, SPI_MASTER_OPEN_FLAGS, SPI_MASTER_DIVIDER);

	// Select the slave.
	DrvSPI_SlaveSelect(SPI_MASTER_HANDLER, TRUE, DRVSPI_IDEL_CLK_LOW);
	DrvSPI_SelectSlave(SPI_MASTER_HANDLER, SPI_MASTER_MOTOR_DEVICE);

	// Read/write data in 16 bit chunks.
	DrvSPI_SetDataConfig(SPI_MASTER_HANDLER, 1, 16);
}

// Init master to talk to cry chip
void spiMaster_Init_Cry(void) {
	// Open the SPI driver.
	DrvSPI_Open(SPI_MASTER_HANDLER, SPI_MASTER_OPEN_FLAGS, SPI_MASTER_DIVIDER);

	// Select the slave.
	DrvSPI_SlaveSelect(SPI_MASTER_HANDLER, TRUE, DRVSPI_IDEL_CLK_LOW);
	DrvSPI_SelectSlave(SPI_MASTER_HANDLER, SPI_MASTER_CRY_DEVICE);

	// Read/write data in 16 bit chunks.
	DrvSPI_SetDataConfig(SPI_MASTER_HANDLER, 1, 16);
}

void spiMaster_Write(UINT32 value)
{
	// Set the data to shift out of the SPI port.
	DrvSPI_SingleWriteData0(SPI_MASTER_HANDLER, (UINT32) value);
	
	// Initiate the SPI transaction.
	DrvSPI_SetGo(SPI_MASTER_HANDLER);
}

void spiMaster_Xchange(UINT16 TxData, UINT16 *RxData)
{
	// Set the data to shift out of the SPI port.
	DrvSPI_SingleWriteData0(SPI_MASTER_HANDLER, (UINT32) TxData);

	// Initiate the SPI transaction.
	DrvSPI_SetGo(SPI_MASTER_HANDLER);

	// Wait while the SPI port is busy
	while (DrvSPI_GetBusy(SPI_MASTER_HANDLER));

	// Read the value shifted in
	*RxData = DrvSPI_SingleReadData0(SPI_MASTER_HANDLER);
}

// Handle the send/receive of the SPI packets at interrupt time.
// On the order of 10 to 100 microseconds
void read_and_write_SPI(void)
{
	UINT8 index = 0;
	UINT16 spiMaster_Data;
	UINT16 spiSlave_Data;
	
	// Assume we receive a zero length packet.
	spiSlave_Data = 0;
	spiMaster_Data = 0;

	// Send/receive bytes until the SPI CS pin is raised.
	while (!DrvGPIO_GetInputPinValue(&SPI_SLAVE_GPIO, SPI_SLAVE_CS_PIN))
	{
		// Wait while the SPI ports are busy and the SPI CS line is low.
		while (DrvSPI_GetBusy(SPI_SLAVE_HANDLER) && !DrvGPIO_GetInputPinValue(&SPI_SLAVE_GPIO, SPI_SLAVE_CS_PIN));  // && DrvSPI_GetBusy(SPI_MASTER_HANDLER)

		// Process the next byte if the SPI CS line is still low.
		if (!DrvGPIO_GetInputPinValue(&SPI_SLAVE_GPIO, SPI_SLAVE_CS_PIN))
		{
			// Read the values shifted in.
			spiSlave_Data = DrvSPI_SingleReadData0(SPI_SLAVE_HANDLER);
			spiMaster_Data = DrvSPI_SingleReadData0(SPI_MASTER_HANDLER);
			
			// Interpret messages
			motor_freq = (spiSlave_Data>>8 & 0x00FF);
			sway_state = (spiSlave_Data>>4 & 0x000F);
			
			if(spiSlave_Data == 0)
				sway_state = NO_STATE;
		}
	}

	// Initialize the first zero status byte to shift out on the next packet.
	spiSlave_Write(spiMaster_Data | activity_button_pressed_flag | power_down_flag | get_safety_clip_flags());
	spiMaster_Write(spiSlave_Data);
	activity_button_pressed_flag = 0;
}

