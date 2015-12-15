#include "Platform.h"
#include <stdlib.h>
#include <Math.h>
#include <stdint.h>

#include "Driver/DrvCLK.h"
#include "Driver/DrvTimer.h"

#include "PWM.h"
#include "app_SinDrive.h"
#include "app_SoftPWM.h"

#define FINE_SINE_STEPS (1024)
#define MAX_AMPLITUDE_FACTOR 5000

#define bool uint8_t
#define false 0
#define true 1

int16_t sine[SINE_STEPS];
int16_t sine_full[SINE_STEPS];
int16_t delta[SINE_STEPS];

int16_t fine_sine[FINE_SINE_STEPS];

int16_t drive_percent = 100;
static uint8_t stepU, stepV, stepW;

float driveAmplitude;
float driveFrequency;
uint32_t intDriveAmplitude;
uint32_t thresholdAmplitude;

int16_t phy_measured_offset;

float t = 16300;	

INT16 amplitudeFactor = 0;
INT16 amplitudeFactorStep;

float amplitudeDriveStep;
float frequencyDriveStep;
//UINT8 amplitudeTransitionDone, frequencyTransitionDone;

UINT8 doAmplitudeTransition = 0;
UINT8 doFrequencyTransition = 0;
enum transition_priority priorityTransition = FREQUENCY;

enum state_machine State = STOPPED;

static void SineDrive_switchState(enum state_machine newState);
static void SineDrive_motorOFF(void);

// Motor movement parameters
float		drvFrequency;
float 	drvPower;
float 	drvAmplitude;
UINT16	drvTransitionTime;
UINT8   drvNewParameters = 0;

// for holding incoming motor movement parameters while motor is transitioning
float  bufferedFrequency;
float  bufferedAmplitude;
float  bufferedPower;
UINT16 bufferedTransitionTime;
UINT8  bufferedMovementWaiting = 0;

static void SineDrive_create_sine_table(void)
{
	float phase;
	uint32_t i;
	
	for (i=0; i<SINE_STEPS; i++)
	{
		phase = (float)((2.00 * PI * i)/SINE_STEPS);
		sine_full[i] = sin(phase) * SINE_MAX;
		sine[i] = sine_full[i];
	}
}

static void SineDrive_create_fine_sine_table(void)
{
	float phase;
	uint32_t i;
	
	for (i=0; i<FINE_SINE_STEPS; i++)
	{
		phase = (float)((2.00 * PI * i)/FINE_SINE_STEPS);
		fine_sine[i] = sin(phase) * 10000;
	}	
}

void SineDrive_init(void)
{
	SineDrive_create_sine_table();
	SineDrive_create_fine_sine_table();
	
	stepU = 0;
	stepV = SINE_STEPS / 3;
	stepW = (SINE_STEPS / 3) * 2;
	
	// *** initialize update timer ***
	
	// Set the timer 0 clock source.
	DrvCLK_SetClkSrcTmr0(eDRVCLK_TIMERSRC_HCLK);
	
	// Configure timer 0 for SineDrive update rate od 1kHz
	DrvTimer_OpenTmr0(DRVTIMER_START_COUNTING | DRVTIMER_ENABLE_INTERRUPT | DRVTIMER_PERIODIC_MODE | 50, (UINT16) SINE_DRIVE_TIMER_1ms);
	
	// Enable timer interrupt.
	DrvTimer_EnableIntTmr0();
	NVIC_EnableIRQ(TMR0_IRQn);
	
	// Enable the timer.
	DrvTimer_EnableTmr0();	
	
	SineDrive_Halt();
}

void SineDrive_stepUp(void)
{
	stepU++;
	stepV++;
	stepW++;
	
	#if SINE_STEPS != 256
	stepU %= SINE_STEPS;
	stepV %= SINE_STEPS;
	stepW %= SINE_STEPS;	
	#endif		
}

void SineDrive_stepDown(void)
{
	stepU--;
	stepV--;
	stepW--;
		
	#if SINE_STEPS != 256
	stepU += SINE_STEPS;
	stepV += SINE_STEPS;
	stepW += SINE_STEPS;
	
	stepU %= SINE_STEPS;
	stepV %= SINE_STEPS;
	stepW %= SINE_STEPS;
	#endif
}

void SineDrive_SineStep(void)
{
	PWM_set_output0(SINE_MAX + sine[stepU]);
	PWM_set_output1(SINE_MAX + sine[stepV]);
	PWM_set_output2(SINE_MAX + sine[stepW]);
}

void SineDrive_setFrequency(float Frequency)
{
	driveFrequency = Frequency;
}

// Amplitude is in the range [0:1], where 1 is one full length oscillation (180deg left, then 180deg right)
void SineDrive_setAmplitude(float Amplitude)
{
	//Amplitude /= 100.00;
	driveAmplitude = Amplitude;
	intDriveAmplitude = (uint32_t)(FULL_TURN * driveAmplitude / 2);
	thresholdAmplitude = (uint32_t)(FULL_TURN * 10.0 / 2);
}

// recalculate values in sine table so that you don't need to do this over and over in real time
void SineDrive_setPower(float Power)
{
	uint32_t i;		
	for (i=0; i<SINE_STEPS; i++)
	{		
		sine[i] = sine_full[i] * Power;
	}
}

void SineDrive_setState(enum state_machine Stat)
{
	State = Stat;	
}

void SineDrive_Start(void)
{
	drvNewParameters = 0;	
	amplitudeFactor = 0;	
	SineDrive_setPower(drvPower); 	
	SineDrive_setFrequency(drvFrequency);
	SineDrive_setAmplitude(drvAmplitude);				
	SineDrive_switchState(ACCELERATE);
}

void SineDrive_Switchover(void)
{
	// activate the transiton code...
	
	if ((frequencyDriveStep >= 0) && (amplitudeDriveStep >= 0))
	{
		// increase frequency, then amplitude
		priorityTransition = FREQUENCY;
	}
	else if ((frequencyDriveStep >= 0) && (amplitudeDriveStep <= 0))
	{
		// drop amplitude, then increase frequency
		priorityTransition = AMPLITUDE;
	}
	else if ((frequencyDriveStep <= 0) && (amplitudeDriveStep >= 0))
	{
		// drop frequency, then increase amplitude
		priorityTransition = FREQUENCY;
	}
	else if ((frequencyDriveStep <= 0) && (amplitudeDriveStep <= 0))
	{
		// drop amplitude, then drop frequency
		priorityTransition = AMPLITUDE;
	}
	
	//amplitudeTransitionDone = 0;
	//frequencyTransitionDone = 0;	
	
	doAmplitudeTransition = 1;
	doFrequencyTransition = 1;
	
	SineDrive_switchState(SWITCHOVER);
}

void SineDrive_Stop(void)
{
	SineDrive_switchState(DECELERATE);	
}

void SineDrive_Halt(void)
{
	SineDrive_motorOFF();
	SineDrive_switchState(STOPPED);
}

void SineDrive_motorOFF(void)
{
	PWM_set_output0(0);
	PWM_set_output1(0);
	PWM_set_output2(0);
}

enum state_machine SineDrive_getState(void)
{
	return State;	
}

static void SineDrive_switchState(enum state_machine newState)
{
	if (newState == STOPPED)
	{		
		amplitudeFactor = 0;
		t = 0;
		State = STOPPED;
	}
	
	if (newState == ACCELERATE)
	{
		amplitudeFactor = 0;
		State = ACCELERATE;	
	}
	
	if (newState == DECELERATE)
	{
		amplitudeFactor = MAX_AMPLITUDE_FACTOR;
		State = DECELERATE;	
	}
	
	if (newState == RUN)
	{
		amplitudeFactor = MAX_AMPLITUDE_FACTOR;
		State = RUN;
	}
	
	if (newState == SWITCHOVER)
	{
		State = SWITCHOVER;
	}
		
}

uint16_t aOffset;	
int16_t  global_phy;
uint8_t tst = 0;
uint32_t old_arg;

void SineDrive_do(void)
{	
	// set correct time step interval according to update timer period!
	// you can check this in SineDrive_init()
	static float    t_step = 1.0 / 1000.0;
	
	static int32_t  step_now, step_old;	
	static int16_t  step_cnt;			
	static uint32_t arg;
	static int16_t  discrete_phy;
	uint32_t i;
	
	if (State == STOPPED)
	{
		// stay here until start command is received
		return;
	}
	
	if (State == ACCELERATE)
	{
		// increase amplitude until set value is reached
		// then switch to RUN state
		amplitudeFactor += amplitudeFactorStep;
		if (amplitudeFactor >= MAX_AMPLITUDE_FACTOR)
		{
			amplitudeFactor = MAX_AMPLITUDE_FACTOR;
			SineDrive_switchState(RUN);
		}
	}
	
	if ( (State == RUN) || (State == ACCELERATE) || (State == DECELERATE) || (State == SWITCHOVER) )
	{
		// RUN until time is up or stop command is received
		// then switch to DECELRATE
		
		// I need to convert sine function argument range [0, PI) into [0, FINE_SINE_STEPS)
		// in order to use table values for sine function, instead of using floating point calculation
		// Also I need to use n:1 conversion for argument angles larger than 2PI, modulo 2PI
		// discrete argument = (FINE_SINE_STEPS)/2PI * float_argument = (FINE_SINE_STEPS)/2PI * 2PI*f*t = FINE_SINE_STEPS*f*t
		// discrete argument %= FINE_SINE_STEPS
		// old function call, very slow
		// phy = sin(2 * PI * Frequency * t);
		arg  = (FINE_SINE_STEPS * driveFrequency * t);
		arg %= FINE_SINE_STEPS;
		discrete_phy = fine_sine[arg];	

		
		global_phy = discrete_phy;
				
		t += t_step;			
		
		step_now = (int32_t)(discrete_phy * intDriveAmplitude + aOffset * intDriveAmplitude)/(10000);
		step_now = step_now * amplitudeFactor / MAX_AMPLITUDE_FACTOR;
		step_cnt = step_now - step_old;		
		step_old = step_now;
		
		// add/remove steps to adjust the offset,
		// but only if motor is already moving, don't affect the steps in which motor is supposed to be still
		if (abs(phy_measured_offset) > 200)
		{
			if ((abs(step_cnt) > 0) && (abs(step_cnt) < 3))
			{
				if (phy_measured_offset<0)
				{
					step_cnt--;
				}
				if (phy_measured_offset>0)
				{
					step_cnt++;
				}
				
				phy_measured_offset = 95 * phy_measured_offset / 100;
			}						
		}
		
		if (step_cnt>0)
		{
			for (i=step_cnt; i>0; i--)
			{				
				SineDrive_stepUp();
				SineDrive_SineStep();
			}
		}
		if (step_cnt<0)
		{
			for (i=-step_cnt; i>0; i--)
			{				
				SineDrive_stepDown();
				SineDrive_SineStep();
			}
		}
	}
	
	if (State == DECELERATE)
	{
		// decrease amplitude until it's 0
		// then switch either to STOP state or start new cycle with pending parameters
		amplitudeFactor -= amplitudeFactorStep;
		if (amplitudeFactor <= 0)
		{		
			amplitudeFactor = 0;
			
			if (drvNewParameters) 
			{
				SineDrive_Start();
			} else {
				SineDrive_switchState(STOPPED);
			}			
		}
	}
	
	if (State == SWITCHOVER)
	{

		// amplitude transition
		if ((doAmplitudeTransition == 1) && (priorityTransition == AMPLITUDE))
		{
			if (amplitudeDriveStep > 0)
			{
				// amplitude up...
				driveAmplitude += amplitudeDriveStep;
				if (driveAmplitude >= drvAmplitude)
				{		
					driveAmplitude = drvAmplitude;
					//amplitudeTransitionDone = 1;
					doAmplitudeTransition = 0;
					priorityTransition = FREQUENCY;
				}
			} else {
				// amplitude down...
				driveAmplitude += amplitudeDriveStep;
				if (driveAmplitude <= drvAmplitude)
				{		
					driveAmplitude = drvAmplitude;
					//amplitudeTransitionDone = 1;
					doAmplitudeTransition = 0;
					priorityTransition = FREQUENCY;
				}
			}
			SineDrive_setAmplitude(driveAmplitude);
		}
		
		// frequency transition
		if ((doFrequencyTransition == 1) && (priorityTransition == FREQUENCY))
		{
			// are we at the swing endpoint! (1/4 sine or 3/4 sine)
			if ( ((old_arg <= 256) && (arg > 256)) || ((old_arg <= 768 ) && (arg > 768)) )
			{
				
				// do the frequency update
				driveFrequency += frequencyDriveStep;
				if (frequencyDriveStep > 0)
				{
						// speeding up...
						if (driveFrequency >= drvFrequency)
						{
							driveFrequency = drvFrequency;
							//frequencyTransitionDone = 1;
							doFrequencyTransition = 0;
							priorityTransition = AMPLITUDE;
						}
				} else {
						// slowing down...
						if (driveFrequency <= drvFrequency)
						{
							driveFrequency = drvFrequency;
							//frequencyTransitionDone = 1;
							doFrequencyTransition = 0;
							priorityTransition = AMPLITUDE;
						}
				}	
				
				// this is where sine functions of different frequencies are seamlessly "sewed" together
				
				//**********************************************************************************************************
				// reset oscillation time to appropriate point on sin function ( Pi/4 or 3Pi/4 )
				if ((old_arg <= 256) && (arg > 256))
				{
					t = arg / (driveFrequency * 1024);
				}
					
				if ((old_arg <= 768) && (arg > 768))
				{
					t = arg / (driveFrequency * 1024);					
				}
											
				// reset arg and setep count variables			
				arg  = (FINE_SINE_STEPS * driveFrequency * t);
				arg %= FINE_SINE_STEPS;
				discrete_phy = fine_sine[arg];
				step_now = (int32_t)(discrete_phy * intDriveAmplitude + aOffset * intDriveAmplitude)/(10000);
				step_now = step_now * amplitudeFactor / MAX_AMPLITUDE_FACTOR;
				step_old = step_now;
				
				// reset any opto corrections
				phy_measured_offset = 0;
				//**********************************************************************************************************			
			}
		}
		
		
		// if all is done...
		//if ((amplitudeTransitionDone == 1) && (frequencyTransitionDone == 1))
		if ((doAmplitudeTransition == 0) && (doFrequencyTransition == 0))
		{
			SineDrive_switchState(RUN);
			
			if (bufferedMovementWaiting != 0)
			{
				// we have received a transition(s) while transitioning, go handle that...
				bufferedMovementWaiting = 0;
				SineDrive_setMotorMovement(bufferedFrequency, bufferedAmplitude, bufferedPower, bufferedTransitionTime);
			}
		}
	}
	
	// always keep track of arg value, it's needed for the smooth transition
	old_arg = arg;
}

/*
  Frequency in Hz, float
	Amplitude in %, [0.00 - 1.00]
	Power, [0.00 - 1.00]
	TransitionTime in ms - max 5000!
*/
void SineDrive_setMotorMovement(float Frequency, float Amplitude, float Power, UINT16 TransitionTime)
{
	float TransitionSteps;
	
	// prevent midtransition movement changes
	if (SineDrive_getState() == SWITCHOVER)
	{
		// save this request for later, and just jump out
		// if multiple requests received, only the last one is saved and aplied after the current switchower
		
		bufferedFrequency      = Frequency;
		bufferedAmplitude      = Amplitude;
		bufferedPower          = Power;
		bufferedTransitionTime = TransitionTime;
		bufferedMovementWaiting  = 1;
		
		return;
	}
	
	drvFrequency = 			Frequency;
	drvPower = 					Power;
	drvAmplitude = 			Amplitude;
	drvTransitionTime = TransitionTime;
	drvNewParameters = 	1;
	
	amplitudeFactorStep = (MAX_AMPLITUDE_FACTOR / TransitionTime);
	
	// amplitude can be modified each ms, that is at each PWM update interrupt
	// to make the change in required time, find out by how much apmplitude must be changed at each update
	amplitudeDriveStep = ( (drvAmplitude -  driveAmplitude) / TransitionTime);
	
	// frequency is changed at each reversal of direction, that's two times per oscillation
	// total number of reversals is (TransitionTime/frequency) * 2
	// transition time is in ms, frequency in Hz
  // there are different ways to select which frequency to use in calculation
	// it can be the new value, the one at which motor is already moving or maybe an average of those two...
	// It looks like that current value is best suited, it gives nice smooth change
	// which ever the option is used, time for frequency change is not exact.
	// The number of half-oscillations in this process is not exact, it's based on startign frequency, therefore f. change time is not exact.
	TransitionSteps = ((2.0 * TransitionTime) / ( driveFrequency * 1000.0));	
	frequencyDriveStep = (drvFrequency - driveFrequency) / TransitionSteps;
	
	// Should we stop?
	if (drvFrequency == 0)
	{
		if (State != STOPPED) {
			SineDrive_Stop();
		}
		return;
	}
	
	// Should we start?
	if (State == STOPPED)
	{
		SineDrive_Start();
		return;
	}
	
	// OK, motion change is needed...		
	SineDrive_Switchover();
	
}
