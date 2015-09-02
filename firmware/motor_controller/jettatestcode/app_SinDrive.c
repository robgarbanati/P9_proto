#include "Platform.h"
#include <stdlib.h>
#include <Math.h>
#include <stdint.h>

#include "Driver/DrvCLK.h"
#include "Driver/DrvTimer.h"

#include "PWM.h"
#include "app_SinDrive.h"

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

float driveFrequency;
uint32_t driveAmplitude;
uint32_t thresholdAmplitude;

float t = 16300;	
float phaseStartTime, phaseStopTime;
float indexStartTime, indexStopTime;
float phaseTime, indexTime;
bool  phaseImpulseStarted = false;
bool  phaseImpulseDone = false;
bool  indexImpulseDone = false;
bool  offsetFound = false;
float timeOffset = 0;
//float direction = 1;

uint16_t amplitudeFactor = MAX_AMPLITUDE_FACTOR;

enum state_machine State = STOPPED;

static void SineDrive_switchState(enum state_machine newState);
static void SineDrive_motorOFF(void);

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

static void SineDrive_create_delta_table(void)
{
	float phase;
	float one_third = (2 * PI) / 3;
	uint32_t i;
	
	for (i=0; i<SINE_STEPS; i++)
	{
		phase = (float)((2.00 * PI * i)/SINE_STEPS);		
		if ((phase >= 0) && (phase < one_third))
		{
			delta[i] = sin(phase) * MOTOR_PWM_PERIOD;
		}
		if ((phase >= one_third) && (phase < 2*one_third))
		{
			delta[i] = -sin(phase - 2*one_third) * MOTOR_PWM_PERIOD;
		}
		if (phase >= 2*one_third)
		{
			delta[i] = 0;
		}
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
	SineDrive_create_delta_table();
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

void SineDrive_DeltaStep(void)
{
	PWM_set_output0(delta[stepU]);
	PWM_set_output1(delta[stepV]);
	PWM_set_output2(delta[stepW]);
}

void SineDrive_setFrequency(float Frequency)
{
	driveFrequency = Frequency;
}

// Amplitude is in the range [0:1], where 1 is one full length oscillation (180deg left, then 180deg right)
void SineDrive_setAmplitude(float Amplitude)
{
	Amplitude /= 100.00;
	driveAmplitude = (uint32_t)(FULL_TURN * Amplitude / 2);
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
	SineDrive_switchState(ACCELERATE);
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
		//SineDrive_motorOFF();
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
		
}

uint16_t aOffset;	
void SineDrive_do(void)
{	
	// set correct time step interval according to update timer period!
	// you can check this in SineDrive_init()
	static float    t_step = 1.0 / 1000.0;
	
	static int32_t  step_now, step_old;	
	static int16_t  step_cnt;			
	static uint32_t arg, last_arg;
	static int16_t  discrete_phy;
	static float    partOffset;
	//static float    angularOffset;
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
		amplitudeFactor++;
		if (amplitudeFactor >= MAX_AMPLITUDE_FACTOR)
		{
			SineDrive_switchState(RUN);
		}
	}
	
	if ( (State == RUN) || (State == ACCELERATE) || (State == DECELERATE) )
	{
		// RUN until time is up or stop command is received
		// then switch to DECELERATE
		
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
		
		// detect change in direction, and reset offset measuring
		// changes occur at pi/2 and 3pi/2, the sine function
		// in this case sine is approximated with fine_sine table with FINE_SINE_STEPS
		// therefore changes occur at FINE_SINE_STEPS/4 and 3*FINE_SINE_STEPS/4
		//port_pin_set_output_level(PHASE_0_PIN, HIGH);
		if ((last_arg <= FINE_SINE_STEPS/4) && (arg > FINE_SINE_STEPS/4))
		{
			// direction change!!!
			SineDrive_resetOffsetMeasuring();
			//direction = -1;
		}
		if ((last_arg <= 3*FINE_SINE_STEPS/4) && (arg > 3*FINE_SINE_STEPS/4))
		{
			// direction change!!!
			SineDrive_resetOffsetMeasuring();
			//direction = 1;
		}
		last_arg = arg;
				
		
		
		// detect phase close to 0
		if (abs(discrete_phy) < 500)
		//if ((arg > FINE_SINE_STEPS/4) && (arg < FINE_SINE_STEPS * 3 / 4))
		{
			if (!phaseImpulseStarted)
			{
				phaseImpulseStarted = true;
				//////////////////////////////////////////////////port_pin_set_output_level(PHASE_0_PIN, HIGH);
				SineDrive_getPhaseStartTime();
			}			
		} else {
			if (phaseImpulseStarted)
			{
				phaseImpulseStarted = false;
				//////////////////////////////////////////////////port_pin_set_output_level(PHASE_0_PIN, LOW);
				SineDrive_getPhaseStopTime();
			}			
		}
		
		t += t_step;			
		
		partOffset = timeOffset/100;
		timeOffset -= partOffset;

		step_now = (int32_t)(discrete_phy * driveAmplitude + aOffset * driveAmplitude)/(10000);
		step_now = step_now * amplitudeFactor / MAX_AMPLITUDE_FACTOR;
		step_cnt = step_now - step_old;
		step_old = step_now;
		
		if (partOffset > t_step/20)
		{
			if (step_cnt<0)
			{
				step_cnt--;
			}	
			if (step_cnt>0)
			{
				step_cnt++;
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
		// then switch to STOP state
		amplitudeFactor--;
		if (amplitudeFactor == 0)
		{			
			SineDrive_switchState(STOPPED);
		}
	}
}

void SineDrive_getPhaseStartTime(void)
{
	phaseStartTime = t;
}

void SineDrive_getPhaseStopTime(void)
{
	phaseStopTime = t;
	phaseTime = (phaseStartTime + phaseStopTime) / 2;	
	
	if (!offsetFound)
	{
		phaseImpulseDone = true;
	}	
			
}

void SineDrive_getIndexStartTime(void)
{		
	indexStartTime = t;			
}

void SineDrive_getIndexStopTime(void)
{
	indexStopTime = t;
	indexTime = (indexStartTime + indexStopTime) / 2;
	
	if (!offsetFound)
	{
		indexImpulseDone = true;
		if (phaseImpulseDone)
		{
			SineDrive_calcTimeDrift();
		}
	}	
}

void SineDrive_calcTimeDrift(void)
{
	timeOffset = indexTime - phaseTime;
	offsetFound = true;
	//printf("Offset t: %d\r\n", (uint32_t)(timeOffset*1000));
}

void SineDrive_resetOffsetMeasuring(void)
{
	offsetFound = false;
	indexImpulseDone = false;
	phaseImpulseDone = false;
	timeOffset = 0;
}
