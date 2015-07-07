/*
 * SineDrive.h
 *
 * Created: 1/27/2015 7:27:10 PM
 *  Author: Marko
 */ 
 
#include "Platform.h"

#ifndef SINEDRIVE_H_
#define SINEDRIVE_H_

#define SINE_STEPS 256
#define SINE_MAX   (MOTOR_PWM_PERIOD/2)
#define PI		   3.14159265358979f
#define FULL_TURN  (SINE_STEPS * 7)

#define SINE_DRIVE_TIMER_10ms		600
#define SINE_DRIVE_TIMER_1ms		60
#define SINE_DRIVE_TIMER_500us	30

typedef enum state_machine
{
	STOPPED,
	ACCELERATE,
	RUN,
	DECELERATE
} stm;

extern void SineDrive_init(void);
extern void SineDrive_stepUp(void);
extern void SineDrive_stepDown(void);
extern void SineDrive_SineStep(void);
extern void SineDrive_DeltaStep(void);
extern void SineDrive_setFrequency(float Frequency);
extern void SineDrive_setAmplitude(float Amplitude);
extern void SineDrive_setPower(float Power);
extern void SineDrive_do(void);
extern void SineDrive_setState(enum state_machine Stat);
extern void SineDrive_Start(void);
extern void SineDrive_Stop(void);
extern void SineDrive_Halt(void);
extern enum state_machine SineDrive_getState(void);

extern void SineDrive_getPhaseStartTime(void);
extern void SineDrive_getPhaseStopTime(void);
extern void SineDrive_getIndexStartTime(void);
extern void SineDrive_getIndexStopTime(void);
extern void SineDrive_calcTimeDrift(void);
extern void SineDrive_resetOffsetMeasuring(void);

extern uint8_t doUpdate;

#endif /* SINEDRIVE_H_ */
