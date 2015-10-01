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

#define MAX_AMPLITUDE_FACTOR 5000

#define SINE_DRIVE_TIMER_10ms		600
#define SINE_DRIVE_TIMER_1ms		60
#define SINE_DRIVE_TIMER_500us	30

extern int16_t phy_measured_offset;

typedef enum state_machine
{
	STOPPED,
	ACCELERATE,
	RUN,
	DECELERATE
} stm;

void SineDrive_init(void);
void SineDrive_do(void);
void SineDrive_Halt(void);
void SineDrive_setMotorMovement(float Frequency, float Amplitude, float Power, UINT16 TransitionTime);

#endif /* SINEDRIVE_H_ */
