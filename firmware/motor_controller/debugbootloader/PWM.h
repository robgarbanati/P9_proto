#ifndef __PWM_H
#define __PWM_H

#include "Platform.h"

//
// Global Defines and Declarations
//
#define ONLINE_STATE	1
#define BASELINE_STATE	2
#define STEPUP1_STATE	3
#define STEPUP2_STATE	4
#define STEPUP3_STATE	5
#define STEPUP4_STATE	6
#define TIMEOUT_STATE	7

#define MOTOR_PWM_PERIOD	128
#define STEP_PERIOD     17

//
// Global Functions
//
void PWM_Init(void);

void PWM_set_output0(uint16_t Value);
void PWM_set_output1(uint16_t Value);
void PWM_set_output2(uint16_t Value);

void TestFunc(void);


#endif // __PWM_H


