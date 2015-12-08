#ifndef __PWM_H
#define __PWM_H

#include "Platform.h"

//
// Global Defines and Declarations
//

#define MOTOR_PWM_PERIOD	128
#define STEP_PERIOD     17

//
// Global Functions
//
void PWM_Init(void);

void PWM_set_output0(uint16_t Value);
void PWM_set_output1(uint16_t Value);
void PWM_set_output2(uint16_t Value);


#endif // __PWM_H


