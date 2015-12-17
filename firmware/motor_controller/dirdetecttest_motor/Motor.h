#ifndef __MOTOR_H
#define __MOTOR_H

#include "Platform.h"

//
// Global Defines and Declarations
//
#define MOTOR_TIMER_1s		60000
#define MOTOR_TIMER_100ms	6000
#define MOTOR_TIMER_10ms	600
#define MOTOR_TIMER_1ms		60
#define MOTOR_TIMER_500us	30
#define NUM_EDGES	2


//
// Global Functions
//

UINT32 get_ms_print(int ms_read_index);

int get_ms_write_index(void);

//void control_motor_speed(int current_speed);
void control_motor_speed(void);

void Start_Timer0(void);

void Motor_Init(void);

void homing_sensor_handler(void);

int is_left_safety_clip_in(void);

int is_right_safety_clip_in(void);

int motor_control(int desired_speed, int current_speed);


#endif // __MOTOR_H


