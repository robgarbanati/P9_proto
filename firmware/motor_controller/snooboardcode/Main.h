#ifndef __MAIN_H
#define __MAIN_H

#include "Platform.h"

//
// Global Defines and Declarations
//

#define POWER_OFF_BUTTON_GPIO	GPIOB
#define POWER_OFF_BUTTON_PIN	DRVGPIO_PIN_6

#define ACTIVITY_BUTTON_GPIO	GPIOB
#define ACTIVITY_BUTTON_PIN		DRVGPIO_PIN_12

#define HOMING_SENSOR_GPIO		GPIOB
#define HOMING_SENSOR_PIN			DRVGPIO_PIN_13

#define SAFETY_CLIP_GPIO		  GPIOB
#define RIGHT_SAFETY_CLIP			DRVGPIO_PIN_14
#define LEFT_SAFETY_CLIP			DRVGPIO_PIN_15


int is_left_safety_clip_in(void);
int is_right_safety_clip_in(void);
int get_activity_button_state(void);

#endif // __MAIN_H


