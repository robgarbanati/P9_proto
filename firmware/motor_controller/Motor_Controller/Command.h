#ifndef __COMMAND_H
#define __COMMAND_H

#include "Platform.h"
#include "cmsis_os.h"

//
// Global Defines and Declarations
//

// Command type constants
#define COMMAND_TYPE_NONE_SYNC 0

#define COMMAND_TYPE_BODY_MOTION 1
#define COMMAND_TYPE_BODY_MOTION_WITH_PARAM 2
#define COMMAND_TYPE_FRONT_LED 3
#define COMMAND_TYPE_TAIL_LED 4
#define COMMAND_TYPE_AUDIO 5
#define COMMAND_TYPE_HEAD_PAN 6
#define COMMAND_TYPE_HEAD_TILT 7
#define COMMAND_TYPE_EYE_BRIGHTNESS 8
#define COMMAND_TYPE_EYE_PATTERN 9
#define COMMAND_TYPE_EYE_SEQUENCE 10
#define COMMAND_TYPE_LEFT_EAR 11
#define COMMAND_TYPE_RIGHT_EAR 12
#define COMMAND_TYPE_BUTTON_LED 13
#define COMMAND_TYPE_AUDIO_VOLUME 14
#define COMMAND_TYPE_EYE_SEQUENCE_PARAM 15
#define COMMAND_TYPE_CLAP_LEVEL 16

#define COMMAND_TYPE_PAN_PGAIN 0x80
#define COMMAND_TYPE_PAN_DGAIN 0x81
#define COMMAND_TYPE_PAN_IGAIN 0x82
#define COMMAND_TYPE_PAN_BOOST 0x83
#define COMMAND_TYPE_TILT_PGAIN 0x84
#define COMMAND_TYPE_TILT_DGAIN 0x85
#define COMMAND_TYPE_TILT_IGAIN 0x86
#define COMMAND_TYPE_TILT_BOOST 0x87

#define COMMAND_TYPE_WHEEL_PGAIN 0x90
#define COMMAND_TYPE_WHEEL_DGAIN 0x91
#define COMMAND_TYPE_WHEEL_IGAIN 0x92
#define COMMAND_TYPE_WHEEL_BOOST 0x93

#define COMMAND_TYPE_DEBUG 0xA0

#define COMMAND_TYPE_POWER_OFF 200

/* Blutooth LE Commands
COMMAND_TYPE_BODY_MOTION
command_type = 1
command_byte_1 = Left motor speed MSB (-2400 to 2400)
command_byte_2 = Left motor speed LSB (-2400 to 2400)
command_byte_3 = Right motor speed MSB (-2400 to 2400)
command_byte_4 = Right motor speed LSB (-2400 to 2400)

COMMAND_TYPE_BODY_MOTION_WITH_PARAM
command_type = 2
command_byte_1 = Left motor speed (0 to 20)
command_byte_2 = Right motor speed (0 to 20)
command_byte_3 = Motion parameters

COMMAND_TYPE_FRONT_LED
command_type = 3
command_byte_1 = LED value Red
command_byte_2 = LED value Green
command_byte_3 = LED value Blue

COMMAND_TYPE_TAIL_LED
command_type = 4
command_byte_1 = LED value Red

COMMAND_TYPE_AUDIO_DATA
command_type = 5
command_byte_1 = Index of audio file to play

COMMAND_TYPE_HEAD_PAN
command_type = 6
command_byte_1 = angle MSB (-13500 to 13500)
command_byte_2 = angle LSB (-13500 to 13500)

COMMAND_TYPE_HEAD_TILT
command_type = 7
command_byte_1 = angle MSB (-2000 to 3000)
command_byte_2 = angle LSB (-2000 to 3000)

COMMAND_TYPE_EYE_BRIGHTNESS
command_type = 8
command_byte_1 = brightness (0 = off, 255 = bright)

COMMAND_TYPE_EYE_PATTERN
command_type = 9
command_byte_1 = Eye bit mask LSB
command_byte_2 = Eye bit mask MSB

COMMAND_TYPE_EYE_SEQUENCE
command_type = 10
command_byte_1 = pre-programmed sequence (Bit 8 = repeat, Bits 0:7 = sequence number)

COMMAND_TYPE_LEFT_EAR
command_type = 11
command_byte_1 = LED value Red
command_byte_2 = LED value Green
command_byte_3 = LED value Blue

COMMAND_TYPE_RIGHT_EAR
command_type = 12
command_byte_1 = LED value Red
command_byte_2 = LED value Green
command_byte_3 = LED value Blue

COMMAND_TYPE_BUTTON_LED
command_type = 13
command_byte_1 = LED value Red
command_byte_2 = LED value Green

COMMAND_TYPE_AUDIO_VOLUME
command_type = 4
command_byte_1 = Audio Volume (0 - off, 255 = max_volume)
*/


typedef struct
{
	UINT8 length;
	UINT8 buffer[31];
} commandMessage;

extern osPoolId commandMemPool;
extern osMessageQId commandQueue;
extern osThreadId commandProcessingThreadID;

//
// Global Functions
//
void commandInit(void);
void commandProcessingThread(void const *argument);

#define BUTTON_CENTER 0x01
#define BUTTON_ONE    0x02
#define BUTTON_TWO    0x04
#define BUTTON_THREE  0x08
#define EVENT_CLAP    0x10

#endif // __COMMAND_H



