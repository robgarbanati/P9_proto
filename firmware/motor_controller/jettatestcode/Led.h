#ifndef __LED_H
#define __LED_H

#include "Platform.h"
//#include "cmsis_os.h"

#define EYE_INTERRUPT_SIGNAL	0x08

extern osThreadId eyeThreadID;

void ledInit(void);
void ledTopBrightness(UINT8 redLevel, UINT8 greenLevel);
void ledLeftEarBrightness(UINT8 redLevel, UINT8 greenLevel, UINT8 blueLevel);
void ledRightEarBrightness(UINT8 redLevel, UINT8 greenLevel, UINT8 blueLevel);
void ledLeftEarlightConfigure(UINT8 red, UINT8 green, UINT8 blue);
void ledRightEarlightConfigure(UINT8 red, UINT8 green, UINT8 blue);
void ledTopButtonLightConfigure(UINT8 red, UINT8 green);

void ledEyeBrightness(UINT8 level);
void ledEyePattern(UINT8 pattern1, UINT8 pattern2);
void ledEyePattern16(UINT16 pattern);
void ledEyeBrightnessPattern(UINT8 level, UINT16 pattern);
void ledEyeSequence(UINT8 sequence_num, BOOL repeat, UINT8 param);

BOOL get_clap(void);

typedef struct sequence_step_t
{
	UINT16 state;
	UINT16 count;
} sequence_step;

// Thread that handles Eye Sequences
void eyeThread(void const *argument);

#define EYE_FULL_OPEN           0xFFFF
#define EYE_FULL_SHUT           0x0000

#define EYE_LED_01              0x0001
#define EYE_LED_02              0x0002
#define EYE_LED_03              0x0004
#define EYE_LED_04              0x0008
#define EYE_LED_05              0x0010
#define EYE_LED_06              0x0020
#define EYE_LED_07              0x0040
#define EYE_LED_08              0x0080
#define EYE_LED_09              0x0100
#define EYE_LED_10              0x0200
#define EYE_LED_11              0x0400
#define EYE_LED_12              0x0800
#define EYE_LED_13              0x1000
#define EYE_LED_14              0x2000
#define EYE_LED_15              0x4000
#define EYE_LED_16              0x8000

#define EYE_PARTLY_OPEN_1       0x7E7E
#define EYE_PARTLY_OPEN_2       0x3C3C
#define EYE_PARTLY_OPEN_3       0x1818
#define EYE_PARTLY_SLIT_OPEN    0x1008

#define TOP_EYELID_CLOSING_1    0xFEFF
#define TOP_EYELID_CLOSING_2    0xFC7F
#define TOP_EYELID_CLOSING_3    0xF83F
#define TOP_EYELID_CLOSING_4    0xF01F
#define TOP_EYELID_CLOSING_5    0xE00F
#define TOP_EYELID_CLOSING_6    0xC007
#define TOP_EYELID_CLOSING_7    0x8003
#define TOP_EYELID_CLOSING_8    0x0001

#define SEQUENCE_ALL_OFF 0

#define SEQUENCE_ALL_ON 1

#define SEQUENCE_FULL_BLINK 2
#define SEQUENCE_FULL_BLINK_SIZE 18

#define SEQUENCE_HALF_BLINK 3
#define SEQUENCE_HALF_BLINK_SIZE 8

#define SEQUENCE_CIRCLE 5
#define SEQUENCE_CIRCLE_SIZE 16

#define SEQUENCE_FAST_BLINK 6
#define SEQUENCE_FAST_BLINK_SIZE 18


#endif
