
#ifndef __ROBOTSOUND_H__
#define __ROBOTSOUND_H__

#include "Platform.h"
#include "System/SysInfra.h"

//#define ROBOT_FRAME_LEN   256	// must be multile times of 8

#define	MAX_PITCH	168	// must be multile times of 4
//#define	MAX_PITCH	192
#define MIN_PITCH	40

//#define ROBOT_MEM_SIZE ( ROBOT_FRAME_LEN*3 + MAX_PITCH*2 )


void RobotSoundA_Init(INT16 i16PitchDuration, INT16 i16FrameLen, INT16 *pi16Ptr);
void RobotSoundA_GetData(INT16 *pi16Ptr);
void RobotSoundA_Convert(INT16 *pi16Ptr);

#endif	//#ifndef __ROBOTSOUND_H__
