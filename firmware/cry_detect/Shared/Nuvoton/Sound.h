#ifndef __SOUND_H
#define __SOUND_H

#include "Global.h"

// Sound thread ID.
extern osThreadId soundThreadId;

void soundPlay(char *fileName);
void soundStop(void);
void soundSetVolume(UINT16 volume);

void soundThread(void const *argument);

#endif // __SOUND_H
