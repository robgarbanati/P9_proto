#ifndef __COMMAND_H
#define __COMMAND_H

#include "Global.h"
#include "Packet.h"

// Command thread ID.
extern osThreadId commandThreadId;

void commandInit(void);

BOOL commandPutRecvQueue(const PACKETData *packet);
BOOL commandGetRecvQueue(PACKETData *packet);

void commandThread(void const *argument);

#endif // __COMMAND_H
