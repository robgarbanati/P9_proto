#ifndef __VUART_H
#define __VUART_H

#include "Global.h"

//
// Global Defines and Declarations
//

//
// Global Functions
//

void vuartInit(void);

BOOL vuartGetChar(UINT8 *ch);
BOOL vuartGetCharTimeout(UINT8 *ch, UINT32 timeout);
BOOL vuartPutChar(UINT8 ch);

BOOL vuartPutRecvQueue(const PACKETData *packet);
BOOL vuartGetRecvQueue(PACKETData *packet);

#endif // __VUART_H


