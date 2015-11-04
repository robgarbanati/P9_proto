#ifndef __VUART_H
#define __VUART_H

#include "global.h"
#include "packet.h"

//
// Global Defines and Declarations
//

//
// Global Functions
//

void vuart_init(void);
BOOL vuart_get_char_with_timeout(UINT8 *ch, UINT32 timeout);
BOOL vuart_put_char_with_timeout(UINT8 ch, UINT32 timeout);

// Helper inline functions.
static inline BOOL vuart_get_char(UINT8 *ch) { return vuart_get_char_with_timeout(ch, osWaitForever); }
static inline BOOL vuart_put_char(UINT8 ch) { return vuart_put_char_with_timeout(ch, osWaitForever); }

// VUART packet queue management functions.
BOOL vuart_get_send_queue(PACKETData *packet);
BOOL vuart_put_recv_queue(const PACKETData *packet);

#endif // __VUART_H


