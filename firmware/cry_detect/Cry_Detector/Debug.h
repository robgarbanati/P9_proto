#ifndef __DEBUG_H
#define __DEBUG_H

#include <stdio.h>
#include "Commu/SemiHost.h"

#define ENABLE_SEMIHOST 0

#if ENABLE_SEMIHOST != 0
#define DEBUG_PRINTF(...) \
            do { if (ENABLE_SEMIHOST) printf(__VA_ARGS__); } while (0)
#else
#define DEBUG_PRINTF(...)
#endif

#endif
