#pragma once

#if STICKF103
#include "StickF103.h"
#else
#error Unknown board
#endif

#include <stdint.h>
#include <stdbool.h>

#include "board/led.h"

// Application config can #if on target.
#include "config.h"
