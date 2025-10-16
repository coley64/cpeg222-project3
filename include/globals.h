#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>
#include <stdbool.h>

extern volatile bool PRINT;
extern volatile int milliSec;
extern volatile int digitSelect;
extern volatile uint32_t currentPress; // variable to track time between button presses
extern volatile uint32_t lastPress; // variable to track time between button presses

#endif