#include "globals.h"

// Define the shared variables declared as extern in globals.h
volatile bool PRINT = false;
volatile int milliSec = 0;
volatile int digitSelect = 0;
volatile uint32_t currentPress = 0; // variable to track time between button presses
volatile uint32_t lastPress = 0; // variable to track time between button presses
