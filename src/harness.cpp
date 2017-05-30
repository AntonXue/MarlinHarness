#include "harness.h"
#include <stdio.h>

#include "Marlin.h"
#include "planner.h"

// A bunch of stuff that were externed to Marlin_main.cpp
bool Running = true;
volatile bool wait_for_heatup = false;
float current_position [XYZE] = { 0.0 };
Stopwatch print_job_timer = Stopwatch();
bool axis_known_position[XYZ] = { false };
int16_t fanSpeeds[FAN_COUNT] = { 0 };
uint8_t marlin_debug_flags = DEBUG_NONE;
float volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(1.0);
int flow_percentage[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(100);

void print_heaterstates() {
    printf("printing heater states\n");
}

void kill(const char* str) {
    printf("killing: %s\n", str);
}

void idle() {
    printf("idling\n");
}

void disable_all_steppers() {
    printf("disabled all steppers\n");
}

int16_t code_value_temp_diff() {
    printf("code_value_temp_diff\n");
    return 0;
}

int16_t code_value_temp_abs() {
    return 0;
}



bool code_seen(char code) {
    // seen_pointer = strchr(current_command_args, code);
    // return (seen_pointer != NULL); // Return TRUE if the code-letter was found
    return true;
}
