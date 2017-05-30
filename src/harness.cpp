#include "harness.h"
#include <stdio.h>

#include "Marlin.h"
#include "planner.h"

// A bunch of stuff that were externed to Marlin_main.cpp
bool Running = true;
volatile bool wait_for_heatup = false;
float current_position [XYZE] = { 0.0 };

void print_heaterstates() {}
void kill(const char*) {}

Stopwatch print_job_timer = Stopwatch();

bool axis_known_position[XYZ] = { false };

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
