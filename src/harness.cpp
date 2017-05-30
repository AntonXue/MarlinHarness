#include "harness.h"

#include "Marlin.h"
#include "planner.h"

// A bunch of stuff that were externed to Marlin_main.cpp
bool Running = true;
volatile bool wait_for_heatup = false;

void print_heaterstates() {}
void kill(const char*) {}

Stopwatch print_job_timer = Stopwatch();
