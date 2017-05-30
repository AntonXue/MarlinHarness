#include "millis.h"

unsigned long millis () {
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    return (unsigned long)round(spec.tv_nsec / 1.0e6);
}
