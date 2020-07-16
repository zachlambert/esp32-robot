#ifndef TIMER_H
#define TIMER_H

#include "esp_types.h"
#include "esp_timer.h"

class ElapsedTimer {
public:
    ElapsedTimer() {}
    double sample_dt();

private:
    int64_t prev_us;
    int64_t current_us;
    double dt;
};

#endif
