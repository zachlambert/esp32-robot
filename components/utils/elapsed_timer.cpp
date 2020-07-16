
#include "elapsed_timer.hpp"

double ElapsedTimer::sample_dt()
{
    current_us = esp_timer_get_time();
    dt = (double)(current_us - prev_us);
    prev_us = current_us;
    return dt;
}
