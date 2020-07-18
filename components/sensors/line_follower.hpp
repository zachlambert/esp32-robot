#ifndef SENSORS_LINE_FOLLOWER_H
#define SENSORS_LINE_FOLLOWER_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "elapsed_timer.hpp"
#include "interrupt_handler.hpp"

// Unecessary to use a mutex lock to handle the
// over_line variable, since its an atomic instruction
class LineFollower: public HasCallback {
public:
    LineFollower(const gpio_num_t GPIO);
    bool is_over_line()const{ return over_line; }
private:
    void callback();
    bool over_line;
    const gpio_num_t GPIO;
};

#endif
