#ifndef SENSORS_ENCODER_H
#define SENSORS_ENCODER_H

#include <array>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "elapsed_timer.hpp"
#include "interrupt_handler.hpp"


// Don't need to use a mutex lock with the count data,
// since it is read and written atomically
class Encoder: public HasCallback {
public:
    Encoder(const gpio_num_t GPIO);
    float sample_speed(float dt);

private:
    void callback();

    static const std::size_t SAMPLE_SIZE = 10;
    std::array<unsigned int, SAMPLE_SIZE> count;
    std::size_t index;
    unsigned int count_sum;
    const gpio_num_t GPIO;
};

#endif
