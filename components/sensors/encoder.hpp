#ifndef SENSORS_ENCODER_H
#define SENSORS_ENCODER_H

#include <array>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "elapsed_timer.hpp"
#include "interrupt_handler.hpp"


class Encoder: public HasCallback {
public:
    Encoder(const gpio_num_t GPIO);
    float sample_speed(float dt);

private:
    void callback();

    static const std::size_t SAMPLE_SIZE = 10;
    std::array<unsigned int, SAMPLE_SIZE> count;
    std::size_t index;
    const gpio_num_t GPIO;
};

#endif
