#ifndef SENSORS_ENCODER_H
#define SENSORS_ENCODER_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "elapsed_timer.hpp"
#include "sampler.hpp"
#include "interrupt_handler.hpp"

class Encoder: public HasCallback {
public:
    Encoder(const gpio_num_t GPIO);
    ~Encoder() {}
    float get_speed()const{ return sampler.get_average(); }
private:
    void callback();

    const gpio_num_t GPIO;
    ElapsedTimer timer;
    Sampler sampler;
};

#endif
