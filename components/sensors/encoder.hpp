#ifndef SENSORS_ENCODER_H
#define SENSORS_ENCODER_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "elapsed_timer.hpp"
#include "sampler.hpp"

class Encoder {
public:
    Encoder(const gpio_num_t GPIO);
    float get_speed()const{ return sampler.get_average(); }
private:
    static void callback(void *data);

    const gpio_num_t GPIO;
    ElapsedTimer timer;
    Sampler sampler;
};

#endif
