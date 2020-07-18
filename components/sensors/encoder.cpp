#include "encoder.hpp"

#include <math.h>
#include <numeric>
#include "esp_log.h"

static const char *TAG = "Encoder";

Encoder::Encoder(const gpio_num_t GPIO)
    : index(0), count_sum(0), GPIO(GPIO)
{
    gpio_pad_select_gpio(GPIO);
    gpio_set_direction(GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO, GPIO_INTR_ANYEDGE);
    register_interrupt(GPIO, this);
    std::fill(count.begin(), count.end(), 0);
}


// Expects the speed to be sampled at regular intervals
// with time step dt
float Encoder::sample_speed(float dt)
{
    count_sum += count[index];
    index = (index+1) % SAMPLE_SIZE;
    count_sum -= count[index];
    count[index] = 0;
    static const float step_size = 2*M_PI / (2 * CONFIG_NUM_ENCODER_SLOTS);
    static const float scale = step_size / (dt * (SAMPLE_SIZE-1));
    return count_sum * scale;
}


void Encoder::callback()
{
    count[index]++;
}
