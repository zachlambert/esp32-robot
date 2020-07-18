#include "encoder.hpp"

#include <math.h>
#include <numeric>
#include "esp_log.h"

static const char *TAG = "Encoder";

Encoder::Encoder(const gpio_num_t GPIO)
    : GPIO(GPIO), timer()
{
    gpio_pad_select_gpio(GPIO);
    gpio_set_direction(GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO, GPIO_INTR_ANYEDGE);
    register_interrupt(GPIO, static_cast<HasCallback*>(this));
    std::fill(count.begin(), count.end(), 0);
}


// Expects the speed to be sampled at regular intervals
// with time step dt
float Encoder::sample_speed(float dt)
{
    index = (index+1) % SAMPLE_SIZE;
    count[index] = 0;
    unsigned int num_steps = std::accumulate(
        count.begin(), count.end(), (unsigned int)0
    );
    static const float step_size = 2*M_PI / (2 * CONFIG_NUM_ENCODER_SLOTS);
    static const float scale = step_size / (dt * (SAMPLE_SIZE-1));
    return num_steps * scale;
}


void Encoder::callback()
{
    count[index]++;
}
