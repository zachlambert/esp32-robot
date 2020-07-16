#include "encoder.hpp"

#include <math.h>

Encoder::Encoder(const gpio_num_t GPIO)
    : GPIO(GPIO), timer(), sampler()
{
    gpio_pad_select_gpio(GPIO);
    gpio_set_direction(GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO, GPIO_INTR_ANYEDGE);
    register_interrupt(GPIO, this);
}

void Encoder::callback()
{
    double dt = timer.sample_dt();
    sampler.add(2*M_PI/(2 * CONFIG_NUM_ENCODER_SLOTS * dt));
    // Interrupts on both edges of the slot, so get
    // 2 * CONFIG_NUM_ENCODER_SLOTS interrupts in a revolution
}
