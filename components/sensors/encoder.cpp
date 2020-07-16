#include "encoder.hpp"
#include "interrupt_handler.hpp"

#include <math.h>

Encoder::Encoder(const gpio_num_t GPIO) : GPIO(GPIO)
{
    gpio_pad_select_gpio(GPIO);
    gpio_set_direction(GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO, GPIO_INTR_POSEDGE);
    register_interrupt(GPIO, (void*)this, callback);
}

void Encoder::callback(void *data)
{
    Encoder& encoder = *(Encoder*)data;
    double dt = encoder.timer.sample_dt();
    encoder.sampler.add(2*M_PI/(CONFIG_NUM_ENCODER_SLOTS * dt));
}
