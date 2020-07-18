#include "line_follower.hpp"

LineFollower::LineFollower(const gpio_num_t GPIO)
    : GPIO(GPIO)
{
    gpio_pad_select_gpio(GPIO);
    gpio_set_direction(GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO, GPIO_INTR_ANYEDGE);
    register_interrupt(GPIO, this);
}

void LineFollower::callback()
{
    over_line = gpio_get_level(GPIO);
}
