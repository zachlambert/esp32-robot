#ifndef UTILS_INTERRUPT_HANDLER_H
#define UTILS_INTERRUPT_HANDLER_H

#include "driver/gpio.h"

class HasCallback {
public:
    virtual ~HasCallback() {}
private:
    virtual void callback()=0;
    friend void task_interrupt_handler(void *params);
};

void register_interrupt(gpio_num_t GPIO, HasCallback *object);

#endif
