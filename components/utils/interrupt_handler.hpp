#ifndef UTILS_INTERRUPT_HANDLER_H
#define UTILS_INTERRUPT_HANDLER_H

#include "driver/gpio.h"

typedef void (*callback_t)(void *);
struct InterruptCallback {
    void *data;
    callback_t callback;
    // Its up to the callback function to cast the void pointer
    InterruptCallback() : data(0), callback(0) {}
    InterruptCallback(void *data, callback_t callback) : data(data), callback(callback) {}
};

void register_interrupt(gpio_num_t GPIO, void *data, callback_t callback);

#endif
