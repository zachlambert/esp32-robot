#include "interrupt_handler.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

xQueueHandle interrupt_queue;

static void IRAM_ATTR isr_handler(void *args)
{
    xQueueSendFromISR(interrupt_queue, args, NULL);
}


void task_interrupt_handler(void *params)
{ 
    InterruptCallback interrupt_callback;
    while (true) {
        if (xQueueReceive(interrupt_queue, &interrupt_callback, portMAX_DELAY)){
            interrupt_callback.callback(interrupt_callback.data);
        }
    }
}


void register_interrupt(gpio_num_t GPIO, void *data, callback_t callback)
{
    if(!interrupt_queue){
        interrupt_queue = xQueueCreate(10, sizeof(InterruptCallback));
        xTaskCreate(
            task_interrupt_handler,
            "interrupt handler",
            4096, NULL, 1, NULL
        );
    }
    InterruptCallback *persistent = new InterruptCallback(data, callback);
    gpio_isr_handler_add(GPIO, isr_handler, persistent);
}
