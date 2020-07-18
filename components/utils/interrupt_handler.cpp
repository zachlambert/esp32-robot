#include "interrupt_handler.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "Interrupt";

xQueueHandle interrupt_queue;

static void IRAM_ATTR isr_handler(void *args)
{
    // Copies the object pointed to by args
    // into the queue, of the requested size
    // Want to copy the pointer itself, so pass
    // the address of the args pointer
    xQueueSendFromISR(interrupt_queue, &args, NULL);
}


void task_interrupt_handler(void *params)
{ 
    void *queue_object;
    while (true) {
        if (xQueueReceive(interrupt_queue, &queue_object, portMAX_DELAY)){
            reinterpret_cast<HasCallback*>(queue_object)->callback();  
        }
    }
}


void register_interrupt(gpio_num_t GPIO, HasCallback* object_p)
{
    if(!interrupt_queue){
        interrupt_queue = xQueueCreate(10, sizeof(HasCallback*));
        xTaskCreate(
            task_interrupt_handler,
            "interrupt handler",
            4096, NULL, 1, NULL
        );
    }
    gpio_isr_handler_add(
        GPIO, isr_handler, reinterpret_cast<void *>(object_p)
    );
}
