#include "encoder.hpp"

#include <math.h>

static void IRAM_ATTR encoder_isr_handler(void *args)
{
    xQueueHandle *interrupt_queue = (xQueueHandle *)args;
    static const int dummy_item = 0;
    xQueueSendFromISR(*interrupt_queue, &dummy_item, NULL);
}

// Friend function for encoder, with encoder pointer passed as a
// parameter. Has private access, while being able to create
// a task using this function.
void task_encoder_interrupt_handler(void *params)
{ 
    Encoder& encoder = *(Encoder *)params;
    xQueueHandle interrupt_queue = xQueueCreate(2, sizeof(int));
    gpio_isr_handler_add(
        encoder.GPIO,
        encoder_isr_handler,
        (void*)&interrupt_queue
    );
    int item;
    while (true) {
        if (xQueueReceive(interrupt_queue, &item, portMAX_DELAY)){
            encoder.callback();
        }
    }
}

Encoder::Encoder(const gpio_num_t GPIO) : GPIO(GPIO)
{
    gpio_pad_select_gpio(GPIO);
    gpio_set_direction(GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO, GPIO_INTR_POSEDGE);

    xTaskCreate(
        task_encoder_interrupt_handler,
        "encoder interrupt handler",
        8192,
        this,
        1,
        NULL
    );
}

void Encoder::callback()
{
    double dt = timer.sample_dt();
    sampler.add(2*M_PI/(CONFIG_NUM_ENCODER_SLOTS * dt));
}
