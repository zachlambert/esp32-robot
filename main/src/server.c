
#include "server.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void task_server(void *params)
{
    int i = 0;
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Dummy code for server [%d]\n", i);
        ++i;
    }
}
