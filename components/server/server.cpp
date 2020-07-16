
#include "server.hpp"

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const static char *TAG = "Server";

void task_server(void *params)
{
    int i = 0;
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // ESP_LOGI(TAG, "Dummy code for server [%d]", i);
        ++i;
    }
}
