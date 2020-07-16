
#include "server.hpp"

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const static char *TAG = "Server";

void server_start()
{
    ESP_LOGI(TAG, "Starting server");
}
