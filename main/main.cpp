#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "robot.hpp"
#include "server.hpp"

static const char *TAG = "Main";

extern "C" void app_main(void)
{
    int stack_memory = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Stack available: %d", stack_memory);
    xTaskCreate(task_robot, "task_robot", 8192, NULL,  2, NULL);
    xTaskCreate(task_server, "task_server", 2048, NULL, 1, NULL);
}
