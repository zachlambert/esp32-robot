#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "robot.h"
#include "server.h"

void app_main(void)
{
    xTaskCreate(task_robot, "task_robot", 2048, NULL,  2, NULL);
    xTaskCreate(task_server, "task_server", 2048, NULL, 1, NULL); 
}
