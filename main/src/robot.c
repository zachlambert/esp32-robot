
#include "robot.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_types.h"

xQueueHandle interrupt_queue;

static void IRAM_ATTR isr_handler(void *args)
{
    int pin_number = *(int*)args;
    xQueueSendFromISR(interrupt_queue, &pin_number, NULL);
}

void robot_setup_interrupt_add(int pin_number)
{
    int *persistent = malloc(sizeof(int));
    *persistent = pin_number;
    gpio_isr_handler_add(*persistent, isr_handler, persistent);
}

void robot_setup_gpio()
{
    gpio_config_t config_digital_inputs;
    config_digital_inputs.intr_type = GPIO_INTR_ANYEDGE;
    config_digital_inputs.mode = GPIO_MODE_INPUT;
    config_digital_inputs.pin_bit_mask =
        (1ULL << CONFIG_PIN_ENCODER_LEFT) |
        (1ULL << CONFIG_PIN_ENCODER_RIGHT) |
        (1ULL << CONFIG_PIN_LINE_FOLLOWER_LEFT) |
        (1ULL << CONFIG_PIN_LINE_FOLLOWER_RIGHT) |
        (1ULL << CONFIG_PIN_MPU_INT);
    gpio_config(&config_digital_inputs);

    gpio_config_t config_digital_outputs;
    config_digital_outputs.mode = GPIO_MODE_OUTPUT;
    config_digital_outputs.pin_bit_mask =
        (1ULL << CONFIG_PIN_MOTOR_ENA) |
        (1ULL << CONFIG_PIN_MOTOR_ENB) |
        (1ULL << CONFIG_PIN_MOTOR_IN1) |
        (1ULL << CONFIG_PIN_MOTOR_IN2) |
        (1ULL << CONFIG_PIN_MOTOR_IN3) |
        (1ULL << CONFIG_PIN_MOTOR_IN4);
    gpio_config(&config_digital_outputs);

    // Todo: Get the right interrupt for processing MPU data
    gpio_pad_select_gpio(CONFIG_PIN_MPU_INT);
    gpio_set_direction(CONFIG_PIN_MPU_INT, GPIO_MODE_INPUT);
    gpio_set_intr_type(CONFIG_PIN_MPU_INT, GPIO_INTR_POSEDGE);

    interrupt_queue = xQueueCreate(10, sizeof(int));
    gpio_install_isr_service(0);
    robot_setup_interrupt_add(CONFIG_PIN_ENCODER_LEFT);
    robot_setup_interrupt_add(CONFIG_PIN_ENCODER_RIGHT);
    robot_setup_interrupt_add(CONFIG_PIN_LINE_FOLLOWER_LEFT);
    robot_setup_interrupt_add(CONFIG_PIN_LINE_FOLLOWER_RIGHT);
    robot_setup_interrupt_add(CONFIG_PIN_MPU_INT);
}

void robot_setup_i2c()
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_PIN_MPU_SDA,
        .scl_io_num = CONFIG_PIN_MPU_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, i2c_config.mode, 0, 0, 0);
}

void task_robot(void *params)
{
    robot_setup_gpio();
    robot_setup_i2c();

    int pin_number;
    while (true) {
        if (xQueueReceive(interrupt_queue, &pin_number, portMAX_DELAY)) {
            switch (pin_number) {

            }
        }
    }
}
