
#include "robot.hpp"

#include <cmath>

#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_types.h"

#include "motor.hpp"
#include "encoder.hpp"
#include "controllers.hpp"
#include "line_follower.hpp"

static const char *TAG = "Robot";

class Robot {
public:
    Robot(float dt);
    void update();
private:
    float dt;
    Motor left_motor;
    Motor right_motor;
    Encoder left_encoder;
    Encoder right_encoder;
    IntegralController left_motor_controller;
    IntegralController right_motor_controller;

    LineFollower left_line_follower;
    LineFollower right_line_follower;
};

/*
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
    gpio_set_direction(pin::MPU_INT, GPIO_MODE_INPUT);
    gpio_set_intr_type(pin::MPU_INT, GPIO_INTR_POSEDGE);

    gpio_isr_handler_add(
        pin::ENCODER_LEFT, isr_handler, (void*)&pin::ENCODER_LEFT
    );
    gpio_isr_handler_add(
        pin::ENCODER_LEFT, isr_handler, (void*)&pin::ENCODER_RIGHT
    );
    gpio_isr_handler_add(
        pin::ENCODER_LEFT, isr_handler, (void*)&pin::LINE_FOLLOWER_LEFT
    );
    gpio_isr_handler_add(
        pin::ENCODER_LEFT, isr_handler, (void*)&pin::LINE_FOLLOWER_RIGHT
    );
    gpio_isr_handler_add(
        pin::ENCODER_LEFT, isr_handler, (void*)pin::MPU_INT
    );
}

void robot_setup_i2c()
{
    i2c_config_t i2c_config = {
        I2C_MODE_MASTER, //mode
        pin::MPU_SDA, //sda_io_num
        pin::MPU_SCL, //scl_io_num
        GPIO_PULLUP_ENABLE, //sda_pullup_en
        GPIO_PULLUP_ENABLE, //scl_pullup_en
        {{100000}} // union -> struct master -> clk_speed
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, i2c_config.mode, 0, 0, 0);
}
*/


Robot::Robot(float dt)
    : dt(dt),
     left_motor(LEFT_MOTOR_CONFIG),
     right_motor(RIGHT_MOTOR_CONFIG),
     left_encoder((gpio_num_t)CONFIG_PIN_ENCODER_LEFT),
     right_encoder((gpio_num_t)CONFIG_PIN_ENCODER_RIGHT),
     left_motor_controller(dt, 0.02, 2),
     right_motor_controller(dt, 0.02, 2),
     left_line_follower(
        (gpio_num_t)CONFIG_PIN_LINE_FOLLOWER_LEFT),
     right_line_follower(
        (gpio_num_t)CONFIG_PIN_LINE_FOLLOWER_RIGHT)
{
    // left_motor_controller.set_sp(400);
    // right_motor_controller.set_sp(400);
    left_motor.set_duty_cycle(50);
    right_motor.set_duty_cycle(50);
}


void Robot::update()
{
    if (left_line_follower.is_over_line()) {
        if (right_line_follower.is_over_line()) {
            // On line
            left_motor.move_forward();
            right_motor.move_forward();
        } else {
            // Off to the right
            left_motor.move_backward();
            right_motor.move_forward();
        }
    } else {
        if (right_line_follower.is_over_line()) {
            // Off to the left
            left_motor.move_forward();
            right_motor.move_backward();
        } else {
            // Off completely
            left_motor.move_backward();
            right_motor.move_backward();
        }
    }

    /*
    float left_speed = left_encoder.sample_speed(dt);
    float right_speed = right_encoder.sample_speed(dt);

    left_motor_controller.update(left_speed);
    right_motor_controller.update(right_speed);

    double left_pwm = left_motor_controller.get_sp() * left_motor_controller.get_cv();
    if (left_pwm > 100) left_pwm = 100;
    if (left_pwm < 0) left_pwm = 0;

    double right_pwm = right_motor_controller.get_sp() * right_motor_controller.get_cv();
    if (right_pwm > 100) right_pwm = 100;
    if (right_pwm < 0) right_pwm = 0;

    left_motor.set_duty_cycle(left_pwm);
    right_motor.set_duty_cycle(right_pwm);

    ESP_LOGI(TAG, "[LEFT] target (%f) | vel (%f) | pwm(%f)", left_motor_controller.get_sp(), left_speed, left_pwm);
    ESP_LOGI(TAG, "[RIGHT] target (%f) | vel (%f) | pwm(%f)", right_motor_controller.get_sp(), right_speed, right_pwm);
    */
}


TaskHandle_t robot_task_handler;
void robot_timer_callback(xTimerHandle timer_handle)
{
    xTaskNotifyGive(robot_task_handler);
}


void robot_task(void *params)
{
    float period_seconds = *(float *)params;
    gpio_install_isr_service(0);
    Robot robot(period_seconds);
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        robot.update();
    }
}


void robot_start()
{
    ESP_LOGI(TAG, "Starting robot");

    TickType_t period_ticks = pdMS_TO_TICKS(50);
    float period_seconds = (float)period_ticks / (portTICK_PERIOD_MS * 1000);

    xTaskCreate(robot_task, "robot", 8192, &period_seconds, 2, &robot_task_handler);

    xTimerHandle robot_timer = xTimerCreate(
        "robot timer",
        period_ticks,
        true, // auto reload
        NULL, // timer id
        robot_timer_callback
    );
    xTimerStart(robot_timer, 0);

    int stack_memory = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Stack available: %d", stack_memory);
}
