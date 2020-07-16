
#include "robot.hpp"

#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_types.h"

#include "motor.hpp"
#include "encoder.hpp"

static const char *TAG = "Robot";

class Robot {
public:
    Robot(float dt);
private:
    void callback();
    friend void robot_timer_callback(xTimerHandle timer_handle);

    float dt;
    Motor left_motor;
    Motor right_motor;
    Encoder left_encoder;
    Encoder right_encoder;
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
    right_encoder((gpio_num_t)CONFIG_PIN_ENCODER_RIGHT)
{

}


void Robot::callback()
{
    left_encoder.sample_speed(dt);
    right_encoder.sample_speed(dt);
    // left_pid_controller.loop(left_encoder.sample_speed(dt));
    // right_pid_controller.loop(right_encoder.sample_speed(dt));
    // left_motor.set_speed(left_pid_controller.get_cv());
    // right_motor.set_speed(right_pid_controller.get_cv());
}


Robot* robot;

void robot_timer_callback(xTimerHandle timer_handle)
{
    robot->callback();
}

void robot_start()
{
    ESP_LOGI(TAG, "Starting robot");

    // Must call before adding interrupts, which is done during
    // the construction of Robot
    gpio_install_isr_service(0);

    TickType_t period_ticks = 1000;
    float period_seconds = (float)period_ticks / (portTICK_PERIOD_MS * 1000);

    robot = new Robot(period_seconds);

    xTimerHandle timer_handle = xTimerCreate(
        "robot timer",
        period_ticks,
        true, // auto reload
        NULL, // timer id
        robot_timer_callback
    );
}
