
#include "motor.hpp"

#include "esp_log.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "pins.hpp"

static const char *TAG = "Motor";

const MotorConfig LEFT_MOTOR_CONFIG = {
    pin::MOTOR_IN1,
    pin::MOTOR_IN2,
    pin::MOTOR_ENA,
    MCPWM_UNIT_0
};

const MotorConfig RIGHT_MOTOR_CONFIG = {
    pin::MOTOR_IN3,
    pin::MOTOR_IN4,
    pin::MOTOR_ENB,
    MCPWM_UNIT_1
};

Motor::Motor(const MotorConfig& config)
    :config(config), duty_cycle(0)
{
    ESP_LOGD(TAG, "Creating motor");

    // Configure enable pin
    gpio_pad_select_gpio(config.GPIO_EN);
    gpio_set_direction(config.GPIO_EN, GPIO_MODE_OUTPUT);

    mcpwm_io_signals_t signal_a =
        (config.MCPWM_UNIT == MCPWM_UNIT_0 ? MCPWM0A : MCPWM1A);
    mcpwm_io_signals_t signal_b =
        (config.MCPWM_UNIT == MCPWM_UNIT_0 ? MCPWM0B : MCPWM1B);

    // Initialise the mcpwm gpio for the two pwm units
    mcpwm_gpio_init(config.MCPWM_UNIT, signal_a, config.GPIO_A);
    mcpwm_gpio_init(config.MCPWM_UNIT, signal_b, config.GPIO_B);

    // Configure the pwm driver
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; // 500 Hz
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // Active high
    pwm_config.counter_mode = MCPWM_UP_COUNTER; // Asymmetric pwm
    // Also pass the MCPWM unit and timer (always use MCPWM_TIMER_0)
    mcpwm_init(config.MCPWM_UNIT, MCPWM_TIMER_0, &pwm_config);

    enable();
}

void Motor::set_speed(float speed)
{
    if (speed == 0) {
        stop();
    } else if (speed > 0) {
        duty_cycle = speed;
        move_forward();
    } else {
        duty_cycle = -speed;
        move_backward();
    }
}

void Motor::move_forward()
{
    // Set the unused pwm to zero
    mcpwm_set_signal_low(config.MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B);

    // Set the duty cycle of the used pwm
    mcpwm_set_duty(config.MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);

    // Set the duty cycle type again, if the mcpwm gpio was set low previously
    mcpwm_set_duty_type(config.MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

void Motor::move_backward(){
    // Set the unused pwm to zero
    mcpwm_set_signal_low(config.MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A);

    // Set the duty cycle of the used pwm
    mcpwm_set_duty(config.MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);

    // Set the duty cycle type again, if the mcpwm gpio was set low previously
    mcpwm_set_duty_type(config.MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void Motor::stop()
{
    mcpwm_set_signal_low(config.MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(config.MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B);
}

void Motor::enable()
{
    gpio_set_level(config.GPIO_EN, 1);
}

void Motor::disable()
{
    gpio_set_level(config.GPIO_EN, 0);
}

