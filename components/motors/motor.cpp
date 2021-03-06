
#include "motor.hpp"

#include "esp_log.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

static const char *TAG = "Motor";

const MotorConfig LEFT_MOTOR_CONFIG = {
    (gpio_num_t)CONFIG_PIN_MOTOR_IN1,
    (gpio_num_t)CONFIG_PIN_MOTOR_IN2,
    (gpio_num_t)CONFIG_PIN_MOTOR_ENA,
    MCPWM_UNIT_0,
    MCPWM_TIMER_0
};

const MotorConfig RIGHT_MOTOR_CONFIG = {
    (gpio_num_t)CONFIG_PIN_MOTOR_IN3,
    (gpio_num_t)CONFIG_PIN_MOTOR_IN4,
    (gpio_num_t)CONFIG_PIN_MOTOR_ENB,
    MCPWM_UNIT_1,
    MCPWM_TIMER_1
};


Motor::Motor(const MotorConfig& config)
    :config(config), duty_cycle(0), state(MotorState::STOPPED)
{
    // Configure enable pin
    gpio_pad_select_gpio(config.GPIO_EN);
    gpio_set_direction(config.GPIO_EN, GPIO_MODE_OUTPUT);

    mcpwm_io_signals_t signal_a =
        (config.MCPWM_UNIT == MCPWM_UNIT_0 ? MCPWM0A : MCPWM1A);
    mcpwm_io_signals_t signal_b =
        (config.MCPWM_UNIT == MCPWM_UNIT_0 ? MCPWM0B : MCPWM1B);

    // Initialise the mcpwm gpio for the two pwm units
    ESP_LOGD(TAG, "Creating motor with:");
    ESP_LOGD(TAG, "- MCPWM_UNIT: %d", config.MCPWM_UNIT);
    ESP_LOGD(TAG, "- signal a: %d", signal_a);
    ESP_LOGD(TAG, "- GPIO a: %d", config.GPIO_A);
    ESP_LOGD(TAG, "- signal b: %d", signal_b);
    ESP_LOGD(TAG, "- GPIO b: %d", config.GPIO_B);
    mcpwm_gpio_init(config.MCPWM_UNIT, signal_a, config.GPIO_A);
    mcpwm_gpio_init(config.MCPWM_UNIT, signal_b, config.GPIO_B);

    // Configure the pwm driver
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; // 500 Hz
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // Active high
    pwm_config.counter_mode = MCPWM_UP_COUNTER; // Asymmetric pwm
    // Also pass the MCPWM unit and timer (always use config.MCPWM_UNIT)
    mcpwm_init(config.MCPWM_UNIT, config.MCPWM_TIMER, &pwm_config);

    gpio_set_level(config.GPIO_EN, 1);
}

void Motor::set_duty_cycle(float duty_cycle)
{
    this->duty_cycle = duty_cycle;
    if (state == MotorState::FORWARD) {
        set_forward_pwm();
    } else if (state == MotorState::BACKWARD) {
        set_backward_pwm();
    }
}

void Motor::move_forward() {
    state = MotorState::FORWARD;
    set_forward_pwm();
}

void Motor::move_backward() {
    state = MotorState::BACKWARD;
    set_backward_pwm();
}

void Motor::stop() {
    state = MotorState::STOPPED;
    disable_pwm();
}


void Motor::set_forward_pwm()
{
    // Set the unused pwm to zero
    mcpwm_set_signal_low(config.MCPWM_UNIT, config.MCPWM_TIMER, MCPWM_OPR_B);

    // Set the duty cycle of the used pwm
    mcpwm_set_duty(config.MCPWM_UNIT, config.MCPWM_TIMER, MCPWM_OPR_A, duty_cycle);

    // Set the duty cycle type again, if the mcpwm gpio was set low previously
    mcpwm_set_duty_type(config.MCPWM_UNIT, config.MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}


void Motor::set_backward_pwm()
{
    // Set the unused pwm to zero
    mcpwm_set_signal_low(config.MCPWM_UNIT, config.MCPWM_TIMER, MCPWM_OPR_A);

    // Set the duty cycle of the used pwm
    mcpwm_set_duty(config.MCPWM_UNIT, config.MCPWM_TIMER, MCPWM_OPR_B, duty_cycle);

    // Set the duty cycle type again, if the mcpwm gpio was set low previously
    mcpwm_set_duty_type(config.MCPWM_UNIT, config.MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}


void Motor::disable_pwm()
{
    mcpwm_set_signal_low(config.MCPWM_UNIT, (mcpwm_timer_t)config.MCPWM_UNIT, MCPWM_OPR_A);
    mcpwm_set_signal_low(config.MCPWM_UNIT, (mcpwm_timer_t)config.MCPWM_UNIT, MCPWM_OPR_B);
}
