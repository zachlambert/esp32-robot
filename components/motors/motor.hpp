#ifndef MOTORS_H
#define MOTORS_H

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

struct MotorConfig {
    const gpio_num_t GPIO_A;
    const gpio_num_t GPIO_B;
    const gpio_num_t GPIO_EN;
    const mcpwm_unit_t MCPWM_UNIT;
    const mcpwm_timer_t MCPWM_TIMER;
};

extern const MotorConfig LEFT_MOTOR_CONFIG;
extern const MotorConfig RIGHT_MOTOR_CONFIG;

enum class MotorState {
    STOPPED,
    FORWARD,
    BACKWARD
};

class Motor {
public:
    Motor(const MotorConfig& config);
    void set_duty_cycle(float velocity);
    void move_forward();
    void move_backward();
    void stop();

private:
    void set_forward_pwm();
    void set_backward_pwm();
    void disable_pwm();

    const MotorConfig& config;
    float duty_cycle;
    MotorState state;
};

#endif
