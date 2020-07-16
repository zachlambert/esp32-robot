#ifndef MOTORS_H
#define MOTORS_H

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

struct MotorConfig {
    const gpio_num_t GPIO_A;
    const gpio_num_t GPIO_B;
    const gpio_num_t GPIO_EN;
    const mcpwm_unit_t MCPWM_UNIT;
};

extern const MotorConfig LEFT_MOTOR_CONFIG;
extern const MotorConfig RIGHT_MOTOR_CONFIG;

class Motor {
public:
    Motor(const MotorConfig& config);
    void set_speed(float speed);
    void enable();
    void disable();
private:
    void move_forward();
    void move_backward();
    void stop();
    const MotorConfig& config;
    float duty_cycle;
};

#endif
