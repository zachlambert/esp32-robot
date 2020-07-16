#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

namespace pin {
    extern const gpio_num_t MOTOR_ENA;
    extern const gpio_num_t MOTOR_ENB;
    extern const gpio_num_t MOTOR_IN1;
    extern const gpio_num_t MOTOR_IN2;
    extern const gpio_num_t MOTOR_IN3;
    extern const gpio_num_t MOTOR_IN4;

    extern const gpio_num_t ENCODER_LEFT;
    extern const gpio_num_t ENCODER_RIGHT;
    extern const gpio_num_t LINE_FOLLOWER_LEFT;
    extern const gpio_num_t LINE_FOLLOWER_RIGHT;

    extern const gpio_num_t MPU_SCL;
    extern const gpio_num_t MPU_SDA;
    extern const gpio_num_t MPU_INT;

    extern const gpio_num_t CURRENT;
};

#endif
