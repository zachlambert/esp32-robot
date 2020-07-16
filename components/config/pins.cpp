#include "pins.hpp"
#include "sdkconfig.h"

const gpio_num_t pin::MOTOR_ENA = (gpio_num_t)CONFIG_PIN_MOTOR_ENA;
const gpio_num_t pin::MOTOR_ENB = (gpio_num_t)CONFIG_PIN_MOTOR_ENB;
const gpio_num_t pin::MOTOR_IN1 = (gpio_num_t)CONFIG_PIN_MOTOR_IN1;
const gpio_num_t pin::MOTOR_IN2 = (gpio_num_t)CONFIG_PIN_MOTOR_IN2;
const gpio_num_t pin::MOTOR_IN3 = (gpio_num_t)CONFIG_PIN_MOTOR_IN3;
const gpio_num_t pin::MOTOR_IN4 = (gpio_num_t)CONFIG_PIN_MOTOR_IN4;

const gpio_num_t pin::ENCODER_LEFT = (gpio_num_t)CONFIG_PIN_ENCODER_LEFT;
const gpio_num_t pin::ENCODER_RIGHT = (gpio_num_t)CONFIG_PIN_ENCODER_RIGHT;
const gpio_num_t pin::LINE_FOLLOWER_LEFT = (gpio_num_t)CONFIG_PIN_LINE_FOLLOWER_LEFT;
const gpio_num_t pin::LINE_FOLLOWER_RIGHT = (gpio_num_t)CONFIG_PIN_LINE_FOLLOWER_RIGHT;

const gpio_num_t pin::MPU_SCL = (gpio_num_t)CONFIG_PIN_MPU_SCL;
const gpio_num_t pin::MPU_SDA = (gpio_num_t)CONFIG_PIN_MPU_SDA;
const gpio_num_t pin::MPU_INT = (gpio_num_t)CONFIG_PIN_MPU_INT;

const gpio_num_t pin::CURRENT = (gpio_num_t)CONFIG_PIN_CURRENT;
