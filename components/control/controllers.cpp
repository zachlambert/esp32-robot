#include "controllers.hpp"
#include "esp_log.h"

static const char *TAG = "PID";

void IntegralController::update(float measured_pv)
{
    pv = measured_pv;

    e = sp - pv;
    kie_integral += ki * (e_prev + e)*0.5 * dt;

    if (kie_limit != 0) {
        if (kie_integral > kie_limit) {
            kie_integral = kie_limit;
        } else if (kie_integral < -kie_limit) {
            kie_integral = -kie_limit;
        }
    }

    cv = kie_integral;

    ESP_LOGI(TAG, "e (%f) | kiei (%f) | cv (%f)", e, kie_integral, cv);
}


void PidController::update(float measured_pv)
{
    pv = measured_pv;

    e = sp - pv;
    e_derivative = (e - e_prev) / dt;
    kie_integral += ki * (e_prev + e)*0.5 * dt;
    e_prev = e;

    if (kie_limit != 0) {
        if (kie_integral > kie_limit) {
            kie_integral = kie_limit;
        } else if (kie_integral < -kie_limit) {
            kie_integral = -kie_limit;
        }
    }

    cv = kp*e + kie_integral + kd*e_derivative;
}
