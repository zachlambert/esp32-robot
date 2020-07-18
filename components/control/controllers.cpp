#include "controllers.hpp"
#include "esp_log.h"

static const char *TAG = "PID";

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

    ESP_LOGI(
        TAG,
        "PV: %f | E: %f | ED: %f | KIEI: %f | CV: %f",
        pv, e, e_derivative, kie_integral, cv
    );
}
