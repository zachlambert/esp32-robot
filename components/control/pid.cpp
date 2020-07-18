#include "pid.hpp"
#include "esp_log.h"

static const char *TAG = "PID";

void PidController::loop(double measured_pv)
{
    pv = measured_pv;

    e = sp - pv;
    e_derivative = (e - e_prev) / dt;
    kie_integral += ki * (e_prev + e)*0.5 * dt;
    if (kie_integral > kie_limit) {
        kie_integral = kie_limit;
    } else if (kie_integral < -kie_limit) {
        kie_integral = -kie_limit;
    }
    e_prev = e;

    cv = kp*e + kie_integral + kd*e_derivative;

    ESP_LOGI(
        TAG,
        "PV: %f | E: %f | ED: %f | KIEI: %f | CV: %f",
        pv, e, e_derivative, kie_integral, cv
    );
}
