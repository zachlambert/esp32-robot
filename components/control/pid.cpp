#include "pid.hpp"

void PidController::loop(double measured_pv)
{
    double dt = timer.sample_dt();
    pv = measured_pv;

    e = sp - pv;
    e_derivative = (e - e_prev) / dt;
    e_integral += (e_prev + e)*0.5 * dt;
    e_prev = e;

    cv = kp*e + ki*e_integral + kd*e_derivative;
}
