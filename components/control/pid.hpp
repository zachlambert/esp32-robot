#ifndef PID_H
#define PID_H

#include "elapsed_timer.hpp"

class PidController {
public:
    PidController(double kp, double ki, double kd, double initial_pv = 0):
        kp(kp), ki(ki), kd(kd) {}
    void loop(double measured_pv);
    double get_cv()const { return cv; }
private:
    double kp, ki, kd;
    double pv; // Process variable
    double sp; // Setpoint
    double cv; // Control variable
    double e; //Error
    double e_prev;
    double e_integral;
    double e_derivative;
    ElapsedTimer timer;
};

#endif
