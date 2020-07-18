#ifndef PID_H
#define PID_H

class PidController {
public:
    PidController(float dt, double kp, double ki, double kd, double initial_pv = 0):
        dt(dt), kp(kp), ki(ki), kd(kd) {}
    void loop(double measured_pv);
    double get_cv()const { return cv; }
    void set_sp(double sp){ this->sp = sp; }
private:
    float dt;
    double kp, ki, kd;
    double pv; // Process variable
    double sp; // Setpoint
    double cv; // Control variable
    double e; //Error
    double e_prev;
    double kie_integral;
    double e_derivative;
    const double kie_limit = 100.0;
};

#endif
