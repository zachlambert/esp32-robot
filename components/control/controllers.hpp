#ifndef PID_H
#define PID_H

class Controller {
public:
    Controller(float dt):dt(dt) {}
    virtual void update(float measured_pv) = 0;
    void set_sp(float sp){ this->sp = sp; }
    float get_cv()const{ return cv; }
protected:
    float dt;
    float pv;
    float sp;
    float cv;
    float e;
};

class PidController : public Controller{
public:
    PidController(float dt, float kp, float ki, float kd, float kie_limit = 0):Controller(dt),
        kp(kp), ki(ki), kd(kd), kie_limit(kie_limit){}
    void update(float measured_pv);
private:
    float kp, ki, kd;
    float e_prev;
    float kie_integral;
    float e_derivative;
    const float kie_limit;
};

#endif
