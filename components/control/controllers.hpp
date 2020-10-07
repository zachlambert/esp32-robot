#ifndef PID_H
#define PID_H

class Controller {
public:
    Controller(float dt):dt(dt) {}
    virtual ~Controller() {}
    virtual void update(float measured_pv) = 0;
    void set_sp(float sp){ this->sp = sp; }
    float get_sp()const{ return sp; }
    float get_cv()const{ return cv; }
protected:
    float dt;
    float pv;
    float sp;
    float cv;
    float e;
    float e_prev;
};

class IntegralController: public Controller {
public:
    IntegralController(float dt, float ki, float kie_limit = 0):Controller(dt), ki(ki) {}
    void update(float measured_pv);
private:
    float ki;
    float kie_integral;
    float kie_limit;
};

class PidController : public Controller {
public:
    PidController(float dt, float kp, float ki, float kd, float kie_limit = 0):Controller(dt),
        kp(kp), ki(ki), kd(kd), kie_limit(kie_limit){}
    void update(float measured_pv);
private:
    float kp, ki, kd;
    float kie_integral;
    float e_derivative;
    const float kie_limit;
};

#endif
