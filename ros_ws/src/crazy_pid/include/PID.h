#ifndef PID_H
#define PID_H

class PID
{
public:
    PID(float kp, float ki, float kd);
    void process(float in, float &out);
private:
    float kp, ki, kd;
    float i, prev, i_limit;
};

#endif
