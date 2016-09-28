#include <PID.h>
#include <algorithm>

PID::PID(float kp, float ki, float kd)
: kp(kp),
  ki(ki),
  kd(kd),
  i(0.0),
  prev(0.0),
  i_limit(5.0)
{}

void PID::process(float in, float &out)
{
    float p, d;

    p = in * kp;
    i = (i + (in * ki)) / 2.0;
    i = std::min(i, i_limit);
    d = (in - prev) * kd;

    prev = in;

    out = p+i+d;
}
