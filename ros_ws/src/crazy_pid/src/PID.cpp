#include <PID.h>
#include <algorithm>

PID::PID(float kp, float ki, float kd)
: kp(kp),
  ki(ki),
  kd(kd),
  i(0.0),
  prev(0.0),
  i_limit(1.0)
{}

void PID::process(float in, float &out)
{
    float p, d;

    p = in * kp;
    i = i + (in * ki);
    i = std::max(std::min(i, i_limit), 0.f);
	//i = 0.5;
    d = (in - prev) * kd;

    prev = in;

    out = p+i+d;
}
