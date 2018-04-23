#include "PID.h"
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  params = {Kp, Ki, Kd};

  iterations = 0;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  error = 0.0;
  best_error = __DBL_MAX__;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  if (iterations > twiddle_iteration_max) {
    error += pow(cte, 2);
  }
}
