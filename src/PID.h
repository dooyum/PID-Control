#ifndef PID_H
#define PID_H
#include <vector>

using namespace std;

class PID {
public:
  unsigned int iterations;

  /*
   * Twiddle
   */
  bool enable_twiddle;
  unsigned int twiddle_iteration_max;
  unsigned int twiddle_param;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double error;
  double best_error;

  /*
  * Coefficients
  */
  vector<double> params;
  vector<double> diff_params;
  bool is_twiddle_coeff_down;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
};

#endif /* PID_H */
