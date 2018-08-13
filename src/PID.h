#include <uWS/uWS.h>

#ifndef PID_H
#define PID_H

#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle
  */
  bool twiddle_standby_flag;
  int twiddle_state;  // 0 = not twiddling, 1 = twiddling, 2 = standby
  int current_gain;
  int gain_state[3];
  int cycle;
  int cycle_trans;
  int cycle_max;
  int twiddle_iteration;
  double best_err;
  double current_err;
  double tol;
  std::vector<double> p;
  std::vector<double> dp;
  uWS::WebSocket<uWS::SERVER>* ws;

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

  /*
  * Calculate the total PID error.
  */
  double TotalError(double cte);

  /*
  * Do the twiddle
  */
  void Twiddle(double err);

  /*
  * Clear the PID error.
  */
  void ClearError();

  /*
  * Restarts the simulator. Taken from the CarND Slack Forum.
  */
  //  void Restart(uWS::WebSocket<uWS::SERVER> ws);
  void Restart(uWS::WebSocket<uWS::SERVER>* ws);
};

#endif /* PID_H */
