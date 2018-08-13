#include "PID.h"
#include <math.h>
#include <numeric>
#include <iostream>
#include <cfloat>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // Initial gains
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  // Initial errors
  ClearError();

  // Twiddle hyperparameters
  dp = { 0.3 * Kp, 0.3 * Ki, 0.3 * Kd };
  tol = 0.005;
  cycle_trans = 200;
  cycle_max = 2000;

  // Init twiddle state machine
  best_err = DBL_MAX;
  cycle = 0;
  current_err = 0;
  current_gain = 0;
  gain_state[0] = 0;
  gain_state[1] = 0;
  gain_state[2] = 0;
  twiddle_iteration = 0;
  twiddle_standby_flag = false;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  if (twiddle_state == 1){
    cycle += 1;

    // Update the error while ignoring some initial cycles
    if (cycle > cycle_trans) {
      current_err += pow(cte, 2);
    }

    // If cycle_max is reached and dp not within tolerance, run twiddle
    if (cycle >= cycle_max) {
      double sum_dp = dp[0] + dp[1] + dp[2];
      if (sum_dp > tol) {
        Twiddle(current_err);
        cycle = 0;
        twiddle_iteration += 1;
      }
      else {
        twiddle_state = 0;  // done twiddling
      }
    }
  }
}

double PID::TotalError(double cte) {  
  return pow(cte, 2);
}

void PID::Twiddle(double err)
{
  //  The control vector is the P, I, and D, gains
  p = { Kp, Ki, Kd };

  // State 0
  if (gain_state[current_gain] == 0) {
    p[current_gain] += dp[current_gain];      
    // Next cycle evaluate this gain
    gain_state[current_gain] = 1;
  }
  // State 1
  else if (gain_state[current_gain] == 1) {
    if (err < best_err) {
      best_err = err;
      dp[current_gain] *= 1.1;
      // Next cycle move onto the next gain
      gain_state[current_gain] = 0;
      current_gain += 1;
    }
    else {
      p[current_gain] -= 2 * dp[current_gain];
      // Next cycle evaluate this gain
      gain_state[current_gain] = 2;
    }
  }
  // State 2
  else if (gain_state[current_gain] == 2) {
    if (err < best_err) {
      best_err = err;
      dp[current_gain] *= 1.1;
    }
    else {
      p[current_gain] += dp[current_gain];
      dp[current_gain] *= 0.9;
    }
    // Next cycle move onto the next gain
    gain_state[current_gain] = 0;
    current_gain += 1;
  }

  // Cycle gain counter
  if (current_gain == 3) {
    current_gain = 0;
    twiddle_state = 2;  // go to standby
    twiddle_standby_flag = true;  // set flag high to signal swap
  }

  // Set gains and reset simulator
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];

  Restart(ws);
}

void PID::ClearError() {
  p_error = i_error = d_error = current_err = 0;
}
/*
void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
*/
void PID::Restart(uWS::WebSocket<uWS::SERVER>* ws) {
std::string reset_msg = "42[\"reset\",{}]";
ws->send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
