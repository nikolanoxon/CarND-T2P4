#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // TODO: Initialize the lat and long PID controllers.
  PID pid_lat;
  PID pid_long;

  pid_lat.Init(0.075, 0.0001, 3);
  pid_long.Init(0.1, 0.00001, .1);
  /*
STEERING Kp: 0.190967 Ki: 0.000159967 Kd: 2.1
STEERING Iteration: 460 Active Gain: 0 Gain State: 0
STEERING cycle: 1901 Current Error: 612.552*/

  //  Enable or Disable Twiddle
  bool so_you_want_to_twiddle = true;

  if (so_you_want_to_twiddle) {
    pid_lat.twiddle_state = 1;  //  active
    pid_long.twiddle_state = 1; //  off
  }
  else {
    pid_lat.twiddle_state = 0;  //  off
    pid_long.twiddle_state = 0; //  off
  }


  h.onMessage([&pid_lat, &pid_long](uWS::WebSocket<uWS::SERVER>* ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          // This allows the PID controllers to execute the reset command to the simulator
          pid_lat.ws = ws;
          pid_long.ws = ws;

          //  Grab the last iteration value
          int lat_iteration_old = pid_lat.twiddle_iteration;
          int long_iteration_old = pid_long.twiddle_iteration;

          pid_lat.UpdateError(cte);         //  Error of steering input is dependent on steering angle sign
          pid_long.UpdateError(fabs(cte));  //  Error of throttle input is independent of steering angle sign

          // DEBUG
          if (pid_lat.cycle % 100 == 1) {
            std::cout << "STEERING Kp: " << pid_lat.Kp << " Ki: " << pid_lat.Ki << " Kd: " << pid_lat.Kd << std::endl;
            std::cout << "STEERING Iteration: " << pid_lat.twiddle_iteration << " Active Gain: " << pid_lat.current_gain << " Gain State: " << pid_lat.gain_state[pid_lat.current_gain] << std::endl;
            std::cout << "STEERING cycle: " << pid_lat.cycle << " Current Error: " << pid_lat.current_err << std::endl;
          }
          else if (pid_long.cycle % 100 == 1) {
            std::cout << "THROTTLE Kp: " << pid_long.Kp << " Ki: " << pid_long.Ki << " Kd: " << pid_long.Kd << std::endl;
            std::cout << "THROTTLE Iteration: " << pid_long.twiddle_iteration << " Active Gain: " << pid_long.current_gain << " Gain State: " << pid_long.gain_state[pid_long.current_gain] << std::endl;
            std::cout << "THROTTLE cycle: " << pid_long.cycle << " Current Error: " << pid_long.current_err << std::endl;
          }

          // if the twiddling has move on to the next iteration, clear the error
          if ((pid_lat.twiddle_iteration > lat_iteration_old) || (pid_long.twiddle_iteration > long_iteration_old)) {
            pid_lat.ClearError();
            pid_long.ClearError();
          }

          // Switch the actively twiddling controller
          if (pid_lat.twiddle_standby_flag) {
            pid_lat.twiddle_standby_flag = false;
            if (pid_long.twiddle_state == 2) { pid_long.twiddle_state = 1; }
            else if (pid_lat.twiddle_state == 2) { pid_lat.twiddle_state = 1; }
          }
          if (pid_long.twiddle_standby_flag) { pid_long.twiddle_standby_flag = false;
            if (pid_lat.twiddle_state == 2) { pid_lat.twiddle_state = 1; }
            else if (pid_long.twiddle_state == 2) { pid_long.twiddle_state = 1; }
          }

          // PID Setpoints and Saturation Values
          double steer_target = 0.0;
          double throttle_target = 0.6;
          double steer_max = 20;  // degrees
          double steer_scale = 0.04;  // convert degrees to sim scaling [-1, 1] -> [-25, 25]

          // Lateral PID
          double steer_value = steer_target - (pid_lat.Kp * pid_lat.p_error + pid_lat.Ki * pid_lat.i_error + pid_lat.Kd * pid_lat.d_error);
          
          // Lateral PID Saturation
          if (steer_value > steer_max * steer_scale){
            steer_value = steer_max * steer_scale;
          }
          else if (steer_value < -steer_max * steer_scale) {
            steer_value = -steer_max * steer_scale;
          }
          
          // Longitudinal PID
          double throttle_value = throttle_target - (pid_long.Kp * pid_long.p_error + pid_long.Ki * pid_long.i_error + pid_long.Kd * pid_long.d_error);

          // DEBUG
//          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code, char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  auto host = "127.0.0.1";
  if (h.listen(host, port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
