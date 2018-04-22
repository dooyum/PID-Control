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
/* Constants */
double const max_steering_angle = 1;
double const min_steering_angle = -1;
double const max_throttle = 1.0;
double const max_speed = 100;

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

  PID pid;

  pid.Init(0.18, 0.001, 1.0);
  pid.enable_twiddle = false;
  pid.diff_params = {1.0, 0.0, 10.0};
  // Number of iterations after which Twiddle should be run.
  pid.twiddle_iteration_max = 200;
  // Begin Twiddle optimization with "P" coefficient.
  pid.twiddle_param = 0;
  pid.is_twiddle_coeff_down = false;
  pid.best_error = __DBL_MAX__;
  //  pid.Init(3.11, 0.0, 82.15);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          pid.iterations++;

          // Check for error and tune parameters using Twiddle algorithm.
          double diff_params_sum = pid.diff_params[0] + pid.diff_params[1] + pid.diff_params[2];
          if (pid.enable_twiddle && pid.iterations > pid.twiddle_iteration_max && diff_params_sum > 0.02) {

            // Twiddle
            if(!pid.is_twiddle_coeff_down) {
              pid.params[pid.twiddle_param] += pid.diff_params[pid.twiddle_param];
              pid.UpdateError(cte);
            }

            // Set the best error to the first error.
            if (pid.best_error == __DBL_MAX__) {
              pid.best_error = pid.error;
            }

            if (pid.error < pid.best_error) {
              pid.best_error = pid.error;
              pid.diff_params[pid.twiddle_param] *= 1.1;
              pid.is_twiddle_coeff_down = false;
              pid.twiddle_param = (pid.twiddle_param + 1) % pid.params.size();
            } else {
              if(pid.is_twiddle_coeff_down) {
                pid.is_twiddle_coeff_down = false;
                pid.params[pid.twiddle_param] += pid.diff_params[pid.twiddle_param];
                pid.diff_params[pid.twiddle_param] *= 0.9;
                pid.twiddle_param = (pid.twiddle_param + 1) % pid.params.size();
              } else {
                pid.is_twiddle_coeff_down = true;
                pid.params[pid.twiddle_param] -= 2 * pid.diff_params[pid.twiddle_param];
                pid.UpdateError(cte);
              }
            }
            // Restart simulator with tuned parameters.
            pid.Init(pid.params[0], pid.params[1], pid.params[2]);
            std::string reset_msg = "42[\"reset\",{}]";
            std::cout << "Restart!" << std::endl;
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
          } else {
            pid.UpdateError(cte);
          }

          double steer_value = -pid.params[0] * pid.p_error - pid.params[2] * pid.d_error - pid.params[1] * pid.i_error;

          if (steer_value > max_steering_angle)
            steer_value = max_steering_angle;
          if (steer_value < min_steering_angle)
            steer_value = min_steering_angle;

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Speed: " << speed << std::endl;

          double speed_to_desired_ratio = max_throttle - (speed / max_speed) * max_throttle;
          // Increase throttle on straight roads and decrease around bends.
          double throttle = speed_to_desired_ratio;
          if (fabs(steer_value) < max_steering_angle/10) {
            throttle *= 0.8;
          } else if(fabs(steer_value) < max_steering_angle/5) {
            throttle *= 0.4;
          } else if(fabs(steer_value) < max_steering_angle) {
            throttle *= 0.2;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
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
