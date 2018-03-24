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

double clip(double value, double max_value){
    double new_value = value;
    if (value < -max_value){
        new_value = -max_value;
    } else if (value > max_value){
        new_value = max_value;
    }
    
    return new_value;
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  std::vector<double> coeffs{0.875299, 0.0, 0.3101};
  pid.Init(coeffs);
  
  // the second is the maximum cte allowed, decrease it to improve the quality
  // 0.107811 di: 0.059049 dd: 0.11979
  std::vector<double> d_coeffs{0.0707348, 0.038742, 0.0707348};
  pid.InitTwiddle(false, 0.01, 1.4, 100, 10000, d_coeffs);
  
  
  PID speed_pid;
  std::vector<double> coeffs2{0.2, 0, 0.001};
  speed_pid.Init(coeffs2);

  h.onMessage([&pid, &speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          
          // angle can be used to adjust steer_value
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double exist_steer = deg2rad(angle);
          double steer_value;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          // tunning
          if (pid.tunning_finished_ == false){
          
              // twiddle on different set of parameters
              if (pid.run_finished_){

                  pid.Twiddle();
                  
                  // this will set run_finished_ to false
                  pid.ResetRun();
                  
                  // restart the simulator
                  std::string reset_msg = "42[\"reset\",{}]";
                  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                  
                  return;
                  
              } else {
                  pid.Run(cte);
              }
          }
          
          // normal run to emit control
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          steer_value = clip(steer_value, 1.0);
          
          // slow down when doing sharp turn
          double target_speed = 30.*(1.-abs(steer_value)) + 20.;
          double throttle_value;
          speed_pid.UpdateError(speed - target_speed);
          throttle_value = speed_pid.TotalError();
          throttle_value = clip(throttle_value, 1.0);
          
          /*
          // DEBUG
          std::cout << "CTE: " << cte << std::endl;
          std::cout << "Steering Value: " << steer_value << std::endl;
          std::cout << "Angle Value: " << exist_steer << std::endl;
          */
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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
