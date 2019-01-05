#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>
#include <cstdio>

using namespace std;
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

  PID pid_steer;
  // TODO: Initialize the pid variable.

  PID pid_throt;

  pid_steer.Init(0.0620,0.00001,1.0);//0.09,0.00001,0.6

  //0.0560,0.00001,1.36 -50
  //0.187.0 ,0.000107,2.237 

  //0.17,0.0,2.1715

  //0.1408,0.0,1.815
  //0.15,0.0,1.8,0.3

  // 0.2,0.000001,0.8,0.15
  //0.15,0.000001,0.8,0.15

  //0.093505,1.9e-08,0.305711 , 0.2
  //0.05,0.0,0.7 ,0.3

  // 0.109,0.0000001,0.306,0.2


  //0.145,0.000001,0.336 --->0.4
  //0.36,0.0,0.012

  //0.175,0.000001,0.406 --->0.5
  //0.435,0.0,0.0145


  pid_throt.Init(0.35,0.0,2.95);
  //pid_throt.Init(0.0,0.0,0.0);

  h.onMessage([&pid_steer,&pid_throt](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value;
          double throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          //steer PID twiddle
          if (pid_steer.num_echo <= 20 || (pid_steer.dp[0] + pid_steer.dp[1] + pid_steer.dp[2]) > 0.02) {

          	//pid_steer.Twiddle(cte,6);         	

          }

          //throttle PID twidle
          if(pid_throt.num_echo <= 20 || (pid_throt.dp[0] + pid_throt.dp[1] + pid_throt.dp[2]) > 0.02){

          	//pid_throt.Twiddle(fabs(cte),5);
          
          }

          //PID update Error
          pid_steer.UpdateError(cte);
          pid_throt.UpdateError(fabs(cte));

          // calculate the steer PID total error
          steer_value = -pid_steer.Kp_ * pid_steer.p_error - pid_steer.Ki_ * pid_steer.i_error - pid_steer.Kd_ * pid_steer.d_error;

          // calculate the throttle PID total error
          throttle_value = 0.8 - pid_throt.Kp_ * pid_throt.p_error - pid_throt.Ki_ * pid_throt.i_error - pid_throt.Kd_ * pid_throt.d_error;

          // set the throttle value in [0.1,1.0]
          if (throttle_value <= 0.1) {

          	  throttle_value = 0.1;
          }else if (throttle_value >= 1.0) {

          	  throttle_value = 1.0;
          }


          //std::cout << "The current throttle is : " << throttle_value << std::endl;

          // steer value is in [-1,1]
          if (steer_value >= 1.0){
          	steer_value = 1.0;
          }else if (steer_value <= -1.0) {
          	steer_value = -1.0;
          }

          //print the cte steer PID total error
          ofstream outFile("log1.txt",ios::app);
      	  if (outFile.is_open()){
			  outFile << "cte: " << cte << " steer: " << steer_value << " totalerror: "<< pid_steer.TotalError() << " throttle: "<< throttle_value <<endl;
			  outFile.close();

      	  }

      	  //Debug
		  //std::cout << "speed :" << speed << " angle :" << angle << std::endl;         
          std::cout << "PID error :" << "P_error:"<<pid_steer.p_error <<" I_error:" << pid_steer.i_error << " D_error:" << pid_steer.d_error <<" TotalError: "<<pid_steer.TotalError()<< std::endl;

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;//0.3;
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
