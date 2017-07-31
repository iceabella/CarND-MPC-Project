#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  
  // define maximum values for steering and throttle
  const double max_steer = 1.0;
  const double max_throttle = 1.0;
  
  // initialize variable to store previous time stamp
  std::chrono::time_point<std::chrono::system_clock> prev_time;
  bool prev_meas = false;

  h.onMessage([&mpc, &max_steer, &max_throttle, &prev_time, &prev_meas](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];
          
          // Initialize time value          
          if(prev_meas){
            // define start time as now
            prev_time = std::chrono::system_clock::now();
            prev_meas = true;
          }
          
          // Get time interval between measurements (latency) 
          std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-prev_time;
          double dt_latency = elapsed_seconds.count();
          std::cout << "dt_lat real: " << dt_latency << std::endl;
          
          // save time stamp of this data to be used in next cycle
          prev_time = std::chrono::system_clock::now();
          
          // Convert speed from mph to m/s
          v *= 0.44704;
          
          // convert steer value to radians
          double delta = steer_value*deg2rad(25);
          
          // acceleration is estimated to be the same as throttle
          double a = throttle_value;
          
          // Transform waypoints in map coordinates to vehicle coordinates
          Eigen::VectorXd ptsx_v(ptsx.size());
          Eigen::VectorXd ptsy_v(ptsy.size());
          for(size_t i=0; i<ptsx.size(); i++){
            ptsx_v[i] = (ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) * sin(-psi);
            ptsy_v[i] = (ptsx[i] - px) * sin(-psi) + (ptsy[i] - py) * cos(-psi);
          }
                    
          // fit a 3rd order polynomial to the x and y coordinates
          Eigen::VectorXd coeffs = polyfit(ptsx_v, ptsy_v, 3);
          // calculate the cross track error
          double cte = polyeval(coeffs,0); // f(x) - y, where (x,y) = (0,0) since local coordinates
          // get the derivative f'(x)
          Eigen::VectorXd fp_coeffs(coeffs.size()-1);
          for (int i = 0; i < coeffs.size()-1; i++) 
            fp_coeffs(i) = coeffs(i+1)*(i+1);
          // calculate orientation error 
          double epsi = - atan(polyeval(fp_coeffs,0));  // epsi = psi - atan(f'(x)) where local heading is 0 and evaluated in point x=0
          
          // Updated states for x,y,psi because of vehicle coordinate system
          px = 0;
          py = 0;
          psi = 0;
          
          // UPDATE INITIAL STATE BECAUSE OF LATENCY
          double Lf = 2.67; // length from front to CoG
          
          double px_latency = px + v * cos(psi) * dt_latency;
          double py_latency = py + v * sin(psi) * dt_latency;
          double psi_latency = psi - v/Lf * delta * dt_latency;
          double v_latency = v + a * dt_latency;
          double cte_latency = cte + v * sin(epsi) * dt_latency;
          double epsi_latency = epsi - v/Lf * delta * dt_latency;  
          
          Eigen::VectorXd state(6);
          state << px_latency, py_latency, psi_latency, v_latency, cte_latency, epsi_latency;
          
          // solve MPC problem    
          vector<double> vars = mpc.Solve(state, coeffs);
          
          // get the updated actuation values
          steer_value = vars[0];
          throttle_value = vars[1];

          // normalize steering values to get values between [-1, 1] instead of [-deg2rad(25), deg2rad(25)]
          steer_value = steer_value/deg2rad(25);
          // Make sure steering values are within min and max limit
          if(steer_value > max_steer)
            steer_value = max_steer;
          if(steer_value < -max_steer)
            steer_value = -max_steer;
            
          // Make sure trottle values are within min and max limit
          if(throttle_value > max_throttle)
            throttle_value = max_throttle;
          if(throttle_value < -max_throttle)
            throttle_value = -max_throttle;

          json msgJson;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for(size_t i=2; i<vars.size(); i+=2){
            // transform point because of latency
            double xmpc_latency = (vars[i] - px_latency) * cos(-psi_latency) - (vars[i+1] - py_latency) * sin(-psi_latency);
            double ympc_latency = (vars[i] - px_latency) * sin(-psi_latency) + (vars[i+1] - py_latency) * cos(-psi_latency);       
            mpc_x_vals.push_back(xmpc_latency);
            mpc_y_vals.push_back(ympc_latency);
          }
          
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          
          // Add 20 points with 3.0 between each x-value
          for(double i = 1.0; i < 20; i ++){
          double x_line = i*3.0;
            double y_line = polyeval(coeffs,x_line);
            // transform point because of latency
            double xline_latency = (x_line - px_latency) * cos(-psi_latency) - (y_line - py_latency) * sin(-psi_latency);
            double yline_latency = (x_line - px_latency) * sin(-psi_latency) + (y_line - py_latency) * cos(-psi_latency);
            next_x_vals.push_back(xline_latency);
            next_y_vals.push_back(yline_latency);
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
