#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // "stay in lane"
          uint lane = 1;
          double target_speed = 48.0;
          int NWP = map_waypoints_x.size(); // should be 181 points
          vector<double> spline_x;
          vector<double> spline_y;
          vector<double> spline_s;
          int wp_idx = ClosestWaypoint(car_x, car_y, map_waypoints_x, map_waypoints_y);
          // select waypoints and s values for interpolation
          for (int i = wp_idx - 1; i <= wp_idx + 2; i++){
            // integer mod and division are implementation dependent
            // these are obviously invalid for i < -NWP
            int i_mod_NWP = (i<0) ? (i+NWP) : (i%NWP);
            int i_div_NWP = (i<0) ? -1 : (i/NWP);
            spline_s.push_back(map_waypoints_s[i_mod_NWP] + (i_div_NWP)*6945.554);
            spline_x.push_back(map_waypoints_x[i_mod_NWP] + (10 - 4 * lane) * map_waypoints_dx[i_mod_NWP]);
            spline_y.push_back(map_waypoints_y[i_mod_NWP] + (10 - 4 * lane) * map_waypoints_dy[i_mod_NWP]);
          }
          // interpolate between these points
          tk::spline s_x;
          s_x.set_points(spline_s, spline_x);
          tk::spline s_y;
          s_y.set_points(spline_s, spline_y);
          // generate appropriately spaced points ahead of the vehicle position
          // this implementation does not yet consider the vehicle's current position!
          double car_ss = (0!=wp_idx || car_s >= map_waypoints_s[0]) ? car_s : (car_s - 6945.554);
          std::cout << "car s: " << car_ss <<std::endl;
          int path_size = 100;
          double d_ps = (0.020 * 0.44704 * target_speed);
          for (int i = 0; i < path_size; i++){
            // this can be dropped
            double s = car_ss + i*d_ps;
            next_x_vals.push_back(s_x(s));
            next_y_vals.push_back(s_y(s));
          }

          // todo: slow down behind a vehicle in this lane
          // todo: adjust target lane to move around vehicles
          //       - implement European/ correct driving behaviour, with the vehicle keeping to the right wherever possible
          //       - implement chaotic & dangerous driving behaviour, with undertaking and lane switches in either direction for the vehicle to move ahead
          // todo: generate smooth trajectories (satisfying constraints)
          //       - when the vehicle has the wrong speed
          //       - when the vehicle is slightly off-spline (e.g. new waypoints)
          //       - in a lane change
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
