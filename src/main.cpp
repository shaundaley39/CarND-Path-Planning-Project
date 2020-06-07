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
           * todo: adjust target lane to move around vehicles
           *       - already implemented a simplified European approach, staying to the right and overtaking on the left but not caring about undertaking/ speed of vehicles to the left
           *       - todo: implement chaotic & dangerous driving behaviour, with undertaking and lane switches in either direction for the vehicle to move ahead
           * todo: generate smooth trajectories (satisfying constraints)
           *       - acceleration towards target speed
           *       - when the vehicle is slightly off-spline (whether due to new waypoints, or due to a lane change)
          */

          int lane = (12.0 - (double)car_d)/4.0;
          double target_speed_mph = 48.0;
          double target_speed_mps = 0.44704 * target_speed_mph;

          // lane model
          bool lane_accessible[3] = {true, true, true};
          double lane_speed[3] = {target_speed_mps, target_speed_mps, target_speed_mps};
          double lane_s_limit[3] = {car_s + 1000, car_s + 1000, car_s + 1000};
          for (auto el : sensor_fusion){
            int lane_idx = (12.0 - (double)el[6])/4.0;
            double track_speed = pow(pow(el[3],2.0) + pow(el[4],2.0), 0.5);
            double s = el[5];
            if (car_s < 3500 && s > 5500) {s -=6945.554;}
            if (car_s > 5500 && s < 3500) {s +=6945.554;}
            if (s > car_s && s < lane_s_limit[lane_idx]){
              lane_s_limit[lane_idx] = s;
              lane_speed[lane_idx] = track_speed;
            }
            double s_diff = car_s - s;
            if (s_diff > -8.0 && s_diff < 8.0 + 3.0*std::max(0.0, track_speed - car_speed)){
              lane_accessible[lane_idx] = false;
            }
          }
          double lane_s_free[3];
          for (int i=0; i<3; i++){
            lane_s_free[i] = std::max(lane_s_limit[i] - 12, lane_s_limit[i] - 12 - 1.6*(car_speed - lane_speed[i]));
          }
          // lane change?
          /*
            Aggressive European approach:
            - stays to the right wherever possible
            - moves left to overtake, where necessary and possible
            - doesn't worry about undertaking/ going faster than vehicles to the left of it
            This is not optimal in a free-for-all US highway. It would be optimal if everyone followed the same rules.
          */
          if (lane>0){
            if (lane_accessible[lane-1] && (lane_s_free[lane-1] - car_s > 4 || lane_s_limit[lane-1] < car_s)){
              lane--;
            }
          }
          if (lane_s_free[lane] - car_s < 2){
            if (lane<2){
              if (lane_accessible[lane+1] && lane_s_free[lane+1] > lane_s_free[lane]){
                lane++;
              }
            }
          }
          // speed adjustment to avoid rear-ending a vehicle in front?
          if (lane_s_free[lane] - car_s < 0){
            target_speed_mps = std::min(target_speed_mps,
                                        lane_speed[lane] + 
                                        (target_speed_mps - lane_speed[lane])*
                                        ((car_s - lane_s_free[lane])/( lane_s_limit[lane] - lane_s_free[lane])));
          }
          // this is a crude sort of conditional PI controller to avoid a collision with the vehicle in front
          if (lane_s_limit[lane] - car_s < 12.0){
            target_speed_mps = std::min(target_speed_mps, lane_speed[lane] - (12 - (lane_s_limit[lane] - car_s)));
          }
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
          double car_ss = (0!=wp_idx || (car_s >= map_waypoints_s[0] && car_s < 4500)) ? car_s : (car_s - 6945.554);
          int path_size = 100;
          double d_ps = (0.020 * target_speed_mps);
          for (int i = 0; i < path_size; i++){
            double s = car_ss + i*d_ps;
            next_x_vals.push_back(s_x(s));
            next_y_vals.push_back(s_y(s));
          }

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
