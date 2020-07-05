#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"
#include "trajectory.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

enum CarState {cruise, tail};


int main() {
  uWS::Hub h;

  // state at last planned trajectory point
  double last_wp_s = 0;
  double last_wp_vs = 0;
  double last_wp_as = 0;
  // and s values for planned trajectory points
  vector<double> path_s = {};

  int target_lane = 1;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  const double max_s = 6945.554;

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

  // construct an array of 2D (cubic spline) paths for the 3-lane highway we want to drive along
  vector<double> ospline_s;
  const int D=2;
  vector<std::array<double, D>> ospline[3];
  for(int i=0; i< map_waypoints_x.size(); i++){
    ospline_s.push_back(map_waypoints_s[i]);
    for(int l=0; l<3; l++){
      double x = map_waypoints_x[i] + (10.0 - 4.0 * l) * map_waypoints_dx[i];
      double y = map_waypoints_y[i] + (10.0 - 4.0 *l) * map_waypoints_dy[i];
      ospline[l].push_back({x,y});
    }
  }
  for(int i=0; i< 0.1*map_waypoints_x.size(); i++){
    ospline_s.push_back(max_s + map_waypoints_s[i]);
    for(int l=0; l<3; l++){
      double x = map_waypoints_x[i] + (10.0 - 4.0 * l) * map_waypoints_dx[i];
      double y = map_waypoints_y[i] + (10.0 - 4.0 *l) * map_waypoints_dy[i];
      ospline[l].push_back({x,y});
    }
  }
  // we will be computing paths and trajectories on a 2 dimensional plane
  tk::spline<D> os[3];
  for(int l=0; l<3; l++){
    os[l].set_points(ospline_s, ospline[l], false);
  }

  // repeat for each new packet of sensor fusion data
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, max_s, &last_wp_s, &last_wp_vs, &last_wp_as, &path_s, &os, &target_lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    const int D=2;
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
          CarState car_state = cruise;

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

          for (double x : previous_path_x){
            next_x_vals.push_back(x);
          }
          for (double y : previous_path_y){
            next_y_vals.push_back(y);
          }
    
          // How long is the remaining path? Update path_s
          if (path_s.size() > previous_path_x.size()){
            path_s.erase(path_s.begin(), path_s.begin() + path_s.size()-previous_path_x.size());
          }
          int path_s_size_prev = path_s.size();
          if(0==path_s_size_prev){
            last_wp_s = car_s;
            last_wp_vs = car_speed;
          } else {
            double offset = car_s - path_s[0];
            for (int i=0; i<path_s.size(); i++){
              path_s[i] += offset;
            }
            last_wp_s += offset;
          }

          // current car_lane update
          int car_lane = (12.0 - car_d)/4.0;

          double target_speed_mph = 49.0;
          double target_speed_mps = 0.44704 * target_speed_mph;

          // Update lane model
          double car_s_f = (path_s.size() == 0)? car_s : path_s[path_s.size()-1];
          double detla_t = 0.02*std::max(0.0, (double)(path_s.size()-1));
          bool lane_accessible[3] = {true, true, true};
          double lane_speed[3] = {target_speed_mps, target_speed_mps, target_speed_mps};
          double lane_s_limit[3] = {car_s + 2000, car_s + 2000, car_s + 2000};
          for (auto el : sensor_fusion){
            int lane_idx = (12.0 - (double)el[6])/4.0;
            double track_speed = std::max(0.0, pow(pow(el[3],2.0) + pow(el[4],2.0), 0.5));
            double s = (double)el[5] + detla_t*track_speed; // projecting time forward to end of the previous path
            double so = (double)el[5]; // without projection
            if (car_s < 3500 && s > 5500) {s -=max_s; so -=max_s;}
            if (car_s > 5500 && s < 3500) {s +=max_s; so +=max_s;}
            if (s > car_s_f && s < lane_s_limit[lane_idx]){
              lane_s_limit[lane_idx] = s;
              if (0.0 <= track_speed && track_speed < target_speed_mps){
                lane_speed[lane_idx] = track_speed;
              }
            }
            double s_diff = car_s_f - s;
            double so_diff = car_s - so;
            if ((s_diff > -12.0 && s_diff < 12.0) || (so_diff > -12.0 && so_diff < 12.0)){
              lane_accessible[lane_idx] = false;
            }
          }
          double lane_s_free[3];
          for (int i=0; i<3; i++){
            lane_s_free[i] = std::min(lane_s_limit[i] - 20, lane_s_limit[i] - 20 - 1.6*fabs(last_wp_vs - lane_speed[i]));
          }

          // State transition: cruise, tail
          int points_to_generate = 32 - path_s_size_prev;

          if (target_lane == car_lane){
            if (car_s_f + 10 < lane_s_limit[car_lane]){
              if (car_lane > 0 && last_wp_vs > 10.0 && lane_accessible[car_lane-1] && (lane_s_free[car_lane-1] - car_s_f > 14 || lane_s_limit[car_lane-1] < car_s_f)){
                target_lane = car_lane -1;
              } else if (car_lane<2 && lane_accessible[car_lane+1] && lane_s_free[car_lane+1] > lane_s_free[car_lane] && lane_s_free[car_lane] - car_s_f < 4.0){
                target_lane = car_lane + 1;
              }
            }
          }

          if (lane_s_free[target_lane] - car_s_f < 0 || (16.0/last_wp_vs)*std::max(0.0, last_wp_vs-lane_speed[car_lane]) > std::max(0.0, (lane_s_limit[car_lane] - car_s_f)) ){
            car_state = tail;
          } else {
            car_state = cruise;
          }

          // generate a path, combining the car's prior trajectory and the path for the target lane
          tk::spline<D> nos;
          double path_offset = (target_lane==car_lane)? 16.0 : 50.0;
          std::vector<std::array<double, D>> npath_points;
          std::vector<double> npath_s;
          int inserted=0;
          if(path_s.size()==0){
            npath_points.push_back({car_x, car_y});
            npath_s.push_back(car_s);
            inserted++;
          } else {
            for(int i=0; i<path_s.size(); i++){
              if(npath_s.size()==0 || path_s[i] > 0.05 + npath_s[npath_s.size()-1]){
                npath_points.push_back({previous_path_x[i],previous_path_y[i]});
                npath_s.push_back(path_s[i]);
                inserted++;
              }
            }
          }
          double s= (0==path_s.size())? car_s : path_s[path_s.size()-1];
          s += path_offset;
          for(int i=0; i<5; i++){
            npath_points.push_back({os[target_lane](s)[0], os[target_lane](s)[1]});
            npath_s.push_back(s);
            s+=16.0;
          }
          nos.set_points(npath_s, npath_points);
          nos.normalize_path(inserted-1);

          // generate a smooth trajectory along that path, while avoiding risk of collision with other vehicles on that path
          std::vector<std::vector<double>> trajectory_s;
          switch(car_state){
            case tail:
            {
              target_speed_mps = std::min(lane_speed[target_lane], lane_speed[car_lane]);
              double dt = 5;
              double target_s = std::min( lane_s_limit[target_lane] + dt*lane_speed[target_lane] - 16.0, lane_s_limit[car_lane] + dt*lane_speed[car_lane] - 16.0);
              target_s = std::min(target_s, last_wp_s + dt*0.6*(last_wp_vs+target_speed_mps));
              target_s = std::max(target_s, last_wp_s + pow(last_wp_vs, 2.0)/(10*dt));
              trajectory_s = getTailTrajectory({last_wp_s, last_wp_vs, last_wp_as}, {target_s, target_speed_mps, 0.0}, dt);
              int i_wp;
              for (i_wp = 0; i_wp <  std::min((int)trajectory_s.size(), points_to_generate); i_wp++){
                double wp_s = trajectory_s[i_wp][0];
                double wp_x = nos(wp_s)[0];
                double wp_y = nos(wp_s)[1];
                path_s.push_back(wp_s);
                next_x_vals.push_back(wp_x);
                next_y_vals.push_back(wp_y);
              }
              last_wp_s = trajectory_s[i_wp-1][0];
              last_wp_vs = trajectory_s[i_wp-1][1];
              last_wp_as = trajectory_s[i_wp-1][2];
              break;
            }
            case cruise:
            {
              trajectory_s = getTrajectory({last_wp_s, last_wp_vs, last_wp_as}, target_speed_mps, 6, 6);
              double p_s = last_wp_s;
              int i_wp;
              for (i_wp = 0; i_wp < std::min((int)trajectory_s.size(), points_to_generate); i_wp++){
                double wp_s = trajectory_s[i_wp][0];
                double wp_x = nos(wp_s)[0];
                double wp_y = nos(wp_s)[1];
                path_s.push_back(wp_s);
                next_x_vals.push_back(wp_x);
                next_y_vals.push_back(wp_y);
              }
              last_wp_s = trajectory_s[i_wp-1][0];
              last_wp_vs = trajectory_s[i_wp-1][1];
              last_wp_as = trajectory_s[i_wp-1][2];
              break;
            }
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
