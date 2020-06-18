#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "trajectory.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

enum CarState {cruise, tail, lane_change};


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
  const double max_s = 6945.554;

  // behavior
  CarState car_state = cruise;

  // state of last waypoint
  double last_p_s = 0;
  double last_wp_s = 0;
  double last_wp_vs = 0;
  double last_wp_as = 0;
  vector<double> path_s = {};

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

  // Spline generation
  int NWP = map_waypoints_x.size(); // should be 181 points
  vector<double> ospline_x[3];
  vector<double> ospline_y[3];
  vector<double> ospline_s;
  for(int i=0; i< NWP; i++){
    ospline_s.push_back(map_waypoints_s[i]);
    for(int l=0; l<3; l++){
      ospline_x[l].push_back(map_waypoints_x[i] + (10 - 4 * l) * map_waypoints_dx[i]);
      ospline_y[l].push_back(map_waypoints_y[i] + (10 - 4 *l) * map_waypoints_dy[i]);
    }
  }
  for(int i=0; i< 0.3*NWP; i++){
    ospline_s.push_back(max_s + map_waypoints_s[i]);
    for(int l=0; l<3; l++){
      ospline_x[l].push_back(map_waypoints_x[i] + (10 - 4 * l) * map_waypoints_dx[i]);
      ospline_y[l].push_back(map_waypoints_y[i] + (10 - 4 *l) * map_waypoints_dy[i]);
    }
  }
  tk::spline os_x[3];
  tk::spline os_y[3];
  for(int i=0; i<3; i++){
    os_x[i].set_points(ospline_s, ospline_x[i]);
    os_y[i].set_points(ospline_s, ospline_y[i]);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, max_s, &last_wp_s, &last_wp_vs, &last_wp_as, &path_s, &car_state, &os_x, &os_y]
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
          for (double x : previous_path_x){
            next_x_vals.push_back(x);
          }
          for (double y : previous_path_y){
            next_y_vals.push_back(y);
          }

          bool generate_new_path_points = (previous_path_x.size() < 16);
    
    // How long is the remaining path? Update path_s
          for (int i=0; i<path_s.size(); i++){
            if(car_s < 3500 && path_s[i] > 5500){
              path_s[i] -= max_s;
            }
          }
 //         std::cout << "previous path sizes, x,y,s: "<<previous_path_x.size()<<", "<< previous_path_y.size() << ", " << path_s.size() << std::endl;
    /* not as tidy as the following implementation
    for (int i=0; i<path_s.size(); i++){
            if(path_s[0] < car_s){
              path_s.erase(0);
      }
          }
    */
    
          if (path_s.size() > previous_path_x.size()){
            path_s.erase(path_s.begin(), path_s.begin() + path_s.size()-previous_path_x.size());
          }
//         std::cout << "copied sizes, x,y,w: " << next_x_vals.size() << ", " << next_y_vals.size() << ", " << path_s.size() << std::endl;
          int path_s_size_prev = path_s.size();
          if (path_s_size_prev > 0){
            std::cout<<"path s size: "<<path_s_size_prev<<" and previous path x size: "<<previous_path_x.size()<<std::endl;
            std::cout<<"car s: "<<car_s<<" and first path point s: "<<path_s[0]<<std::endl;
          }
          int car_lane = (12.0 - (double)car_d)/4.0;
          int target_lane = car_lane;
          double target_speed_mph = 46.0;
          double target_speed_mps = 0.44704 * target_speed_mph;
          if(0==path_s_size_prev){
            last_wp_s = car_s;
            last_wp_vs = car_speed;
          }

    
      //
      // Update lane model
      // - for car_s, car_lane, car_speed, t, is there a forward collision risk?
      // - for car_s, car_speed, t, lane, is the lane accessible?
      // - what is the lane_s, lane_v for tailing at time t?
      double car_s_f = (path_s.size() == 0)? car_s : path_s[path_s.size()-1];
      double detla_t = 0.02* path_s.size();
      bool lane_accessible[3] = {true, true, true};
      double lane_speed[3] = {target_speed_mps, target_speed_mps, target_speed_mps};
      double lane_s_limit[3] = {car_s + 2000, car_s + 2000, car_s + 2000};
      for (auto el : sensor_fusion){
        int lane_idx = (12.0 - (double)el[6])/4.0;
        double track_speed = pow(pow(el[3],2.0) + pow(el[4],2.0), 0.5);
        double s = (double)el[5] + detla_t*track_speed; // projecting time forward to end of the previous path
        if (car_s < 3500 && s > 5500) {s -=max_s;}
        if (car_s > 5500 && s < 3500) {s +=max_s;}
        if (s > car_s_f && s < lane_s_limit[lane_idx]){
          lane_s_limit[lane_idx] = s;
          if (0.0 <= track_speed && track_speed < target_speed_mps){
            lane_speed[lane_idx] = track_speed;
          }
        }
        double s_diff = car_s_f - s;
        if (s_diff > -8.0 && s_diff < 8.0 + 3.0*std::max(0.0, track_speed - last_wp_vs)){
          lane_accessible[lane_idx] = false;
        }
      }
      double lane_s_free[3];
      for (int i=0; i<3; i++){
        lane_s_free[i] = std::min(lane_s_limit[i] - 12, lane_s_limit[i] - 12 - 1.6*(last_wp_vs - lane_speed[i]));
      }

      // State transition: cruise, tail, laneshift
      //
      // if collision anticipated, discard path and tail
      //
      // if cruise or tail and space exists, laneshift to the right
      if (generate_new_path_points){
      //  std::cout<<"generating some path points"<< std::endl;
        car_state = cruise;
        if (lane_s_free[car_lane] - car_s_f < 0){
          car_state = tail;
        }
    if (car_lane > 0 && last_wp_vs > 10.0 && lane_accessible[car_lane-1] && (lane_s_free[car_lane-1] - car_s_f > 8 || lane_s_limit[car_lane-1] < car_s_f)){
      target_lane = car_lane -1;
      car_state = lane_change;
    } else if (car_lane<2 && lane_accessible[car_lane+1] && lane_s_free[car_lane+1] > lane_s_free[car_lane] && car_lane < 3 && lane_s_free[car_lane] - car_s_f < 2){
              target_lane = car_lane + 1;
              car_state = lane_change;
        }


        // now, either generate a lane change maneuver or generate 8 path points
        std::vector<std::vector<double>> trajectory_s;
        std::cout<<"switching"<< std::endl;
        
        switch(car_state){
          case lane_change:
          {
        std::cout<<"lane change"<< std::endl;
            std::vector<std::vector<double>> trajectory_lane = getLaneShift(0, 4, 2, 2);
            double dt = 4.2;
            // target_speed_mps = std::min(lane_speed[target_lane], last_wp_vs + 4.0);
            target_speed_mps -= 4.0;
            double target_s = last_wp_s + dt * (0.2*last_wp_vs + 0.8*target_speed_mps);
            trajectory_s = getTailTrajectory({last_wp_s, last_wp_vs, last_wp_as}, {target_s, target_speed_mps, 0.0}, dt);
            // this value ranges from 0 to 4, designating a weighted average of the former and target lanes
            int i_wp;
            for (i_wp = 0; i_wp < trajectory_lane.size(); i_wp++){
                double wp_s = trajectory_s[i_wp][0];
                double wp_x = 0.25*(4.0-trajectory_lane[i_wp][0]) * os_x[car_lane](wp_s) + 0.25*trajectory_lane[i_wp][0] * os_x[target_lane](wp_s);
                double wp_y = 0.25*(4.0-trajectory_lane[i_wp][0]) * os_y[car_lane](wp_s) + 0.25*trajectory_lane[i_wp][0] * os_y[target_lane](wp_s);
                path_s.push_back(wp_s);
                next_x_vals.push_back(wp_x);
                next_y_vals.push_back(wp_y);
              }
              last_wp_s = trajectory_s[i_wp-1][0];
              last_wp_vs = trajectory_s[i_wp-1][1];
              last_wp_as = trajectory_s[i_wp-1][2];
            
            break;
            // still to implement break;
          }
          case tail:
          {
        std::cout<<"tail"<< std::endl;
            target_speed_mps = std::min(target_speed_mps, lane_speed[car_lane]);
            if (fabs(last_wp_vs - target_speed_mps < 3)){
              double dt = 5;
              double target_s = lane_s_limit[car_lane] + dt*lane_speed[car_lane] - 10.0;
              if ((target_s - last_wp_s)/dt > (last_wp_vs + 3)){
                target_s = (last_wp_vs+3)*dt;
              }
              trajectory_s = getTailTrajectory({last_wp_s, last_wp_vs, last_wp_as}, {target_s, target_speed_mps, 0.0}, dt);
              std::cout<<"new tail"<< std::endl;
              int i_wp;
              for (i_wp = 0; i_wp < 24 - path_s_size_prev; i_wp++){
                double wp_s = trajectory_s[i_wp][0];
                double wp_x = os_x[car_lane](wp_s);
                double wp_y = os_y[car_lane](wp_s);
                path_s.push_back(wp_s);
                next_x_vals.push_back(wp_x);
                next_y_vals.push_back(wp_y);
              }
              last_wp_s = trajectory_s[i_wp-1][0];
              last_wp_vs = trajectory_s[i_wp-1][1];
              last_wp_as = trajectory_s[i_wp-1][2];

            } else {
              trajectory_s = getTrajectory({last_wp_s, last_wp_vs, last_wp_as}, target_speed_mps, 7,7);
              int i_wp;
              for (i_wp = 0; i_wp < trajectory_s.size(); i_wp++){
                double wp_s = trajectory_s[i_wp][0];
                double wp_x = os_x[car_lane](wp_s);
                double wp_y = os_y[car_lane](wp_s);
                path_s.push_back(wp_s);
                next_x_vals.push_back(wp_x);
                next_y_vals.push_back(wp_y);
              }
              last_wp_s = trajectory_s[i_wp-1][0];
              last_wp_vs = trajectory_s[i_wp-1][1];
              last_wp_as = trajectory_s[i_wp-1][2];
              double dt = 5;
              double target_s = lane_s_limit[car_lane] + (dt+0.02*trajectory_s.size())*lane_speed[car_lane] - 12.0;
              trajectory_s = getTailTrajectory({last_wp_s, last_wp_vs, last_wp_as}, {target_s, target_speed_mps, 0.0}, dt);
              for (i_wp = 0; i_wp < 24; i_wp++){
                double wp_s = trajectory_s[i_wp][0];
                double wp_x = os_x[car_lane](wp_s);
                double wp_y = os_y[car_lane](wp_s);
                path_s.push_back(wp_s);
                next_x_vals.push_back(wp_x);
                next_y_vals.push_back(wp_y);
              }
              last_wp_s = trajectory_s[i_wp-1][0];
              last_wp_vs = trajectory_s[i_wp-1][1];
              last_wp_as = trajectory_s[i_wp-1][2];
            }
/*
           if(gane_s_limit[ca = lane_s_limit[car_lane] + dt*lane_speed[car_lane] - 10.0;
              trajectory_s = getTailTrajectory({last_wp_s, last_wp_vs, last_wp_as}, {target_s, target_speed_mps, 0.0}, dt);
_lane]-car_s_f < 12){
              target_speed_mps -= fabs(0.1*(8-(lane_s_limit[car_lane]-car_s_f)));
              target_speed_mps = std::max(target_speed_mps, 0.0);
            }
*/

            break;
            }
          case cruise:
        std::cout<<"cruise"<< std::endl;
            trajectory_s = getTrajectory({last_wp_s, last_wp_vs, last_wp_as}, target_speed_mps, 7, 7);
        //    std::cout<<"trajectory size: "<< trajectory_s.size()<<std::endl;
          //  std::cout<<"final s,v,a: "<<trajectory_s[trajectory_s.size()-1][0] <<", "<<trajectory_s[trajectory_s.size()-1][1]<<", "<< trajectory_s[trajectory_s.size()-1][2]<<std::endl;
            double p_s = last_wp_s; //path_s[path_s.size()-1];
            int i_wp;/*
            std::cout<<"before:";
            for(int j=0; j<next_x_vals.size(); j++){
              std::cout<<"("<<path_s[j]<<", "<<next_x_vals[j]<<", "<<next_y_vals[j]<<"), ";
            }
            std::cout<<std::endl; */
            if( trajectory_s.size() < 25 - path_s_size_prev){
            //  std::cout<<"short trajectory: "<< trajectory_s.size()<<std::endl;
            }
            for (i_wp = 0; i_wp < std::min((int)trajectory_s.size(), 24 - path_s_size_prev); i_wp++){
              double wp_s = trajectory_s[i_wp][0];
              double wp_x = os_x[car_lane](wp_s);
              double wp_y = os_y[car_lane](wp_s);
              path_s.push_back(wp_s);
              next_x_vals.push_back(wp_x);
              next_y_vals.push_back(wp_y);
            }/*
            std::cout<<"after:";
            for(int j=0; j<next_x_vals.size(); j++){
              std::cout<<"("<<path_s[j]<<", "<<next_x_vals[j]<<", "<<next_y_vals[j]<<"), ";
            }
            std::cout<<std::endl; */
            last_wp_s = trajectory_s[i_wp-1][0];
            last_wp_vs = trajectory_s[i_wp-1][1];
            last_wp_as = trajectory_s[i_wp-1][2];
           // std::cout<<"final s, v, a: "<<last_wp_s<<", "<<last_wp_vs<<", "<<last_wp_as<<std::endl;
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
