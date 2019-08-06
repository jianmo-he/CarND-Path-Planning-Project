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

  static int lane = 1;
  static double ref_vel = 0.0;

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

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          int prev_size = previous_path_x.size();

          if (prev_size > 0){
            car_s = end_path_s;
          }

          bool too_close = false;

          // Decide lane change or slow down in lane
          for (size_t i = 0; i < sensor_fusion.size(); ++i){
            float d = sensor_fusion[i][6];

            // Get vehicle in current lane
            if (d < (2+4*lane+2) && d > (2+4*lane-2)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];

              check_car_s += ((double)prev_size * 0.02 * check_speed);
              
              // If vehicle is the next vehicle ahead of ours
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)){
                too_close = true;
                vector<float> left_vehicle_s, right_vehicle_s;

                // Get other vehicles s and sort into left and right lanes
                for (size_t j = 0; j < sensor_fusion.size(); ++j){
                  float other_vehicle_d = sensor_fusion[j][6];
                  if (other_vehicle_d <= (2+4*lane-2) && other_vehicle_d >= (2+4*lane-6)){
                    left_vehicle_s.push_back(sensor_fusion[j][5]);
                  }
                  else if (other_vehicle_d >= (2+4*lane+2) && other_vehicle_d <= (2+4*lane+6)){
                    right_vehicle_s.push_back(sensor_fusion[j][5]);
                  }
                }

                int lane_change = -1; //default to left lane change

                // Check for space in the left lane
                if (lane > 0){
                  for (auto lvs : left_vehicle_s){
                    if (lvs < car_s + 30 && lvs > car_s - 30){
                      //if any vehicle is in the safe pocket, adjust to right lane change
                      lane_change = 1;
                    }
                  }
                }
                else{
                  // If in the leftmost lane, adjust to right lane change
                  lane_change = 1;
                }

                // Left lane change was not available, check right lane
                if (lane < 2 && lane_change == 1){
                  for (auto rvs : right_vehicle_s){
                    if (rvs < car_s + 30 && rvs > car_s - 30){
                      // if any vehicle is in the safe pocket, adjust to no lane change
                      lane_change = 0;
                    }
                  }
                }

                // adjust lane
                lane += lane_change;

                // adjust is lane is out of bounds
                if (lane < 0){
                  lane = 0;
                }
                else if (lane > 2){
                  lane = 2;
                }
              }
            }
          }

          // slow down if too close to the vehicle ahead
          if (too_close){
            ref_vel -= 0.224; //5 meters/sec^2
          }

          // speed up if no vehicle within 30m and still under the allowed speed
          else if (ref_vel < 49.5){
            ref_vel += 0.224;
          }

          // get coarse path
          if (prev_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else{
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          for (size_t d = 1; d < 3; ++d){
            vector<double> next_wp = getXY(car_s+(d*30), 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(next_wp[0]);
            ptsy.push_back(next_wp[1]);
          }

          // shift path to vehicle frame
          for (size_t i = 0; i < ptsx.size(); ++i){
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          // Generate spline
          tk::spline s;
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals, next_y_vals;

          for (size_t i = 0; i < previous_path_x.size(); ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);

          double x_add_on = 0;

          // Fill out missing points from spline to make 50 points in the smooth path
          for (size_t i = 1; i <= 50-previous_path_x.size(); ++i){
            double N = (target_dist/(0.02*ref_vel/2.24/*mph->meters/sec*/));
            double x_point = x_add_on+target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
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