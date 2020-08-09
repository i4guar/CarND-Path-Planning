#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

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



          // START
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_path_size = previous_path_x.size();
          
          double desired_velocity = mphToMps(car_speed);
          std::cout << "speed_value_car: " << car_speed << std::endl;

          std::cout << "d_value_car: " << car_d << std::endl;
          int current_lane = dToLane(car_d);
          int desired_lane = current_lane;
          
          
          if (prev_path_size > 0) {
            car_s = end_path_s;
          }
          
          vector<double> target_speed_for_lanes;
          // target speed for each lane
          target_speed_for_lanes.push_back(SPEED_LIMIT);
          target_speed_for_lanes.push_back(SPEED_LIMIT);
          target_speed_for_lanes.push_back(SPEED_LIMIT);
          
          // for each car
          for (int i = 0; i < sensor_fusion.size(); i++) {
            int lane = dToLane(sensor_fusion[i][6]);
            int id = sensor_fusion[i][0];
            double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double s = sensor_fusion[i][5];

            double v = sqrt(vx*vx + vy*vy);
            
            //std::cout << "lane: " << lane << std::endl;
            //std::cout << "current_lane: " << current_lane << std::endl;
            if (current_lane == lane) {
              //std::cout << "SAME_LANE" << std::endl;
              

              //std::cout << "My car:" << car_s << std::endl;
              //std::cout << "car:" << s << std::endl;

              if (car_s + SAFETY_DIST > s && s > car_s) {
                std::cout << "CAR INFRONT" << std::endl;
                if( target_speed_for_lanes[lane] > v) {
                  target_speed_for_lanes[lane] = v;
                }
              }    
            } else {
              // check if lane is blocked
              if ((car_s + SAFETY_DIST > s && s > car_s) || (car_s - SAFETY_DIST < s && s < car_s)) {
                target_speed_for_lanes[lane] = 0.0;
                continue;
              } 
              if (car_s + 2 * SAFETY_DIST > s && s > car_s) {
                if( target_speed_for_lanes[lane] > v) {
                  target_speed_for_lanes[lane] = v;
                }
                continue;
              } 
              
            }
          }

          

          vector<int> possible_lanes = possibleLanes(current_lane);
          // consider lane change
          for (int i = 0; i < possible_lanes.size(); i++) {
            if (target_speed_for_lanes[possible_lanes[i]] - target_speed_for_lanes[desired_lane] > 0) {
              desired_lane = possible_lanes[i];
            }
          }

          
          double target_speed = target_speed_for_lanes[desired_lane];
          
          std::cout << "target_speed: " << target_speed << std::endl;
          double speed_diff = target_speed - desired_velocity;
          
          if (abs(speed_diff) < ACCELERATION) {
            desired_velocity = target_speed;
          } else if (speed_diff > 0) {
            desired_velocity += ACCELERATION;
          } else {
            desired_velocity -= ACCELERATION;
          }
          std::cout << "desired_speed: " << desired_velocity << std::endl;
          
          
          vector<double> planned_x;
          vector<double> planned_y;
          
          
          double x_origin = car_x;
          double y_origin = car_y;
          double yaw_origin = car_yaw;
          
          if(prev_path_size < 2) {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            planned_x.push_back(prev_car_x);
            planned_y.push_back(prev_car_y);

            planned_x.push_back(car_x);
            planned_y.push_back(car_y);
          } else {
            double prev_x = previous_path_x[prev_path_size - 2];
            double prev_y = previous_path_y[prev_path_size - 2];
            planned_x.push_back(prev_x);
            planned_y.push_back(prev_y);

            double last_x = previous_path_x[prev_path_size - 1];
            double last_y = previous_path_y[prev_path_size - 1];
            
            planned_x.push_back(last_x);
            planned_y.push_back(last_y);
            x_origin = last_x;
            y_origin = last_y;
            yaw_origin = atan2(last_y - prev_y, last_x - prev_x);
          }
          
          //std::cout << "lane: " << desired_lane << std::endl;
          double d = laneToD(desired_lane);
          //std::cout << "d: " << d << std::endl;

          vector<double> next_waypoint0 = getXY(car_s + WAYPOINT_DIST, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint1 = getXY(car_s + 2 * WAYPOINT_DIST, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint2 = getXY(car_s + 3 * WAYPOINT_DIST, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
         
          planned_x.push_back(next_waypoint0[0]);
          planned_y.push_back(next_waypoint0[1]);
          
          planned_x.push_back(next_waypoint1[0]);
          planned_y.push_back(next_waypoint1[1]);
          
          planned_x.push_back(next_waypoint2[0]);
          planned_y.push_back(next_waypoint2[1]);
          
          
          
          // transform planned xy coordinates to local car coordinates
          for (int i = 0; i < planned_x.size(); i++) {
            //std::cout << "SIZE " << planned_x.size() << std::endl;
            double translation_x = planned_x[i] - x_origin;
            double translation_y = planned_y[i] - y_origin;
            //std::cout << "BEFORE x: " << planned_x[i] << std::endl;
            //std::cout << "BEFORE y: " << planned_y[i] << std::endl;
            planned_x[i] = translation_x * cos(0 - yaw_origin) - translation_y * sin(0 - yaw_origin);
            planned_y[i] = translation_x * sin(0 - yaw_origin) + translation_y * cos(0 - yaw_origin);
            //std::cout << "AFTER x: " << planned_x[i] << std::endl;
            //std::cout << "AFTER y: " << planned_y[i] << std::endl; 
          }
          
          
          
          
          tk::spline s;
          
          s.set_points(planned_x, planned_y);
          
                    
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          for (int i = 0; i < previous_path_x.size(); i++) {
            //std::cout << "FINAL_Prev x: " << previous_path_x[i] << std::endl;
            //std::cout << "FINAL_prev y: " << previous_path_y[i] << std::endl; 
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          
          double target_x = WAYPOINT_DIST;
          double target_y = s(target_x);

          //std::cout << "target y: " << target_y << std::endl;

          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          //std::cout << "target_dist: " << target_dist << std::endl;

          double N = target_dist / (0.02 * desired_velocity);
          double x_step = target_x / N;
          //std::cout << "N: " << N << std::endl;
   
          int size = next_x_vals.size();
          
          // Plan for 50 points in future
          for (int i = 1; next_x_vals.size() < POINTS_TO_PLAN; i++) {
            double x = i * x_step;
            double y = s(x);
            //std::cout << "B_FINAL x: " << x << std::endl;
            //std::cout << "B_FINAL y: " << y << std::endl;
            
            // transform back
            double x_map_coordinates = (x * cos(yaw_origin) - y * sin(yaw_origin)) + x_origin;
            double y_map_coordinates = (x * sin(yaw_origin) + y * cos(yaw_origin)) + y_origin;
            
            
            //std::cout << "FINAL x: " << x_map_coordinates << std::endl;
            //std::cout << "FINAL y: " << y_map_coordinates << std::endl; 
            
            next_x_vals.push_back(x_map_coordinates);
            next_y_vals.push_back(y_map_coordinates);
          }
          
          //std::cout << "ONE RUN" << std::endl; 
          // END
          
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
