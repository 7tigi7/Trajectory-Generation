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
int lane = 1;
double ref_vel = 0.0;//mph
int closeCounter=0;

bool distSmall = true;
bool speedTooFar = true;

constexpr double gapFar = 22.0;
constexpr double gapClose = 13.0;
constexpr double gapVeryClose = 7.0;
int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  double previous_distance = 0.0;
  double current_distance = 0.0;
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
               &map_waypoints_dx,&map_waypoints_dy,&current_distance,&previous_distance]
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
          int prev_size = previous_path_x.size();
          
          if(prev_size>0)
          {
              car_s = end_path_s;
          }
          bool too_close = false;
          
          
          double vel_change=0.33;
          double target_car_speed = 49.0;
          //find ref_v to use
          bool laneOneFree = true;
          bool laneTwoFree = true;
          bool laneThreeFree = true;
          for(int i=0; i<sensor_fusion.size();++i)
          {
            //sensor_fusion[i] i th car on road detected by sensor fusion
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(pow(vx,2)+pow(vy,2));
            double check_car_s = sensor_fusion[i][5];
            double targetCarSpeed = check_speed*2.24;
            current_distance = check_car_s-car_s;
            //checking that the lane has enough space to change into, if its less than 27 we consider it is not safe
            distSmall = abs(current_distance) < 27;
            //checking that the car is not moving too slow or too fast, compared to us if it is more than 8.5 we consider it is not safe.
            speedTooFar = 8.5 < abs(targetCarSpeed - car_speed);
            
            //Checking that is there a car in the danger zone (for each lane)
            if((d < (4.0) && d > (0.0)) && (distSmall || speedTooFar))
            {
                laneOneFree=false;
            }
            if((d < (8.0) && d > (4.0)) && (distSmall || speedTooFar))
            {
                laneTwoFree = false;
            }
            if((d < (12.0) && d > (8.0)) && (distSmall || speedTooFar))
            {
                laneThreeFree = false;
            }
            
            //car in our lane
            if(d < (2+4*lane+2) && d > (2+4*lane-2))
            {
                check_car_s+=(double)prev_size*0.02*check_speed;
                //Checking the gap between us and the other car in the lane, we are braking harder and harder as we get closer
                bool otherCarSBigger = (check_car_s > car_s);
                if(otherCarSBigger && (current_distance < gapVeryClose))
                {
                     vel_change = 0.77;
                     too_close = true;
                     target_car_speed = check_speed;
                }
              	else if(otherCarSBigger && (current_distance < gapClose))
                {
                     vel_change = 0.57;
                     too_close = true;
                     target_car_speed = check_speed;
                }
              	else if(otherCarSBigger && (current_distance < gapFar))
                {
                     vel_change = 0.3251992;
                     too_close = true;
                     target_car_speed = check_speed;
                }
            }
          }

          if(too_close)
          {
              if(closeCounter==10)
              {
                  if(laneOneFree)
                  {
                      if(lane==1)
                      {
                         lane=0;
                         closeCounter=0;
                      }
                      
                  }
                  if(laneThreeFree)
                  {
                      if(lane==1)
                      {
                         lane=2;
                         closeCounter=0;
                      }
                  }
                  if(laneTwoFree)
                  {
                      if(lane!=1)
                      {
                          lane=1;
                          closeCounter=0;
                      }
                      
                  }
              }
              else
              {
                  ++closeCounter;
              }
            
              if(closeCounter!=0 && target_car_speed * 2.24 < car_speed)
              {
                  ref_vel -=(ref_vel-vel_change > 0 ? vel_change : ref_vel);
              }
              else
              {
                 ref_vel +=(ref_vel+vel_change < 40.5 ? vel_change : 0.0);
              }
              
          }
          else
          {
              if(target_car_speed > car_speed)
              {
                  ref_vel +=(ref_vel+vel_change < 46.5 ? vel_change : 0.0);
              }
          }
  
          json msgJson;


          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(prev_size < 2)
          {
           	double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          //change 30,60,90, to make lane change more "agressive"
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i=0;i<ptsx.size();++i)
          {
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;
              ptsx[i] = (shift_x * cos(0-ref_yaw))-(shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw))+(shift_y*cos(0-ref_yaw));
          }
          
          tk::spline s;
          s.set_points(ptsx,ptsy);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          
         for(int i = 0;i < previous_path_x.size();++i)
         {
             next_x_vals.push_back(previous_path_x[i]);
             next_y_vals.push_back(previous_path_y[i]);
         }
         double target_x = 30.0;
         double target_y = s(target_x);
         double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));
          
         double x_add_on =0;
         double dist_inc = 0.3;
          
         size_t prev_path_size = previous_path_x.size(); 
          
         for (int i = 1; i <= 50-prev_path_size; ++i)
         {
             double N = target_dist/(0.02*ref_vel/2.24);
             double x_point = x_add_on+(target_x)/N;
             double y_point = s(x_point);
           
             x_add_on = x_point;
             double x_ref = x_point;
             double y_ref = y_point;
             //rotate back global 
             x_point = (x_ref * cos(ref_yaw)-y_ref* sin(ref_yaw));
             y_point = (x_ref * sin(ref_yaw)+y_ref* cos(ref_yaw));
             
             x_point += ref_x;
             y_point += ref_y;
             next_x_vals.push_back(x_point);
             next_y_vals.push_back(y_point);
             
         }
          
         /*
         for (int i = 0; i < 50; ++i) 
         {
             //keep in the lanes
             double next_s = car_s+(i+1)*dist_inc;
             double next_d = 6;
             vector<double> xy = getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y); 
             next_x_vals.push_back(xy[0]);
             next_y_vals.push_back(xy[1]);
          }
          */
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