/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

string hasData(string s) {
  auto found_null = s.find("null");
    auto b1 = s.find_first_of("{");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  State ego_state;

  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;

  if( x_points.size() > 1 ){
  	ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
  	ego_state.velocity.x = v_points[v_points.size()-1];
  	if(velocity < 0.01)
  		ego_state.rotation.yaw = yaw;

  }

  Maneuver behavior = behavior_planner.get_active_maneuver();

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if(behavior == STOPPED){

  	int max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];
  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);

  	}
  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if(spirals.size() == 0){
  	cout << "Error: No spirals generated " << endl;
  	return;
  }

  for(int i = 0; i < spirals.size(); i++){

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);

  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if(best_spirals.size() > 0)
  	best_spiral_idx = best_spirals[best_spirals.size()-1];

  if (best_spiral_idx == -1)
  {
    cout << "Warning: No best idx generated, using spiral 0 " << endl;
    best_spiral_idx=0;
  }

  int index = 0;
  int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();
  while( x_points.size() < max_points && index < add_points ){
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }


}

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);
	}
	obst_flag = true;
}

int main ()
{
  cout << "starting server" << endl;
  uWS::Hub h;

  double new_delta_time;
  int i = 0;

  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  time_t prev_timer;
  time_t timer;
  time(&prev_timer);

  // initialize pid steer
  /**
  * TODO (Step 1): create pid (pid_steer) for steer command and initialize values
  **/


  // initialize pid throttle
  /**
  * TODO (Step 1): create pid (pid_throttle) for throttle command and initialize values
  **/

  PID pid_steer = PID();
  PID pid_throttle = PID();

  pid_steer.Init(0.08, 0.008, 0.5, +1.2, -1.2); // WORKS
  pid_steer.Init(0.08, 0.008, 0.4, +1.2, -1.2); // 
  pid_steer.Init(0.1, 0.005, 0.7, +1.2, -1.2); // WORKS
  pid_steer.Init(0.02, 0.006, 0.2, +1.2, -1.2); // final
  //
  pid_throttle.Init(0.1, 0.01, 0.05, +1, -1); // WORKS
  pid_throttle.Init(0.15, 0.01, 0.05, +1, -1); // WORKS
  pid_throttle.Init(0.15, 0.01, 0.1, +1, -1); // works more or less, issue with the steer i think
  pid_throttle.Init(0.15, 0.02, 0.08, +1, -1); // works
  pid_throttle.Init(0.12, 0.005, 0.02, +1, -1); // final

  #define RATIO_POS_Y 0.4 // works
  #define RATIO_POS_X 0.8 // works

  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
        auto s = hasData(data);

        if (s != "") {

          auto data = json::parse(s);

          // create file to save values
          fstream file_steer;
          file_steer.open("steer_pid_data.txt");
          fstream file_throttle;
          file_throttle.open("throttle_pid_data.txt");

          vector<double> x_points = data["traj_x"];
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"];
          double yaw = data["yaw"];
          double velocity = data["velocity"];
          double sim_time = data["time"];
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          string tl_state = data["tl_state"];

          double x_position = data["location_x"];
          double y_position = data["location_y"];
          double z_position = data["location_z"];

          if(!have_obst){
          	vector<double> x_obst = data["obst_x"];
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst);
          }

          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

          vector< vector<double> > spirals_x;
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals;

          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);

          // Save time and compute delta time
          time(&timer);
          new_delta_time = difftime(timer, prev_timer);
          prev_timer = timer;

          ////////////////////////////////////////
          // Transform Trajectory into vehicle coordinate system
          ////////////////////////////////////////

          std::cout << "-------" << std::endl;
	        //std::cout << "Goal " << waypoint_x << " " << waypoint_y << " " << waypoint_t*180/3.14159 << std::endl;
          vector<double> x_points_veh;
          vector<double> y_points_veh;
          //double rot = yaw;
          for(int jj = 0; jj < y_points.size(); jj++)
          {
            //std::cout << "x " << x_points[jj] <<  " y " << y_points[jj] << " v " << v_points[jj] << std::endl;

            x_points_veh.push_back(x_points[jj]-x_position);
            y_points_veh.push_back(y_points[jj]-y_position);
          }


          //std::cout << "...." << std::endl;
          std::cout << "x " << x_position << " y " << y_position << " yaw " << yaw*180/3.14159 << " v " << velocity << std::endl;

          std::cout << "x " << x_points_veh[0] << " y " << y_points_veh[0] << std::endl;
          std::cout << "x " << x_points_veh[x_points_veh.size()-1] << " y " << y_points_veh[x_points_veh.size()-1] << std::endl;

          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////
          
          // Update the delta time with the previous command
          pid_steer.UpdateDeltaTime(new_delta_time);

          // Compute steer error
          double error_steer;
          double error_yaw;
          double error_position_y;

          error_steer = 0;

          int i_best = x_points_veh.size();
          float min_dist_sq=1e20;
          for(int jj = 0; jj < x_points_veh.size(); jj++)
          {
            float dist_sq = x_points_veh[jj]*x_points_veh[jj]+ y_points_veh[jj]*y_points_veh[jj];
            if ((dist_sq < min_dist_sq) && (x_points_veh[jj] > 0.f))
            {
              min_dist_sq = dist_sq;
              i_best = jj;
            }
          }
          double yaw_target = 0.f;
          if (i_best < x_points_veh.size())
          {
            yaw_target = atan2(y_points_veh[i_best], x_points_veh[i_best]);
            error_yaw = yaw_target-yaw;
            error_position_y = y_points_veh[i_best]*cos(yaw)-x_points_veh[i_best]*sin(yaw);          error_yaw = yaw_target-yaw;

            std::cout << "using best "  << x_points_veh[i_best] << " " << y_points_veh[i_best] << " " << std::endl;
          }
          else
          {
            error_yaw = 0.f;
            error_position_y = 0.f; 
          }

          std::cout << "y_ego    " << yaw*360/3.14159 << " y_target " << yaw_target*360/3.14159  << std::endl;
	        //std::cout << "error y " << error_position_y << " yaw " << error_yaw << std::endl;

          error_steer = error_yaw+RATIO_POS_Y*error_position_y;

          ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          /**
          * (step 2): uncomment these lines
          **/
          // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Compute error of throttle
          double error_speed;
          double error_position_x;
          double error_throttle;
          /**
          * (step 2): compute the throttle error (error_throttle) from the position and the desired speed
          **/
          //std::cout << "v_ego    " << velocity << std::endl;
          //std::cout << "v_target " << v_points[v_points.size()-1]  << std::endl;
          if (i == 1) velocity = 0;
          if (i_best < v_points.size())
          {
            // behind traj
            error_speed = v_points[i_best+1]-velocity;
            error_position_x = x_points_veh[i_best]*cos(yaw)+x_points_veh[i_best]*sin(yaw);
          }
          else
          {
            // ahead of traj
            error_position_x = 0.f; // going too far
            error_speed=-velocity; // going too fast, simulating full stop
          }

          std::cout << "error x " << error_position_x << " v " << error_speed << std::endl;
          std::cout << "idx best " << i_best << std::endl;

                  
          error_throttle = error_speed+RATIO_POS_X*error_position_x;       


		      ///////////////////////
          // Compute control to apply
          pid_steer.UpdateError(error_steer);
          pid_throttle.UpdateError(error_throttle);



          // commands
          double steer_output = pid_steer.TotalError();
          double throttle = pid_throttle.TotalError();
          double throttle_output;
          double brake_output;

           // Adapt the negative throttle to break
           if (throttle > 0.0) {
             throttle_output = throttle;
             brake_output = 0;
           } else {
             throttle_output = 0;
             brake_output = -throttle;
           }


           // Save data
           file_steer.seekg(std::ios::beg);
           for(int j=0; j < i - 1; ++j) {
               file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
           }
           file_steer  << i ;
           file_steer  << " " << error_steer;
           file_steer  << " " << steer_output << endl;

           // Save data
           file_throttle.seekg(std::ios::beg);
           for(int j=0; j < i - 1; ++j){
               file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
           }
           file_throttle  << i ;
           file_throttle  << " " << error_throttle;
           file_throttle  << " " << brake_output;
           file_throttle  << " " << throttle_output;
           file_throttle  << " " << velocity;
           file_throttle  << " " << error_speed;
           file_throttle  << " " << error_position_x << endl;


          // Send control
          json msgJson;
          msgJson["brake"] = brake_output;
          msgJson["throttle"] = throttle_output;
          msgJson["steer"] = steer_output;

          msgJson["trajectory_x"] = x_points;
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points;
          msgJson["spirals_x"] = spirals_x;
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v;
          msgJson["spiral_idx"] = best_spirals;
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

          //  min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
          msgJson["update_point_thresh"] = 20;

          auto msg = msgJson.dump();

          i = i + 1;
          file_steer.close();
          file_throttle.close();

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    }

  });


  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
      cout << "Connected!!!" << endl;
    });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      ws.close();
      cout << "Disconnected" << endl;
    });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    }
  else
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }


}
