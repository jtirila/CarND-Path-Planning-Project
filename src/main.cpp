#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

bool DEBUG = true;

double RAW_DISTANCE_THRESHOLD_AHEAD = 26.0;
double RAW_DISTANCE_THRESHOLD_BEHIND= 10.0;
double DISTANCE_WITH_SPEED_THRESHOLD_AHEAD = 50.0;
double DISTANCE_WITH_SPEED_THRESHOLD_BEHIND= 15.0;
double SPEED_DIFF_THRESHOLD = 10.0;

using namespace std;

struct TrajectoryAndLane {vector<vector<double>> trajectory; int lane;};

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}


int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<vector<double>> generateTrajectory(
  double car_x, double car_y, double car_yaw, double car_s, double ref_vel,
  int lane,
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  vector<double>& map_waypoints_s, vector<double>& map_waypoints_x, vector<double>& map_waypoints_y) {

  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  // Later we will interpolate these waypoints with a spline and fill it in with more points that
  // control speed

  int prev_size = previous_path_x.size();
  vector<double> ptsx;
  vector<double> ptsy;

  // Reference x, y, yaw states
  // Either we will reference the starting points as where the car is or at the previous path's end point

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);


  // If previous size is almost empty, use the car as starting reference

  if(prev_size < 2) {
    cout << "car_yaw: " << car_yaw << "\n";
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);

    // Use the previous path's end point as starting reference
  } else {
    // cout << "car_yaw: " << car_yaw << "\n";
    // for(int i = 0; i < previous_path_x.size(); i++){
    //   cout << "previous_path_x[" << i << "]: " << previous_path_x[i] << "\n";
    // }

    // Redefine reference state as previous path end point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];

    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    // Use two points that make the path tangent to the previous path's end point

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);


  for(int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }


  // for (int i = 0; i < ptsx.size(); i++){
  //   cout << "ptsx[" << i << "]: " << ptsx[i] << "\n";
  // }
  // Create a spline
  tk::spline s;

  s.set_points(ptsx, ptsy);

  // Define the actual (x, y) points we will use for the planner

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Start with all the previous path points from last time

  for(int i = 0; i < previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0.0;

  // Fill up the rest of our path planner after filling it with previous points. Here we will always output 50 points.

  for(int i = 0; i < 50 - previous_path_x.size(); i++) {
    double N = target_dist / (0.02 * ref_vel / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);

  }
  return {next_x_vals, next_y_vals};

}


/* 
 * The cost function. The function is very simple due to the fact that infeasible 
 * paths are eliminated in advance. Hence, the cost function just considers distance
 * travelled and the amount of lane changes.  
 *
 * The cost function prefers paths where lanes are not changed excessively, and the car
 * is able to drive a maximum distance. 
 */
double cost(vector<vector<double>> trajectory, int next_lane, int lane){
  int size = trajectory[0].size();
  double x_dist = trajectory[0][size - 1] - trajectory[0][0];
  double y_dist = trajectory[1][size - 1] - trajectory[1][0];
  double abs_dist = x_dist * x_dist + y_dist * y_dist;
  double lane_change = (double) (next_lane != lane);

  return -abs_dist + 300 * (double) lane_change;
}


/* 
 * A helper function to choose the best one from the generated trajectories. Also, 
 * each trajectory is associated with
 * a related lane so that we can consider lane changes in the cost function. 
 */
TrajectoryAndLane ChooseBestTrajectory(vector<vector<vector<double>>>& trajectories, vector<int> next_lanes, int lane){
  // TODO: no need to keep index, can keep vector directly?
  int chosen_index = 0;
  int chosen_lane = next_lanes[0];
  double min_cost = cost(trajectories[0], next_lanes[0], lane);
  for(int i = 1; i < trajectories.size(); i++){
    double current_cost = cost(trajectories[i], next_lanes[i], lane);
    if(current_cost < min_cost){
      chosen_index = i;
      min_cost = current_cost;
      chosen_lane = next_lanes[i];
    }
  }
  TrajectoryAndLane trajectory_and_lane = { trajectories[chosen_index], chosen_lane};
  return trajectory_and_lane;
}

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  int lane = 1;
  double ref_vel = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
          double car_s_original = car_s;
         	double car_d = j[1]["d"];
         	double car_yaw = j[1]["yaw"];
         	double car_speed = j[1]["speed"];

         	// Previous path data given to the Planner
         	auto previous_path_x = j[1]["previous_path_x"];
         	auto previous_path_y = j[1]["previous_path_y"];
         	// Previous path's end s and d values
         	double end_path_s = j[1]["end_path_s"];
         	double end_path_d = j[1]["end_path_d"];

          int prev_size = previous_path_x.size();

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          cout << "Prev_size: " << prev_size << "\n";


          if(prev_size > 0) {
            car_s = end_path_s;
          }

          // Declare some flags to keep track of various possible states of nearby cars. Also for the cars
          // close straight ahead, keep track of the maximum speed difference to the ego vehicle
          bool straight_ahead_safe = true;
          bool left_lane_safe = true;
          bool right_lane_safe = true;
          double too_close_ahead_max_speed_diff = 0.0;


          // A loop where we determine whether is it safe to proceed
          // * straight ahead:  straight_ahead_safe
          // * to the left: left_lane_safe
          // * to the right: right_lane_safe
          //
          // Trajectories are subsequently generated for all the options that are safe

          for(int i = 0; i < sensor_fusion.size(); i++) {
            // car is in my lane
            float check_car_d = sensor_fusion[i][6];
            double check_car_vx = sensor_fusion[i][3];
            double check_car_vy = sensor_fusion[i][4];
            double check_speed = 2.24 * sqrt(check_car_vx * check_car_vx + check_car_vy * check_car_vy);
            double check_car_s = sensor_fusion[i][5];
            // Save the original check_car_s so that we can consider both the car's projected location and its current
            // location when determining whether other cars are too close
            double check_car_s_original = check_car_s;
            // project check_car_s value outwards in time to the end of the previous trajectory
            check_car_s += ((double)prev_size * 0.02 * check_speed / 2.24 );

            // Declare some flags to keep track of various possible states of nearby cars
            bool check_car_in_intended_lane = abs(check_car_d - 2 - 4 * lane) < 2.5;
            bool too_close_ahead_projected = false;
            bool too_close_behind_projected = false;
            bool too_close_ahead_now = false;
            bool too_close_behind_now = false;
            double speed_diff;
            bool too_close;

            if(DEBUG) {
              cout << "Observed car! "
                   << "; check_car_d: " << check_car_d
                   << "; car_s: " << car_s
                   << "; check_car_s: " << check_car_s
                   << "; car_s_original: " << car_s_original
                   << "; check_car_s_original: " << check_car_s_original
                   << "; car_speed: " << car_speed
                   << "; check_speed: " << check_speed << "\n";
            }

            // check car ahead of us and too close in terms of projected locations?
            if(check_car_s > car_s) {
              if(check_car_s - car_s < RAW_DISTANCE_THRESHOLD_AHEAD) {
                too_close_ahead_projected = true;
                speed_diff = car_speed - check_speed;
                if(speed_diff > too_close_ahead_max_speed_diff) {
                  too_close_ahead_max_speed_diff = speed_diff;
                }
              } else if (check_car_s - car_s < DISTANCE_WITH_SPEED_THRESHOLD_AHEAD && car_speed - check_speed > SPEED_DIFF_THRESHOLD){
                too_close_ahead_projected = true;
                speed_diff = car_speed - check_speed;
                if(speed_diff > too_close_ahead_max_speed_diff) {
                  too_close_ahead_max_speed_diff = speed_diff;
                }
              }

            // check car behind us and too close in terms of projected locations?
            } else {
              if(car_s - check_car_s < RAW_DISTANCE_THRESHOLD_BEHIND) {
                too_close_behind_projected = true;
              } else if (car_s - check_car_s < DISTANCE_WITH_SPEED_THRESHOLD_BEHIND && check_speed - car_speed > SPEED_DIFF_THRESHOLD) {
                too_close_ahead_projected = true;
              }
            }

            // check car ahead of us and too close in terms of current location?
            if(check_car_s_original > car_s_original) {
              if(check_car_s_original - car_s_original < RAW_DISTANCE_THRESHOLD_AHEAD) {
                too_close_ahead_now = true;
                speed_diff = car_speed - check_speed;
                if(speed_diff > too_close_ahead_max_speed_diff) {
                  too_close_ahead_max_speed_diff = speed_diff;
                }
              } else if (check_car_s_original - car_s_original < DISTANCE_WITH_SPEED_THRESHOLD_AHEAD && car_speed - check_speed > SPEED_DIFF_THRESHOLD){
                too_close_ahead_now = true;
                speed_diff = car_speed - check_speed;
                if(speed_diff > too_close_ahead_max_speed_diff) {
                  too_close_ahead_max_speed_diff = speed_diff;
                }
              }
            // check car behind us and too close in terms of current locations?
            } else {
              if(car_s_original - check_car_s_original < RAW_DISTANCE_THRESHOLD_BEHIND) {
                too_close_behind_now = true;
              } else if (car_s_original - check_car_s_original < DISTANCE_WITH_SPEED_THRESHOLD_BEHIND && check_speed - car_speed > SPEED_DIFF_THRESHOLD) {
                too_close_behind_now = true;
              }
            }


            // In the subsequent processing, the details of whether the check car is too close ahead or behind,
            // at the projected or current time, do not matter. Any of these will be just considered "too close".
            too_close = too_close_ahead_now || too_close_ahead_projected || too_close_behind_now || too_close_behind_projected;

            if(DEBUG) {
              cout << "Too close: " << too_close << "\n";
            }

            if(too_close && check_car_in_intended_lane) {

              if (DEBUG) {
                cout << "Car ahead! Lane: " << lane
                     << "; check_car_d: " << check_car_d
                     << "; car_s: " << car_s
                     << "; check_car_s: " << check_car_s
                     << "; car_s_original: " << car_s_original
                     << "; check_car_s_original: " << check_car_s_original
                     << "; car_speed: " << car_speed
                     << "; check_speed: " << check_speed << "\n";
                straight_ahead_safe = false;
              }
            }

            // Check if a car is going to be dangerously close on the left or right
            bool check_car_on_left = too_close && (
              (2 + (lane - 1) * 4 - check_car_d < 2.0) &&
              (2 + (lane - 1) * 4 - check_car_d > -2.0)
            );
            bool check_car_on_right = too_close && (
              (2 + (lane + 1) * 4 - check_car_d < 2.0) &&
              (2 + (lane + 1) * 4 - check_car_d > -2.0)
            );

            // Print some diagnostic info if in DEBUG mode
            if (DEBUG) {
              if(check_car_on_left) {
                cout << "Car on left! Lane: " << lane
                     << "; check_car_d: " << check_car_d
                     << "; car_s: " << car_s
                     << "; check_car_s: " << check_car_s
                     << "; car_s_original: " << car_s_original
                     << "; check_car_s_original: " << check_car_s_original
                     << "; car_speed: " << car_speed
                     << "; check_speed: " << check_speed << "\n";
              }

              if(check_car_on_right) {
                cout << "Car on right! Lane: " << lane
                     << "; check_car_d: " << check_car_d
                     << "; car_s: " << car_s
                     << "; check_car_s: " << check_car_s
                     << "; car_s_original: " << car_s_original
                     << "; check_car_s_original: " << check_car_s_original
                     << "; car_speed: " << car_speed
                     << "; check_speed: " << check_speed << "\n";
              }
            }

            // Also eliminate changing too far to the left or right
            if(lane == 0 || (check_car_on_left)) {
              left_lane_safe = false;
            }
            if(lane == 2 || (check_car_on_right)){
              right_lane_safe = false;
            }
          }

          vector<double> previous_path_x_doubles;
          vector<double> previous_path_y_doubles;

          // Convert the previous path's x and y values to a vector of doubles for easier access and manipulation
          for(int i = 0; i < previous_path_x.size(); i++){
            previous_path_x_doubles.push_back(previous_path_x[i]);
            previous_path_y_doubles.push_back(previous_path_y[i]);
          }

          if(DEBUG) {
            cout << "About to generate trajectories\n";
            cout << "Ref vel: " << ref_vel << "\n";
          }
          vector<vector<vector<double>>> next_trajectories;
          vector<int> next_lanes;

          // Increase target speed if we are going slowly and there is at least one safe driving option
          if((straight_ahead_safe || left_lane_safe || right_lane_safe) && ref_vel < 49.0){
            ref_vel += 0.5;
          }

          if(straight_ahead_safe){
            if(DEBUG) {
              cout << "OK to go straight ahead on lane " << lane << "; generating corresponding trajectory\n";
            }
            next_trajectories.push_back(generateTrajectory(car_x, car_y, car_yaw, car_s, ref_vel, lane, previous_path_x_doubles,
                                                previous_path_y_doubles, map_waypoints_s, map_waypoints_x, map_waypoints_y));
            next_lanes.push_back(lane);
          }
          // If we can go to the left safely
          if(left_lane_safe) {
            int this_lane = lane - 1;
            if(DEBUG) {
              cout << "OK to go to the left (lane " << this_lane << "); generating corresponding trajectory\n";
            }
            next_trajectories.push_back(
                generateTrajectory(car_x, car_y, car_yaw, car_s, ref_vel, this_lane, previous_path_x_doubles,
                                   previous_path_y_doubles, map_waypoints_s, map_waypoints_x, map_waypoints_y));
            next_lanes.push_back(this_lane);
          }

          // If we can go to the right safely
          if(right_lane_safe){
            int this_lane = lane + 1;
            if(DEBUG) {
              cout << "OK to go to the right (lane " << this_lane << "); generating corresponding trajectory\n";
            }
            next_trajectories.push_back(
                generateTrajectory(car_x, car_y, car_yaw, car_s, ref_vel, this_lane, previous_path_x_doubles,
                                   previous_path_y_doubles, map_waypoints_s, map_waypoints_x, map_waypoints_y));
            next_lanes.push_back(this_lane);
          }
          if(!(straight_ahead_safe || left_lane_safe || right_lane_safe)){
            if(DEBUG) {
              cout << "No safe options other than slow down on lane " << lane << "\n";
            }

            // In case we need to stay on lane and pay attention to the vehicle ahead, slow down,
            // but only if we are going faster than them
            if(too_close_ahead_max_speed_diff > 0.0){
              ref_vel -= 0.5;
            }
            next_trajectories.push_back(
                generateTrajectory(car_x, car_y, car_yaw, car_s, ref_vel, lane, previous_path_x_doubles,
                                   previous_path_y_doubles, map_waypoints_s, map_waypoints_x, map_waypoints_y));
            next_lanes.push_back(lane);

          }

          // pick the trajectory with the lowest cost
          TrajectoryAndLane trajectory_and_lane = ChooseBestTrajectory(next_trajectories, next_lanes, lane);
          // Assign the new lane
          lane = trajectory_and_lane.lane;
          vector<vector<double>> best_trajectory = trajectory_and_lane.trajectory;
          if(DEBUG) {
            cout << "Chosen lane: " << lane << "\n";
          }

          auto next_x_vals = best_trajectory[0];
          auto next_y_vals = best_trajectory[1];

         	json msgJson;

         	msgJson["next_x"] = next_x_vals;
         	msgJson["next_y"] = next_y_vals;

         	auto msg = "42[\"control\","+ msgJson.dump()+"]";

         	//this_thread::sleep_for(chrono::milliseconds(1000));
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
