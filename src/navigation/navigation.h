//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>
#include <limits>
#include "eigen3/Eigen/Dense"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

 private:
  float StraightCollision(const Eigen::Vector2f& cloud_point, float stopping_distance);
  float CurveCollision(const Eigen::Vector2f& cloud_point,float stopping_distance, float r, float r_1, float r_2, float curvature);
  
  void LatencyDecelerate(float path_length);
  float AvoidObstacle(const std::vector<Eigen::Vector2f>& cloud, double time);

  float StraightClearance(const Eigen::Vector2f& point, float path_length);
  float CurveClearance(const Eigen::Vector2f& point, float r, float r_1, float r_2, float curvature, float path_length);

  void StopDrivingStraight(float path_length);
  void StopDrivingCurvature(float path_length);

  bool CheckReverse();

  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  bool first_timestep;


  // LIDAR boolean to denote if we should stop or not
  bool lidar_stop;

  // our variables located below

  // self explanatory
  float total_distance_traveled_;

  // what deceleration we should use. can be less than or equal to MAX_DEC_
  float deceleration;

  // the position difference vector from this position and the prev position
  Eigen::Vector2f pos_diff;
  
  // constants given on the sheet for CP1
  const float MAX_SPEED_ = 1;
  const float MAX_ACC_   = 3;
  const float MAX_DEC_   = 3;

  const float MARGIN_OF_COMFORT_ = 0.1;

  // according to 
  const float CAR_LENGTH_     = 0.535;
  const float CAR_WIDTH_      = 0.281;
  const float DIST_BTWN_TIRE_ = 0.324;
  // from the base_link to front of the car
  const float DIST_TO_FRONT_  = ((CAR_LENGTH_ - DIST_BTWN_TIRE_) / 2) + DIST_BTWN_TIRE_;
  // from the base_link to back of the car
  const float DIST_TO_BACK_   = CAR_LENGTH_ - DIST_TO_FRONT_;
  // from the base_link to middle of the car
  const float DIST_TO_MID_    = CAR_LENGTH_/2;
  Eigen::Vector2f LIDAR_CORD;

  const float MAX_FLOAT = std::numeric_limits<float>::max();
  const float MIN_FLOAT = std::numeric_limits<float>::max() * -1;

  const float DIST_TO_GOAL = 2.0; // i can't find out how to get 2.0 from the actual code

  // Tune weights
  const uint32_t NUM_ARCS_ = 21;
  //const float ARC_INC      = 2 / (NUM_ARCS_ - 1);
  const float ARC_INC      = 0.1;
  //const float CLEARANCE_WEIGHT_ = 1.5;
  //const float CLEARANCE_WEIGHT_ = 5.0;  // working!!!
  const float CLEARANCE_WEIGHT_ = 4.0;
  //const float CLEARANCE_WEIGHT_ = 3.5;
  //const float CLEARANCE_WEIGHT_ = 0.7;
  const float DIST_WEIGHT_      = -0.5;
  const float MAX_CLEARANCE     = 1.3;
  float curvature_;

};

}  // namespace navigation

#endif  // NAVIGATION_H
