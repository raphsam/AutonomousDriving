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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <limits>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using std::sin;
using std::cos;
using std::sqrt;
using std::min;
using std::max;

using namespace math_util;
using namespace ros_helpers;

DEFINE_double(cp1_distance, 2.5, "Distance to travel for 1D TOC (cp1)");
DEFINE_double(cp3_curvature, 0.5, "Curvature for arc path (cp3)");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(false),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    first_timestep(false),
    lidar_stop(false),
    total_distance_traveled_(0),
    deceleration(0),
    pos_diff(0, 0),
    LIDAR_CORD(0.2, 0),
    curvature_(0)
    {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    // Update the current navigation target
    nav_goal_loc_   = loc;
    nav_goal_angle_ = angle;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
    // Update the current estimate of the robot's position in the map reference frame.
    if (!first_timestep) {
        // first timestep
        first_timestep = true;
        robot_loc_   = loc;
        robot_angle_ = angle;
    }
    else {
        robot_omega_ = (angle - robot_angle_);
        pos_diff     = loc - robot_loc_;
        robot_loc_   = loc;
        robot_angle_ = angle;

        // magnitude of the 2D vector
        total_distance_traveled_ += std::sqrt(Pow(pos_diff.x(), 2) + Pow(pos_diff.y(), 2));
        //printf("total: %f\n", total_distance_traveled_);
    }

}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
    // Update the robot's position in the odometry reference frame.
    // Update the current estimate of the robot's velocity
    Eigen::Matrix2f rotation_matrix;
    rotation_matrix <<      std::cos(angle), std::sin(angle),
                       -1 * std::sin(angle), std::cos(angle);

    odom_loc_ = rotation_matrix * pos_diff;

    //robot_loc_   = loc;
    //robot_angle_ = angle;
    robot_vel_   = vel;
    robot_omega_ = ang_vel;

    //printf("x:%f y:%f\n", pos_diff.x(), pos_diff.y());
}



float Navigation::StraightCollision(const Vector2f& cloud_point, float stopping_distance) {
    // returns the path length for a collision
    Vector2f in_base_link = cloud_point + LIDAR_CORD;
    float min_length = MAX_FLOAT;
    //printf("point: (%f,%f)\n", in_base_link.x(), in_base_link.y());
    if (abs(in_base_link.y()) <= MARGIN_OF_COMFORT_ + (CAR_WIDTH_ / 2)) {
        //printf("what\n");
        // effective path length
        min_length = min(min_length, abs(in_base_link.x() - MARGIN_OF_COMFORT_ - DIST_TO_FRONT_));
        //if (stopping_distance + MARGIN_OF_COMFORT_ + DIST_TO_FRONT_ >= abs(in_base_link.x())) {
        //    printf("uh oh obstacle approaching\n");
        //    //return abs(in_base_link.x());
        //    //return abs(in_base_link.x()) - MARGIN_OF_COMFORT_ - DIST_TO_FRONT_;
        //}
    }
    //printf("min: %f\n", min_length);
    return min_length;
    //return MAX_FLOAT;       // i'm not going to hit anything
}

float Navigation::CurveCollision(const Vector2f& cloud_point,float stopping_distance, float r,
                                 float r_1, float r_2, float curvature) {
    Vector2f curvature_vector(0, r);
    Vector2f in_base_link = cloud_point + LIDAR_CORD;

    if (curvature < 0.0) {
        in_base_link[1] = -1 * in_base_link[1];
    }

    Vector2f difference = in_base_link - curvature_vector;

    const float theta = atan2(in_base_link.x(), r - in_base_link.y());

    //if (difference.norm() >= r_1 && difference.norm() <= r_2 && theta > 0 && theta < M_PI) {
    if (difference.norm() >= r_1 && difference.norm() <= r_2 && theta > 0){
        // path length computation
        const float omega = atan2(DIST_TO_FRONT_ + MARGIN_OF_COMFORT_, r - (CAR_WIDTH_ / 2) - MARGIN_OF_COMFORT_);
        const float phi = theta - omega;
        const float path_length = r * phi;

        return abs(path_length);
    }
    return MAX_FLOAT;
}



float Navigation::StraightClearance(const Vector2f& point, float path_length) {
    if (point.x() > -1 * DIST_TO_BACK_ && point.x() < path_length + DIST_TO_FRONT_ && abs(point.y()) < MAX_CLEARANCE) {
        return abs(point.y()) - (CAR_WIDTH_ / 2) - MARGIN_OF_COMFORT_;
    }
    return MAX_CLEARANCE;
}

float Navigation::CurveClearance(const Vector2f& point, float r, float r_1, float r_2,
                                 float curvature, float path_length) {
    //Vector2f curvature_vector(0, r);
    Vector2f curvature_vector(0, r);
    Vector2f in_base_link = point + LIDAR_CORD;

    // account for negative curvature
    if (curvature < 0.0) {
        in_base_link[1] *= -1;
    }

    float norm = (curvature_vector - in_base_link).norm();

    if (norm > r) {
        return norm - r_2;
    }
    else {
        return r_1 - norm;
    }
}



void Navigation::LatencyDecelerate(float path_length) {

    float new_stopping_time = abs(abs(path_length) - MARGIN_OF_COMFORT_ -
                                      DIST_TO_FRONT_) * 2 / robot_vel_.x();

    // calculate the new deceleration (should be slightly slower)
    float new_dec = abs(robot_vel_.x() / new_stopping_time);

    if (new_dec > deceleration) {
        deceleration = new_dec;
        //deceleration = MAX_DEC_;
    }
    if (deceleration > MAX_DEC_) {
        deceleration = MAX_DEC_;
    }
    //printf("deceleration: %f\n", deceleration);
    lidar_stop = true;
}

float Navigation::AvoidObstacle(const vector<Vector2f>& cloud, double time) {
    // stopping time and calculate kinematic distance integral
    float stopping_time = robot_vel_.x() / MAX_DEC_;
    float stopping_distance = (0.5) * robot_vel_.x() * (stopping_time + 0.149);

    float max_score = MIN_FLOAT;
    float best_arc  = -2;
    float best_path_length = 0.0;

    // need to determine what path is safe
    for (uint32_t i = 0; i < NUM_ARCS_; i++) {
        const float CURVATURE = 1.0 - (i * ARC_INC);

        float arc_score = MAX_FLOAT;
        float path_length = MAX_FLOAT;
        float clearance = MAX_FLOAT;

        // Compute the closest point of approach
        // (if ARC=0, then it's just the x component. if it's arc, you
        // have to do a bit more computation)
        float closest_point_of_approach = DIST_TO_GOAL;
        float distance_to_goal = 0.0; // also x coordinate of goal??? (i tried y coordinate but idk if that's right)
        if (CURVATURE != 0.0) {
            // a bunch of extra computations if you're taking a curve
            float r   = 1.0 / abs(CURVATURE);

            // calculate the arc length of taking this trajectory
            float theta = atan2(DIST_TO_GOAL, abs(r));
            closest_point_of_approach = abs(r) * theta;

            // calculate the distance from the point of approach to the object
            float hypotenuse = sqrt(Pow(DIST_TO_GOAL, 2) + Pow(r, 2));
            distance_to_goal = hypotenuse - abs(r);
            //printf("dist: %f\n", distance_to_goal);
        }

        // compute the min path length in the point cloud
        for (uint32_t j = 0; j < cloud.size(); j++) {
            float new_path = 0.0;
            float new_clearance = 0.0;
            // first compute if the point will get hit in this particular arc
            if (CURVATURE == 0.0) {
                new_path = Navigation::StraightCollision(cloud[j], stopping_distance);
                new_clearance = Navigation::StraightClearance(cloud[j], path_length);
            }
            else {
                // wanna bring this out of the for loop but i don't think it's possible
                float r   = 1.0 / abs(CURVATURE);
                float r_1 = r - (CAR_WIDTH_ / 2) - MARGIN_OF_COMFORT_;
                float r_2 = Pow(r + (CAR_WIDTH_ / 2) + MARGIN_OF_COMFORT_, 2) +
                            Pow(DIST_TO_FRONT_ + MARGIN_OF_COMFORT_, 2);
                r_2 = std::sqrt(r_2);

                new_path = Navigation::CurveCollision(cloud[j], stopping_distance, r, r_1, r_2, CURVATURE);
                new_clearance = Navigation::CurveClearance(cloud[j], r, r_1, r_2, CURVATURE, new_path);
            }

            // cap the path length
            new_path = std::min(closest_point_of_approach, new_path);
            path_length = std::min(new_path, path_length);

            // cap the clearance terms
            new_clearance = std::min(new_clearance, MAX_CLEARANCE);
            clearance = std::min(clearance, new_clearance);
        }

        //printf("ARC: %f   C: %f\n", CURVATURE, clearance);
        // compute the score
        float score = path_length + (clearance * CLEARANCE_WEIGHT_) + (DIST_WEIGHT_ * distance_to_goal);
        arc_score = min(arc_score, score);

        if (max_score < arc_score) {
            //printf("score %f\n", arc_score);
            max_score = arc_score;
            best_arc = CURVATURE;
            best_path_length = path_length;
        }
        
        visualization::DrawPathOption(CURVATURE, path_length, clearance, local_viz_msg_);
        //printf("path_length: %f\n", path_length);
    }

    curvature_ = best_arc;

    viz_pub_.publish(local_viz_msg_);
    //if (curvature_ == 0.0) {
    //    return best_path_length;// + DIST_TO_FRONT_ + MARGIN_OF_COMFORT_;
    //}
    return best_path_length;
}



void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
    // This function will be called when the LiDAR sensor on the robot has a new scan.
    // Here cloud is an array of points observed by the laser sensor, in the sensor's reference frame
    // This information can be used to detect obstacles in the robot's path.
    //lidar_stop = false;
    //if (lidar_stop == false) {        
    float path_length = Navigation::AvoidObstacle(cloud, time);
    float stopping_time = robot_vel_.x() / MAX_DEC_;
    float stopping_distance = (0.5) * robot_vel_.x() * (stopping_time + 0.149);
    //printf("best_path_length: %f   stopping_distance %f\n", path_length, stopping_distance);

    if (stopping_distance >= abs(path_length)) {
        if (curvature_ == 0.0) {
            Navigation::LatencyDecelerate(path_length + MARGIN_OF_COMFORT_ + DIST_TO_FRONT_);
        }
        else {
            Navigation::LatencyDecelerate(path_length);
        }
    }
}

void Navigation::Run() {
    // Called every timestep. This will be the main entrypoint of the navigation code, and is responsible for publishing appropriate navigation commands.
    //printf("robot velocity %lf\n",robot_vel_.x());
    visualization::ClearVisualizationMsg(local_viz_msg_);
    visualization::ClearVisualizationMsg(global_viz_msg_);

    // top horiontal
    Vector2f p1(-1 * (MARGIN_OF_COMFORT_ + DIST_TO_BACK_),
                CAR_WIDTH_ / 2 + MARGIN_OF_COMFORT_);
    Vector2f p2(MARGIN_OF_COMFORT_ + DIST_TO_FRONT_,
                CAR_WIDTH_ / 2 + MARGIN_OF_COMFORT_);

    // bottom horizontal
    Vector2f p3(-1 * (MARGIN_OF_COMFORT_ + DIST_TO_BACK_),
                -1 * (CAR_WIDTH_ / 2 + MARGIN_OF_COMFORT_));
    Vector2f p4(MARGIN_OF_COMFORT_ + DIST_TO_FRONT_,
                -1 * (CAR_WIDTH_ / 2 + MARGIN_OF_COMFORT_));
    
    visualization::DrawLine(p1, p2, 0xcc5500, local_viz_msg_);
    // bottom
    visualization::DrawLine(p3, p4, 0xcc5500, local_viz_msg_);
    //left
    visualization::DrawLine(p1, p3, 0xcc5500, local_viz_msg_);
    //right
    visualization::DrawLine(p2, p4, 0xcc5500, local_viz_msg_);

    // just car
    Vector2f p5(-1*DIST_TO_BACK_ , CAR_WIDTH_ / 2);
    Vector2f p6(DIST_TO_FRONT_, CAR_WIDTH_ / 2);
    Vector2f p7(-1*DIST_TO_BACK_ , -1 * (CAR_WIDTH_ / 2));
    Vector2f p8(DIST_TO_FRONT_, -1 * (CAR_WIDTH_ / 2));

    visualization::DrawLine(p5, p6, 0x00ff00, local_viz_msg_);
    visualization::DrawLine(p7, p8, 0x00ff00, local_viz_msg_);
    visualization::DrawLine(p5, p7, 0x00ff00, local_viz_msg_);
    visualization::DrawLine(p6, p8, 0x00ff00, local_viz_msg_);

    // object detected
    // since each timestep is 1/20 seconds, we can determine what the new
    // velocity should be and issue that command
    float new_velocity = robot_vel_.x();
    if (lidar_stop == false) {
        new_velocity += (MAX_ACC_ * 0.05);
        if (new_velocity > MAX_SPEED_) {
            new_velocity = MAX_SPEED_;
        }
    }
    // 2) we need to decelerate (aka our current velocity won't get us to the target distance)
    // lidar_stop == true also
    else{
        // since each timestep is 1/20 seconds, we can determine what the new
        // velocity should be and issue that command
        new_velocity -= (deceleration * 0.05);
        if (new_velocity < 0) {
            new_velocity = 0;
        }
    }
    drive_msg_.velocity  = new_velocity;
    drive_msg_.curvature = curvature_;

    // send out the message
    drive_pub_.publish(drive_msg_);
    viz_pub_.publish(local_viz_msg_);
    viz_pub_.publish(global_viz_msg_);

}

}  // namespace navigation
