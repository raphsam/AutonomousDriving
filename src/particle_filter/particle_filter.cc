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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::Line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Rotation2Df;
using vector_map::VectorMap;
using math_util::Pow;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    map_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    /*
    const Line2f map_line = map_.lines[i];
    // The Line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    Line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    printf("P0: %f, %f P1: %f,%f\n", 
           my_line.p0.x(),
           my_line.p0.y(),
           my_line.p1.x(),
           my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      printf("Intersects at %f,%f\n", 
             intersection_point.x(),
             intersection_point.y());
    } else {
      printf("No intersection\n");
    }
    */
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
}

void ParticleFilter::Resample() {
    // Resample the particles, proportional to their weights.
    // The current particles are in the `particles_` variable. 
    // Create a variable to store the new particles, and when done, replace the
    // old set of particles:
    // vector<Particle> new_particles';
    // During resampling: 
    //    new_particles.push_back(...)
    // After resampling:
    // particles_ = new_particles;

    // You will need to use the uniform random number generator provided. For
    // example, to generate a random number between 0 and 1:
    float x = rng_.UniformRandom(0, 1);
    printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
           x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
    // A new laser scan observation is available (in the laser frame)
    // Call the Update and Resample steps as necessary.
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
    // A new odometry value is available (in the odom frame)
    // Implement the motion model predict step here, to propagate the particles
    // forward based on odometry.
    
    // TODO: Need to forward predict the particles using motion models
    // need to be updating the model every time this function is called

    if (odom_initialized_ && map_initialized_) {

        // calculate the deviations for this particular odometry
        const float DEVIATION_1 = (K1_ * (odom_loc - prev_odom_loc_).norm()) + (K2_ * (odom_angle - prev_odom_angle_));
        const float DEVIATION_2 = (K3_ * (odom_loc - prev_odom_loc_).norm()) + (K4_ * (odom_angle - prev_odom_angle_));
      
        // iterate through each particle and update it's position?
        for (uint32_t i = 0; i < NUM_SAMPLES_; i++) {
            // sample each error based off of the two distributions
            float error_x     = rng_.Gaussian(0.0, DEVIATION_1);
            float error_y     = rng_.Gaussian(0.0, DEVIATION_1);
            float error_theta = rng_.Gaussian(0.0, DEVIATION_2);

            Rotation2Df pos_given(particles_[i].angle);
            Rotation2Df neg_given(-1 * odom_angle);
            Vector2f error_vector(error_x, error_y);

            // get the updated transition vector and angles with noise
            Vector2f delta_t_bl = neg_given * (odom_loc - prev_odom_loc_);
            Vector2f new_translation = (pos_given * delta_t_bl) + error_vector;

            float delta_angle_bl = odom_angle - prev_odom_angle_;
            float new_rotation = delta_angle_bl + error_theta;

            // update previous particle information
            particles_[i].loc    += new_translation;
            particles_[i].angle  += new_rotation;
        }

    }

    // update the previous state
    prev_odom_loc_    = odom_loc;
    prev_odom_angle_  = odom_angle;
    odom_initialized_ = true;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
    // The "set_pose" button on the GUI was clicked, or an initialization message
    // was received from the log. Initialize the particles accordingly, e.g. with
    // some distribution around the provided location and angle.
    map_.Load(map_file);

    //const float DEVIATION_1 = (K1_ * loc.norm()) + (K2_ * angle);
    //const float DEVIATION_2 = (K3_ * loc.norm()) + (K4_ * angle);
    const float DEVIATION_1 = 0.2;
    const float DEVIATION_2 = 0.2;

    // TODO: create a set of particles
    for (uint32_t i = 0; i < NUM_SAMPLES_; i++) {
        Particle* newStruct = (Particle*)malloc(sizeof(Particle));
        //Particle newStruct = new 
        float error_x     = rng_.Gaussian(0.0, DEVIATION_1);
        float error_y     = rng_.Gaussian(0.0, DEVIATION_1);
        float error_theta = rng_.Gaussian(0.0, DEVIATION_2);
        Vector2f error(error_x, error_y);

        newStruct->loc = loc + error;
        newStruct->angle = angle + error_theta;
        particles_.push_back(*newStruct);   // dunno if this is correct
    }

    // Set the map variables
    map_initialized_ = true;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
    Vector2f& loc = *loc_ptr;
    float& angle = *angle_ptr;
    // Compute the best estimate of the robot's location based on the current set
    // of particles. The computed values must be set to the `loc` and `angle`
    // variables to return them. Modify the following assignments:
    //loc = t_map_;
    loc = Vector2f(0, 0);
    angle = 0;

    if (map_initialized_) {

        // find mean and set theta_map_ and t_map_ (learn that mean isn't the best later)
        for (uint32_t i = 0; i < NUM_SAMPLES_; i++) {
            loc[0] += particles_[i].loc[0];
            loc[1] += particles_[i].loc[1];
            angle  += particles_[i].angle;
        }

        loc[0] /= NUM_SAMPLES_;
        loc[1] /= NUM_SAMPLES_;
        angle  /= NUM_SAMPLES_;
    }
}


}  // namespace particle_filter
