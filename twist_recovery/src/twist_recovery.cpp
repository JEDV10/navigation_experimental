/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <twist_recovery/twist_recovery.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>

// Always necesary to register as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(twist_recovery::TwistRecovery, nav_core::RecoveryBehavior)

using std::vector;
using std::max;

namespace twist_recovery
{

  // Constructor
  TwistRecovery::TwistRecovery () :
  global_costmap_(NULL), local_costmap_(NULL), tf_(NULL), initialized_(false)
  {}

    // Destructor
    TwistRecovery::~TwistRecovery ()
    {
      delete world_model_;
    }

    // Initialize the recovery reading all necessary params
    void TwistRecovery::initialize (std::string name, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* global_cmap, costmap_2d::Costmap2DROS* local_cmap)
      {
        ROS_ASSERT(!initialized_);
        name_ = name;
        tf_ = tf;
        local_costmap_ = local_cmap;
        global_costmap_ = global_cmap;
        world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

        pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        ros::NodeHandle private_nh("~/" + name);
        /*
        {
        bool found=true;
        found = found && private_nh.getParam("linear_x", base_frame_twist_.linear.x);
        found = found && private_nh.getParam("linear_y", base_frame_twist_.linear.y);
        found = found && private_nh.getParam("angular_z", base_frame_twist_.angular.z);
        if (!found) {
          ROS_FATAL_STREAM ("Didn't find twist parameters in " << private_nh.getNamespace());
          ros::shutdown();
        }
        }
        */

    private_nh.param("linear_x", base_frame_twist_.linear.x, -0.2);
    private_nh.param("linear_y", base_frame_twist_.linear.y, 0.0);
    private_nh.param("angular_z", base_frame_twist_.angular.z, 0.0);

    private_nh.param("duration", duration_, 3.0);
    private_nh.param("linear_speed_limit", linear_speed_limit_, 0.3);
    private_nh.param("angular_speed_limit", angular_speed_limit_, 1.0);
    private_nh.param("linear_acceleration_limit", linear_acceleration_limit_, 4.0);
    private_nh.param("angular_acceleration_limit", angular_acceleration_limit_, 3.2);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("simulation_inc", simulation_inc_, 1/controller_frequency_);

    ROS_INFO_STREAM_NAMED ("top", "Initialized twist recovery with twist " <<
    base_frame_twist_ << " and duration " << duration_);

    initialized_ = true;
  }

  geometry_msgs::Twist scaleTwist (const geometry_msgs::Twist& twist, const double scale)
  {
    geometry_msgs::Twist t;
    t.linear.x = twist.linear.x * scale;
    t.linear.y = twist.linear.y * scale;
    t.angular.z = twist.angular.z * scale;
    return t;
  }

  geometry_msgs::Pose2D forwardSimulate (const geometry_msgs::Pose2D& p, const geometry_msgs::Twist& twist, const double t=1.0)
  {
    geometry_msgs::Pose2D p2;
    p2.x = p.x + twist.linear.x*t;
    p2.y = p.y + twist.linear.y*t;
    p2.theta = p.theta + twist.angular.z*t;
    return p2;
  }

  // Return the cost of a pose, modified so that -1 does not equal infinity; instead 1e9 does.
  double TwistRecovery::normalizedPoseCost (const geometry_msgs::Pose2D& pose) const
  {
    const double c = world_model_->footprintCost(pose.x, pose.y, pose.theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
    return c < 0 ? 1e9 : c;
  }


  // Return the maximum d <= duration_ such that starting at the current pose,
  // the cost is nonincreasing for d seconds if we follow twist
  // There is also a minimum time that we are allowed to have lethal cost for
  // the first k of those d seconds.
  double TwistRecovery::nonincreasingCostInterval (const geometry_msgs::Pose2D& current_position, const geometry_msgs::Twist& twist, const double min_time=0.0) const
  {
    ROS_INFO_NAMED ("top", "Time allowing lethal cost: %.2f", min_time);
    if(min_time > duration_) {
      ROS_ERROR_STREAM_NAMED("top", "Parameter 'min_time' greater than maximum duration allowed to twist_recovery [" << duration_ << "]");
      return 0;
    }
    double cost = normalizedPoseCost(current_position); // Cost of the original positon
    double cost_after_min_time = normalizedPoseCost(forwardSimulate(current_position, twist, min_time)); // Cost after the first k seconds we allow lethal cost
    if (cost_after_min_time > cost) {
      ROS_WARN_STREAM_NAMED("cost", "Cost after time allowing lethal cost [" << cost_after_min_time <<
      "] is greater than original cost [" << cost << "]");
    }

    double t; // Will hold the first time that is invalid
    cost = cost_after_min_time;
    for (t=min_time+simulation_inc_; t<=duration_; t+=simulation_inc_) {
      const double next_cost = normalizedPoseCost(forwardSimulate(current_position, twist, t));
      if (next_cost > cost) {
        ROS_DEBUG_STREAM_NAMED ("cost", "Cost after " << t << "seconds and pose " << forwardSimulate(current_position, twist, t)
        << " [" << next_cost << "] is greater than previous cost [" << cost << "]");
        break;
      }
      cost = next_cost;
    }

    return t-simulation_inc_;
  }

  double linearSpeed (const geometry_msgs::Twist& twist)
  {
    return sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y);
  }

  double angularSpeed (const geometry_msgs::Twist& twist)
  {
    return fabs(twist.angular.z);
  }

  // Scale twist so we can stop in the given time, and so it's within the max velocity
  geometry_msgs::Twist TwistRecovery::scaleGivenAccelerationLimits (const geometry_msgs::Twist& twist, const double time_remaining) const
  {
    const double linear_speed = linearSpeed(twist);
    const double angular_speed = angularSpeed(twist);
    const double linear_acc_scaling = linear_speed/(time_remaining*linear_acceleration_limit_);
    const double angular_acc_scaling = angular_speed/(time_remaining*angular_acceleration_limit_);
    const double acc_scaling = max(linear_acc_scaling, angular_acc_scaling);
    const double linear_vel_scaling = linear_speed/linear_speed_limit_;
    const double angular_vel_scaling = angular_speed/angular_speed_limit_;
    const double vel_scaling = max(linear_vel_scaling, angular_vel_scaling);
    return scaleTwist(twist, max(1.0, max(acc_scaling, vel_scaling)));
  }

  // Get pose in local costmap frame
  geometry_msgs::Pose2D TwistRecovery::getCurrentLocalPose () const
  {
    tf::Stamped<tf::Pose> p;
    local_costmap_->getRobotPose(p);
    geometry_msgs::Pose2D pose;
    pose.x = p.getOrigin().x();
    pose.y = p.getOrigin().y();
    pose.theta = tf::getYaw(p.getRotation());
    return pose;
  }

  void TwistRecovery::runBehavior ()
  {
    ROS_ASSERT (initialized_);

    // Threre is a maximum duration this recovery can run defined in duration_.
    // However, there may be an obstacle so it is necessary to figure out how
    // long we can safely run the behavior.
    const geometry_msgs::Pose2D& current_pose = getCurrentLocalPose();
    const double max_duration = nonincreasingCostInterval(current_pose, base_frame_twist_);
    ros::Rate r(controller_frequency_);
    ROS_WARN_NAMED ("top", "Applying Twist recovery (%.2f, %.2f, %.2f) for %.2f seconds", base_frame_twist_.linear.x,
    base_frame_twist_.linear.y, base_frame_twist_.angular.z, max_duration);

    // We will now apply this twist open-loop for d seconds (scaled so we can
    // guarantee stopping at the end)
    for (double t=0; t<max_duration; t+=1/controller_frequency_) {
      pub_.publish(scaleGivenAccelerationLimits(base_frame_twist_, max_duration-t));
      r.sleep();
    }
  }

} // namespace twist_recovery
