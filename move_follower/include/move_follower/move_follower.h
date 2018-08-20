/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_MOVE_FOLLOWER_ACTION_H_
#define NAV_MOVE_FOLLOWER_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <nav_msgs/Odometry.h>  // wb

#include <straight_planner/straight_planner.h>   // wb

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_follower/MoveFollowerConfig.h"

namespace move_follower {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  enum MoveFollowerState {
    PLANNING,
    CONTROLLING,
    CLEARING
  };

  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

  /**
   * @class MoveFollower
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class MoveFollower {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveFollower(tf::TransformListener& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveFollower();

      /**
       * @brief  Performs a control cycle
       * @param goal A reference to the goal to pursue
       * @param global_plan A reference to the global plan being used
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf::TransformListener& tf_;

      MoveBaseActionServer* as_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;

      
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> sc_;   // wb
      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      std::string robot_base_frame_, global_frame_;

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      unsigned int recovery_index_;

      tf::Stamped<tf::Pose> global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_;
      uint32_t planning_retries_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      double oscillation_timeout_, oscillation_distance_;

      MoveFollowerState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> sgp_loader_;      // wb
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;

      //set up the planner's thread
      bool runPlanner_;
      boost::recursive_mutex planner_mutex_;
      boost::condition_variable_any planner_cond_;   // 多进程管理
      geometry_msgs::PoseStamped planner_goal_;
      boost::thread* planner_thread_;


      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_follower::MoveFollowerConfig> *dsrv_;
      
      void reconfigureCB(move_follower::MoveFollowerConfig &config, uint32_t level);

      move_follower::MoveFollowerConfig last_config_;
      move_follower::MoveFollowerConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;
      
      // wb
      bool turn_in_place_first_;
      double max_vel_lin_, max_vel_th_;
      double min_in_place_vel_th_, in_place_trans_vel_;
      double min_vel_lin_, min_vel_th_;
      double max_vel_lin_acel_,max_vel_th_acel_;
      double max_vel_lin_dacel_,max_vel_th_dacel_;
      double publish_twist_frequence_;
      double accept_goal_frequence_;
      double goal_distance_;
      double max_heading_diff_before_moving_;
      double linear_scale_factor_;
      double angular_scale_factor_;
      geometry_msgs::PoseStamped last_relative_goal_, relative_goal_; 
      nav_msgs::Odometry base_odom_;
      ros::Subscriber odom_sub_;
      boost::mutex odom_lock_;
      
      geometry_msgs::Twist computeVelocity(const geometry_msgs::PoseStamped& last_relative_goal, const geometry_msgs::PoseStamped& relative_goal);
      geometry_msgs::Twist limitVelocity(const geometry_msgs::Twist& twist);
      void publishVelocity(const geometry_msgs::Twist& finaltwist);
      inline double sign(double n){
            return n < 0.0 ? -1.0 : 1.0;
          }
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  };
};
#endif

