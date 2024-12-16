#ifndef PID_LOCALPLANNER_H_
#define PID_LOCALPLANNER_H_
#pragma once
  
// abstract class from which our plugin inherits

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_local_planner.h>    

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf/tf.h>

#include <costmap_2d/costmap_2d_ros.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

#include <dynamic_reconfigure/server.h>
//#include <pid_local_planner/PIDLocalPlannerConfig.h>

#include <angles/angles.h>
#include <vector>
#include <cmath>





//#include "/home/jetson/Rosmaster_R2/yahboomcar_ws/src/toTarget/srv/ODOMSERV.srv"

//#include </home/jetson/Rosmaster_R2/yahboomcar_ws/src/toTarget/include/toTarget/ODOMSERV.h>
#include "toTarget/ODOMSERV.h"







//include header generated from srv file 
 
using namespace std;
 
namespace pid_local_planner{
 

//start defining functions that will be used in the cpp file 
class PIDLocalPlanner : public nav_core::BaseLocalPlanner{
public:
     PIDLocalPlanner(); //default constructer 
     PIDLocalPlanner(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros); // init

     ~PIDLocalPlanner(); //default destructer

     void initialize(std::string name, tf2_ros::Buffer* tf,
                     costmap_2d::Costmap2DROS* costmap_ros); 
     bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
   
     bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
   
     bool isGoalReached();

     double angleVerifier(double);


     bool odomCallback(toTarget::ODOMSERV::Request &req,toTarget::ODOMSERV::Response &res);
     void odomMsgCallback(const nav_msgs::Odometry::ConstPtr& msg); //create a callback for the service 

     
   private:
      //grabbing variables from outside namespace
 
      ros::NodeHandle private_nh;// for cmd publisher
      ros::NodeHandle Odom_nh; // for odom service 



  
      tf2_ros::Buffer* tf_;
  
      costmap_2d::Costmap2DROS* costmap_ros_;
      //boost::mutex odom_lock_;
      ros::Publisher vel_pub_;
      ros::Subscriber odom_sub_;
      ros::ServiceServer odom_sub;

      nav_msgs::Odometry current_odom_;
      geometry_msgs::PoseStamped current_pose;
  

      std::vector<geometry_msgs::PoseStamped> global_plan_;
      //dynamic_reconfigure::Server<pid_local_planner::PIDConfig> *dsrv_;
      //define functions local to this class
      bool goal_reached_;
      bool initialized;
      int plan_index_;

      double current_vel_x_;
      double current_vel_y_;
      geometry_msgs::Quaternion current_steering_angle_;
   
      //double current_odom_;
      double current_x;
      double current_y;
      bool first_run_;
      double initial_x_;
      double initial_y_;

      int wayindex_;
      double old_x_position_error_;
      bool error_flag_;

      //function creation 
     
      double pidCompute(double target, double current);
      double angleCalc(const nav_msgs::Odometry& current_pose, const geometry_msgs::PoseStamped& goal_pose);
      
};
};
 
#endif
 

