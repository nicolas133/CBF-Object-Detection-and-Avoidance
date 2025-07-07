#include </home/jetson/Rosmaster_R2/yahboomcar_ws/src/toTarget/include/toTarget/PID_localplanner.h>
#include <pluginlib/class_list_macros.h>

// an attempt to integrate PID into the local planner plguin in order to get it to work with all the already predefined global planners 

//#include </home/jetson/Rosmaster_R2/yahboomcar_ws/src/toTarget/srv/ODOMSERV.h>

PLUGINLIB_EXPORT_CLASS(pid_local_planner::PIDLocalPlanner, nav_core::BaseLocalPlanner)



  
namespace pid_local_planner{
 

// Bring in pidlocalplannner from pid local planner class default constructor
PIDLocalPlanner::PIDLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized(false) {}


// Parameterized constructor
PIDLocalPlanner::PIDLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized(false), goal_reached_(false), plan_index_(0),
      current_x(0.0), current_y(0.0) {
    initialize(name, tf, costmap_ros);
    ROS_INFO("IN COnSTRUCTOR");// will need to name node in future
}

// destructor 
PIDLocalPlanner::~PIDLocalPlanner() {}


//initalization function 
// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void PIDLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                               costmap_2d::Costmap2DROS* costmap_ros)
{
     ROS_INFO("IN INITIALIZED");
     if(!initialized) {

         ros::NodeHandle private_nh;
         ros::NodeHandle Odom_nh;
         

         vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
       
     

         odom_sub_ = Odom_nh.subscribe("/odometry/filtered",1000,&PIDLocalPlanner::odomMsgCallback,this);
    


         odom_sub=Odom_nh.advertiseService("odom_serv",&PIDLocalPlanner::odomCallback,this);

 

         initialized = true;
         ROS_INFO("PID_LOCALPLANNER HAS INTIALIZED YAY YAY YAY YAY YAY YAY!");
         tf_ = tf;
         costmap_ros_ = costmap_ros;
     }
}

//set the global plan 
bool PIDLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
 )
 {
     if(!initialized)
     {
         ROS_ERROR("This planner has not been initialized");
         return false;
     }
     global_plan_ = orig_global_plan;
     goal_reached_ = false;
     plan_index_ = 0;
     return true;

     ROS_INFO("WE ARE IN setPLan");
 }

//compute vel commands----PID------
bool PIDLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
 {
     if(!initialized)
     {
         ROS_ERROR("This planner has not been initialized");
         return false;
     }

     if (goal_reached_ || global_plan_.empty()) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return false;
    }

     ROS_INFO("Start ComputeVEl");
     ros::ServiceClient client= private_nh.serviceClient<toTarget::ODOMSERV>("odom_serv");
     // name of srv file
     toTarget::ODOMSERV srv;
     srv.request.odom_req=current_odom_;

     ROS_INFO("CurrentODOM IS %f",current_odom_.pose.pose.position.x);
       
     // might need to create new node handle
     // creates cleint for odom_serv service
     // not sire if this is correct 

     
    current_steering_angle_=current_odom_.pose.pose.orientation;
    current_x = current_odom_.pose.pose.position.x;
    //ROS_INFO("current steering angle is %f",current_steering_angle_.x);
    
    current_y = current_odom_.pose.pose.position.y;

    

    
    const geometry_msgs::PoseStamped& goal_pose = global_plan_[plan_index_];
    double goal_x = goal_pose.pose.position.x;
    double goal_y = goal_pose.pose.position.y;
    
   // tf::Stamped<tf::Pose> robot_pose;
  //  costmap_ros_->getRobotPose(robot_pose);
   // robot_pose=PoseSE2(robot_pose);
    

    double x_diff = goal_x - current_x;
    double y_diff = goal_y - current_y;
    double distance = sqrt(x_diff * x_diff + y_diff * y_diff);

    if (distance < 0.1) {
        if (plan_index_ + 1 < global_plan_.size()) {
            plan_index_++;
        } else {
            goal_reached_ = true;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            return true;
        }
    }

    //desired angle 
    double desired_angle = angleCalc(current_odom_, goal_pose); 


    double mag = sqrt(current_steering_angle_.x*current_steering_angle_.x + current_steering_angle_.y*current_steering_angle_.y+current_steering_angle_.z*current_steering_angle_.z + current_steering_angle_.w*current_steering_angle_.w);
   ROS_INFO("THIS IS CSA %f,%f,%f,%f", current_steering_angle_.x, current_steering_angle_.y, current_steering_angle_.z, current_steering_angle_.w);// yaw ouputed is incorrect
   
   if(mag!=0){
  // used to normalize the steering angle 
   ROS_INFO("NORMAlized BITCH");
   current_steering_angle_.x/=mag;
   current_steering_angle_.y/=mag;
   current_steering_angle_.z/=mag;
   current_steering_angle_.w/=mag;
   
}

// angle that we are at 
    double yaw = tf::getYaw(current_steering_angle_);
    ROS_INFO("THIS IS YAW %f ",yaw);// yaw ouputed is incorrect
    ROS_INFO("THIS IS mag %f ",mag);// yaw ouputed is incorrect
    
   
    


   // Get angle dif and wrap to -pi to pi
   double angle_diff = angles::shortest_angular_distance(yaw, desired_angle); 
   ROS_INFO("THIS IS Desired Angle %f ",desired_angle);

    //cmd_vel.linear.x = pidCompute(0.3, current_odom_.twist.twist.linear.x);
    cmd_vel.linear.x=-.5;
   
   
    //ROS_INFO("hELLLO");
    ROS_INFO("THIS IS ANGLE %f ",angle_diff);

    
    cmd_vel.angular.z = angle_diff;
    //cmd_vel.angular.z = .5;
    //cmd_vel.angular.z=.2;
      


    
    //ros::spin();
   
    vel_pub_.publish(cmd_vel);
     return true;
    
 }

//check if goal has been reached 
bool PIDLocalPlanner::isGoalReached() {
     return goal_reached_;
     ROS_INFO("WE ARE IN isGoalReached");
 
 }

// Service Function 
  bool PIDLocalPlanner::odomCallback(toTarget::ODOMSERV::Request &req,toTarget::ODOMSERV::Response &res) {
    
     current_odom_ = req.odom_req;
    //based of type defined in srv
    
    res.odom_rec=current_odom_;
    //set input=to output

    ROS_WARN("ODOMCALLBACK COMPLETED");
    return true;
    
    

}

//Callback for Service Function
void PIDLocalPlanner::odomMsgCallback(const nav_msgs::Odometry::ConstPtr& msg){
//not sure if it should be const nav_msgs::Odometry::ConstPtr& ms


ROS_INFO("withinodommsgcallback");
current_odom_=*msg;
}


// PID control calculation
double PIDLocalPlanner::pidCompute(double target, double current) {
    static double previous_error = 0.0;
    static double integral = 0.0;
    double error = target - current;
    integral += error;
    double derivative = error - previous_error;
    previous_error = error;
    return 1.0 * error + 0.0 * integral + 0.0 * derivative;  // Adjust these gains as necessary
}

// Calculate the desired angle to move from the current pose to the goal pose
double PIDLocalPlanner::angleCalc(const nav_msgs::Odometry& current_pose, const geometry_msgs::PoseStamped& goal_pose) {
    double diff_y = goal_pose.pose.position.y - current_pose.pose.pose.position.y;
    double diff_x = goal_pose.pose.position.x - current_pose.pose.pose.position.x;
    return atan2(diff_y, diff_x);
}

// Ensure the calculated angle is within the valid range
double PIDLocalPlanner::angleVerifier(double angle) {
    if (angle > M_PI) {
        return angle - 2 * M_PI;
    } else if (angle < -M_PI) {
        return angle + 2 * M_PI;
    } else {
        return angle;
    }
}
}

