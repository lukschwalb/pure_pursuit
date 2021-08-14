
#include <ros/ros.h>
#include <pure_pursuit_local_planner/pure_pursuit_planner.h>
#include <pure_pursuit_local_planner/misc.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(pure_pursuit_local_planner::PurePursuitPlanner, nav_core::BaseLocalPlanner)

namespace pure_pursuit_local_planner {

  PurePursuitPlanner::PurePursuitPlanner() : initialized_(false){}

  PurePursuitPlanner::~PurePursuitPlanner() {}

  void PurePursuitPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  {
    ros::NodeHandle nh("~/" + name);
    local_plan_publisher_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    tf_ = tf;
    goal_reached_ = false;

    local_goal_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("local_goal_pose", 1);
    global_goal_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("global_goal_pose", 1);

    //Parameter for dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<PurePursuitReconfigureConfig>(nh);
    dynamic_reconfigure::Server<PurePursuitReconfigureConfig>::CallbackType cb = boost::bind(&PurePursuitPlanner::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    last_vel_ = cfg_.velocity;

    ROS_INFO("PurePursuitPlanner Initialized");
    initialized_ = true;
  }


  void PurePursuitPlanner::reconfigureCB(PurePursuitReconfigureConfig &config, uint32_t level)
  {
    cfg_.reconfigure(config);
  }

  bool PurePursuitPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
  {
    global_plan_.clear();
    global_plan_ = plan;
    goal_reached_ = false;
    final_goal_pose_ = plan[plan.size() - 1];

    return true;
  }

  bool PurePursuitPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    if(!initialized_)
    {
      ROS_ERROR("PurePursuitPlanner is not yet initialized");
      return false;
    }
    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    const std::string global_frame = costmap_ros_->getGlobalFrameID();
    pure_pursuit_local_planner::getGoalPose(*tf_, *costmap_, cfg_, global_plan_, robot_pose, goal_pose_, global_frame, global_goal_publisher_);

    local_goal_publisher_.publish(goal_pose_);
    geometry_msgs::TransformStamped global_to_base_transform;
    try {
      global_to_base_transform = tf_->lookupTransform("base_link", ros::Time(), global_frame, ros::Time(),
                                                                                    "base_link", ros::Duration(0.2));
    }
    catch(tf::ExtrapolationException& ex) 
    {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      return false;
    }

    nav_msgs::Path local_plan;
    local_plan.header.frame_id = "base_link";
    local_plan.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped goal_pose_base;
    tf2::doTransform(goal_pose_, goal_pose_base, global_to_base_transform);

    // Lookahead distance
		double ld = cfg_.look_ahead_ratio * last_vel_ + cfg_.look_ahead_dist;
    double ld_2 = ld * ld;

    // x and y difference from robot pose to goal pose in base frame
		double dy = goal_pose_base.pose.position.y;
		double dx = goal_pose_base.pose.position.x;


    // Gradient of line connecting (0|0) and (dx|dy)
    double gradient = dx / dy;
    // Y Position where line has a length of ld
    double py = ld / std::sqrt( 1 + gradient*gradient );

    // Curvature of the arc connecting (0|0) and (px|py)
    double curvature = (2*py)/ld_2;
    // Radius of the circle that describes the arc
    double radius = 1/curvature;

    for(double _y = 1; _y >= 1 - (py / radius); _y -= (py / radius) / 10) {
      double alpha = acos(_y);
      double _x = sin(alpha);
      double pos_x = _x * radius;
      double pos_y = (1 - _y) * radius;
      pos_y = std::copysign(pos_y, gradient);

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = local_plan.header.frame_id;
      pose.header.stamp = local_plan.header.stamp;
      pose.pose.position.x = pos_x;
      pose.pose.position.y = pos_y;
      local_plan.poses.push_back(pose);
    }

    local_plan_publisher_.publish(local_plan);
    double ang_z = (2 * cfg_.velocity) / ld_2 * py;
    ang_z = std::copysign(ang_z, gradient);
    cmd_vel.angular.z = ang_z;

    cmd_vel.linear.x = cfg_.velocity;
    last_vel_ = cfg_.velocity;

    if(distance_points2d(robot_pose.pose.position, goal_pose_.pose.position) <= cfg_.goal_accuracy)
    {
        goal_reached_ = true;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        last_vel_ = 0;
    }

    return true;
  }


  bool PurePursuitPlanner::isGoalReached()
  {
    if(goal_reached_)
    {
        ROS_INFO("PurePursuitPlanner: Goal reached.");
    }
    return goal_reached_;
  }

}; 

