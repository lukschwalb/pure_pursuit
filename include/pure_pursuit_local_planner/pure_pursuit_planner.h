#ifndef CATKIN_WS_PURE_PURSUIT_PLANNER_H
#define CATKIN_WS_PURE_PURSUIT_PLANNER_H

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Core>
#include <dynamic_reconfigure/server.h>
#include <pure_pursuit_local_planner/transform_global_plan.h>
#include <pure_pursuit_local_planner/pure_pursuit_config.h>
#include <vector>

namespace pure_pursuit_local_planner {

  class PurePursuitPlanner : public nav_core::BaseLocalPlanner {

  public:
    PurePursuitPlanner();
    ~PurePursuitPlanner();

    /**
     * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
     * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
     * @return True if a valid velocity command was found, false otherwise
     */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
     * @brief  Check if the goal pose has been achieved by the local planner
     * @return True if achieved, false otherwise
     */
    bool isGoalReached();

    /**
     * @brief  Set the plan that the local planner is following
     * @param plan The plan to pass to the local planner
     * @return True if the plan was updated successfully, false otherwise
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief Constructs the local planner
     * @param name The name to give this instance of the local planner
     * @param tf A pointer to a transform listener
     * @param costmap_ros The cost map to use for assigning costs to local plans
     */
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  private:
    /**
    *@brief Reconfigure config_
    */
    void reconfigureCB(PurePursuitReconfigureConfig &config, uint32_t level);
    /**
      * @brief Get robot coordinates in local frame
      * @param name The name to give this instance of the local planner
      */

    bool initialized_;
    double last_vel_;
    ros::Publisher local_goal_publisher_;
    ros::Publisher global_goal_publisher_;
    //publisher where the local plan for visulatation is published
    ros::Publisher local_plan_publisher_;
    //costmap to get the current position
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;

    //used for transformation
    tf2_ros::Buffer* tf_;
    //true if the goal point is reache and orientation of goal is reached
    bool goal_reached_;
    //for dynamic reconfigure
    dynamic_reconfigure::Server<PurePursuitReconfigureConfig> *dsrv_;
    pure_pursuit_local_planner::PurePursuitConfig cfg_;
    //global plan which we run along
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    //transformed global plan in global frame with only the points with are needed for calculation (max_points)
    std::vector<geometry_msgs::PoseStamped> transformed_global_plan_;
    //last point of the global plan in global frame
    geometry_msgs::PoseStamped goal_pose_;
    geometry_msgs::PoseStamped final_goal_pose_;

  };
};


#endif //CATKIN_WS_PURE_PURSUIT_PLANNER_H
