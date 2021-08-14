#ifndef PURE_PURSUIT_CONFIG_H_
#define PURE_PURSUIT_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>

#include <pure_pursuit_local_planner/PurePursuitReconfigureConfig.h>


namespace pure_pursuit_local_planner
{
  class PurePursuitConfig
  {
  public:

    double max_planning_dist;
    double goal_accuracy;
    double velocity;
    double look_ahead_dist;
    double look_ahead_ratio;
    double road_dist_weight;
    double goal_dist_weight;
    
    PurePursuitConfig()
    {
      max_planning_dist = 1.0;
      goal_accuracy = 1.0;
      velocity = 0.5;
      look_ahead_dist = 0.1;
      look_ahead_ratio = 0.1;
      road_dist_weight = 1.0;
      goal_dist_weight = 1.0;
    }

    void loadParamFromNodeHandle(const ros::NodeHandle& nh);
    void reconfigure(PurePursuitReconfigureConfig& cfg);
  };


} // namespace pure_pursuit_local_planner

#endif
