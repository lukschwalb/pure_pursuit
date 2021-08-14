
#include <pure_pursuit_local_planner/pure_pursuit_config.h>

namespace pure_pursuit_local_planner
{
    
void PurePursuitConfig::loadParamFromNodeHandle(const ros::NodeHandle& nh)
{
  nh.param("max_planning_dist", max_planning_dist, max_planning_dist);
  nh.param("goal_accuracy", goal_accuracy, goal_accuracy);
  nh.param("velocity", velocity, velocity);
  nh.param("look_ahead_dist", look_ahead_dist, look_ahead_dist);
  nh.param("look_ahead_ratio", look_ahead_ratio, look_ahead_ratio);
  nh.param("road_dist_weight", road_dist_weight, road_dist_weight);
  nh.param("goal_dist_weight", goal_dist_weight, goal_dist_weight);
}

void PurePursuitConfig::reconfigure(PurePursuitReconfigureConfig& cfg)
{   
  max_planning_dist = cfg.max_planning_dist;
  goal_accuracy = cfg.goal_accuracy;
  velocity = cfg.velocity;
  look_ahead_dist = cfg.look_ahead_dist;
  look_ahead_ratio = cfg.look_ahead_ratio;
  road_dist_weight = cfg.road_dist_weight;
  goal_dist_weight = cfg.goal_dist_weight;
}

    
} // namespace pure_pursuit_local_planner
