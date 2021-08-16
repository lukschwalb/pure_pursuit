/**

Copyright (c) 2016, Marek Felix
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <pure_pursuit_local_planner/transform_global_plan.h>
#include <pure_pursuit_local_planner/misc.h>
#include<costmap_2d/costmap_layer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

using costmap_2d::FREE_SPACE;

namespace pure_pursuit_local_planner
{
  bool getGoalPose(const tf2_ros::Buffer& tf, const costmap_2d::Costmap2D& costmap,
                pure_pursuit_local_planner::PurePursuitConfig cfg, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                const geometry_msgs::PoseStamped& global_pose, geometry_msgs::PoseStamped& goal_pose,
                const std::string global_frame, ros::Publisher global_goal_publisher)
  {
    try 
    {
      if (global_plan.empty())
      {
          ROS_ERROR("Received plan with zero length");
          return false;
      }

      const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

      // get plan_to_global_transform from plan frame to global_frame
      geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, ros::Time(),
                                                                                    plan_pose.header.frame_id, ros::Duration(0.1));

      geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(plan_pose.header.frame_id, ros::Time(), global_frame, ros::Time(),
                                                                                    global_frame, ros::Duration(0.1));

      //let's get the pose of the robot in the frame of the plan
      geometry_msgs::PoseStamped robot_pose;
      tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);
      
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                      costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
      dist_threshold *= 0.85; // TODO: Make this configurable?
      int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = 1e10;
      geometry_msgs::PoseStamped furthest_pose;

      bool robot_reached = false;
      for(int j=0; j < (int)global_plan.size(); ++j)
      {
        double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
        double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
        double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

        if (robot_reached && new_sq_dist > sq_dist)
          break;

        if (new_sq_dist < sq_dist) // find closest distance
        {
          sq_dist = new_sq_dist;
          i = j;
          if (sq_dist < 0.005)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
            robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
        }
      }

      double plan_length = 0.0;
      sq_dist = 0.0;
      // Next we find the point in the global plan that is the furthest awaz
      while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (cfg.max_planning_dist<=0 || plan_length <= cfg.max_planning_dist))
      {
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        furthest_pose = pose;

        double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
        double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;
        
        // caclulate distance to previous pose
        if (i>0 && cfg.max_planning_dist>0)
          plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

        ++i;
      }
      global_goal_publisher.publish(furthest_pose);


      cv::Mat map, mask, dist_map, dist_map_32f;
      // Convert costmap to opencv Mat    
      unsigned char *char_map = costmap.getCharMap();
      cv::Size map_size(costmap.getSizeInCellsY(), costmap.getSizeInCellsX());
      map = cv::Mat(map_size, CV_8UC1, char_map).clone();

      // set all free to space 255, lethal and unknown to 0
      cv::inRange(map, FREE_SPACE, FREE_SPACE, mask);
      map = 0;
      map.setTo(255, mask);

      // Perform distance transform
      cv::distanceTransform(map, dist_map_32f, cv::DIST_L2, 3);
      //cv::normalize(dist_32f, dist_32f, 0, 255, cv::NORM_MINMAX);
      dist_map_32f.convertTo(dist_map, CV_8U);

      double dist_min, dist_max;
      cv::minMaxLoc(dist_map, &dist_min, &dist_max);

      geometry_msgs::PoseStamped tmp_pose, cmp_pose;
      double goal_cost = 1000000000000000;

      for(int y=0; y < costmap.getSizeInCellsY(); y++) {
        uchar* map_values = map.ptr<uchar>(y);
        uchar* distance_values = dist_map.ptr<uchar>(y);

        for(int x=0; x < costmap.getSizeInCellsX(); x++) {
          int map_value = map_values[x];
          uchar dist_value = distance_values[x];

          if(map_value == 255) {
            tmp_pose.header.stamp = ros::Time::now();
            tmp_pose.header.frame_id = global_pose.header.frame_id;
            tmp_pose.pose.position.x = global_pose.pose.position.x + ((x - map.rows / 2) * costmap.getResolution());
            tmp_pose.pose.position.y = global_pose.pose.position.y + ((y - map.cols / 2) * costmap.getResolution());

            tf2::doTransform(tmp_pose, cmp_pose, global_to_plan_transform);
            double dist_x = cmp_pose.pose.position.x;
            double dist_y = cmp_pose.pose.position.y;

            double x_diff = dist_x - furthest_pose.pose.position.x;
            double y_diff = dist_y - furthest_pose.pose.position.y;

            double cost = distance_points2d(cmp_pose.pose.position, furthest_pose.pose.position) * cfg.goal_dist_weight;
            cost += (dist_max - dist_value) * cfg.road_dist_weight;
            //ROS_INFO("C1: %f   C2: %f    C1_F: %f,   C2_F: %f", distance_points2d(cmp_pose.pose.position, furthest_pose.pose.position),(dist_max - dist_value), distance_points2d(cmp_pose.pose.position, furthest_pose.pose.position) * cfg.goal_dist_weight, (dist_max - dist_value) * cfg.road_dist_weight);

            if(cost < goal_cost) {
              goal_cost = cost;
              goal_pose = tmp_pose;
            }
          }
        }
      }
    }
    catch(tf::LookupException& ex)
    {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) 
    {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) 
    {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      return false;
    }

    return true;
  }
};
