#ifndef __INCLUDE_LS_GLOBAL_LOCATION_H__
#define __INCLUDE_LS_GLOBAL_LOCATION_H__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

namespace LS_SLAM
{
  class LS_GLOBAL_LOCSTION
  {
    public:
      LS_GLOBAL_LOCSTION(ros::NodeHandle *nh);
      ~LS_GLOBAL_LOCSTION();

      void FindInitPose(void);

      bool finish_find_;

    private:
      void init(void);
      void MapCallBack(const nav_msgs::OccupancyGrid &msg);
      void LidarCallBack(const sensor_msgs::LaserScan &msg);
      void InitPosePublish(void);
      void GenerateGlobalMapCostMatrix(void);
      void RetrievalInitPose(void);
      void Display(void);

      nav_msgs::OccupancyGrid map_;
      sensor_msgs::LaserScan lidar_;
      geometry_msgs::PoseWithCovarianceStamped pose_;

      ros::Subscriber *lidar_sub_;
      ros::Subscriber *map_sub_;
      ros::Publisher *init_pose_pub_;
      ros::Publisher *cost_map_pub_;
      ros::Publisher *socre_map_pub_;

      short cost_map_[1000][1000];

      bool get_map_;
      bool get_lidar_;
      bool start_find_;
  };

  short cost_que_[2][1000*1000][2];
}

#endif
