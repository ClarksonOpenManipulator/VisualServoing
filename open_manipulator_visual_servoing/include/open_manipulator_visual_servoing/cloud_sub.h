#ifndef _CLOUD_SUB_H_
#define _CLOUD_SUB_H_

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "open_manipulator_visual_servoing/IndexPointCloud.h"

class CloudSub {

   private:
      ros::NodeHandle _nh;
      
      pcl::PointCloud<pcl::PointXYZ> cloud;
      
      ros::Subscriber cloud_sub;
      ros::ServiceServer indexer_service;
      
   public:
      CloudSub();
      ~CloudSub();
      
      void onCloudCb(const sensor_msgs::PointCloud2ConstPtr& msg);
      bool indexerCb(open_manipulator_visual_servoing::IndexPointCloud::Request  &req, open_manipulator_visual_servoing::IndexPointCloud::Response  &res);
};


#endif // _CLOUD_SUB_H_
