#ifndef _BLOB_TRACKING_H_
#define _BLOB_TRACKING_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <vector>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "open_manipulator_visual_servoing/LocateBlob.h"


class BlobTracker {

   private:
      ros::NodeHandle _nh;
      
      ros::Subscriber image_sub;
      ros::ServiceServer blob_service;
      
      cv::Mat cvImageMat;
   public:
      BlobTracker();
      ~BlobTracker();
      
      void onImageCb(const sensor_msgs::ImageConstPtr& msg);
      bool locateBlobCb(open_manipulator_visual_servoing::LocateBlob::Request  &req, open_manipulator_visual_servoing::LocateBlob::Response  &res);
};

#endif // _BLOB_TRACKING_H_
