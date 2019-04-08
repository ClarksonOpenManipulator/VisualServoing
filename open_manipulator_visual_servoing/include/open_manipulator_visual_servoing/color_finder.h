#ifndef _COLOR_FINDER_H_
#define _COLOR_FINDER_H_

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


class ColorFinder {

   private:
      ros::NodeHandle _nh;
      
      ros::Subscriber image_sub;
   
   public:
      ColorFinder();
      ~ColorFinder();
      
      cv::Scalar lower;
      cv::Scalar upper;
      
      void onImageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif // _COLOR_FINDER_H_
