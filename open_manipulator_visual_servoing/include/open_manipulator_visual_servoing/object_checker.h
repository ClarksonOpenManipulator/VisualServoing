#ifndef _OBJECT_CHECKER_H_
#define _OBJECT_CHECKER_H_

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/node_handle.h>

#include <open_manipulator_visual_servoing/ObjectChecking.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>


class ObjectChecker {

   private:
      ros::NodeHandle _nh;
      
      ros::Subscriber camera_sub;
		ros::Publisher debug_image;
      ros::ServiceServer checker_service;

		cv::Mat cvImageMat;
      
   public:
      ObjectChecker();
      ~ObjectChecker();
      
      bool checkerCb(open_manipulator_visual_servoing::ObjectChecking::Request& req, open_manipulator_visual_servoing::ObjectChecking::Response& res);
		void cameraCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif // _OBJECT_CHECKER_H_
