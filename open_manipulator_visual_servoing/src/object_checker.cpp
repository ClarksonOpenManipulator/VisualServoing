#include "open_manipulator_visual_servoing/object_checker.h"

#include <cmath>

ObjectChecker::ObjectChecker() {
   camera_sub = _nh.subscribe("cv_camera/image_raw", 10, &ObjectChecker::cameraCb, this);
	debug_image = _nh.advertise<sensor_msgs::Image>("debug_image", 10);
   checker_service = _nh.advertiseService("visual_servoing/object/checker", &ObjectChecker::checkerCb, this);
}

ObjectChecker::~ObjectChecker() {

}

//Copies image from sensor_msg::Image type to cv::Mat
void ObjectChecker::cameraCb(const sensor_msgs::ImageConstPtr& msg) {
   cv_bridge::CvImagePtr cvImagePtr;
   try {
      cvImagePtr = cv_bridge::toCvCopy(msg);
   } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }
   cvImageMat = cvImagePtr->image;
}

bool ObjectChecker::checkerCb(open_manipulator_visual_servoing::ObjectChecking::Request& req, 
               open_manipulator_visual_servoing::ObjectChecking::Response& res) {
               
   //Checks that request colors have at least 6 8 bit numbers (colors)
	if (req.colors.size() < 6) {
      ROS_WARN_STREAM("Minimum of 6 color values required [lowB, lowG, lowR, highB, highG, highR]");
      return false;
   }
	for (size_t i = 0; i < req.colors.size(); i++) {
      if (req.colors[i] > 255 || req.colors[i] < 0) {
         ROS_WARN_STREAM("Color values must be between 0 and 255. colors[" << i << "] = " << req.colors[i]);
         return false;
      }
   }
   
   //Build color range limits
   cv::Scalar lower(req.colors[0], req.colors[1], req.colors[2]);
   cv::Scalar upper(req.colors[3], req.colors[4], req.colors[5]);
   cv::Mat maskMat;
   
   //Creates mask image (white = in range, black = not)
   cv::inRange(cvImageMat, lower, upper, maskMat);

   //Locate continuous areas of mask
   std::vector<std::vector<cv::Point>> contours;
   std::vector<cv::Vec4i> hierarchy;
   cv::findContours(maskMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

   //Select largest contour and get min area rect
   double maxArea = 0;
   cv::RotatedRect boundingRect;
   for (int i = 0; i < contours.size(); i++) {
      double area = cv::contourArea(contours[i], false);
      if (area > maxArea) {
         maxArea = area;
         boundingRect = cv::minAreaRect(contours[i]);
      }
   }
 
   //Creates mats for transformed images
   cv::Mat rotationMatrix, rotated, cropped;
   cv::Point2f points[4];
   boundingRect.points(points);   
   
   //Get rotattion angle for AABB
   float angle = boundingRect.angle;
   
   //Get AABB size
   double adjustment = boundingRect.size.height / 4;
   boundingRect.size.width -= adjustment;
   boundingRect.size.height -= adjustment;
	
   const cv::Point2f center = boundingRect.center;
   const cv::Size2f size = boundingRect.size;
   
   //Rotates and crops image to rotated rectangle
   rotationMatrix = getRotationMatrix2D(center, angle, 1.0);
   warpAffine(cvImageMat, rotated, rotationMatrix, maskMat.size());
   getRectSubPix(rotated, size, center, cropped);
   
   //Searches for white contours within reduced image
   contours.clear();
   hierarchy.clear();
   cv::Mat dotsMat;
   cv::inRange(cropped, cv::Scalar(150, 150, 150), cv::Scalar(255, 255, 255), dotsMat);
   cv::findContours(dotsMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
   
   //Counts countours area greater than threshold (8)
   int dotCount = 0;
   ROS_INFO_STREAM("Contour count: " << contours.size());
   for (int i = 0; i < contours.size(); i++) {
      float radius;
      cv::Point2f dotCenter;
      cv::minEnclosingCircle(contours[i], dotCenter, radius);
      if (radius > 8) dotCount++; //TODO: replace literal 8 with parameter
   }
   
   ROS_INFO_STREAM("Dot count: " << dotCount << " contours.size(): " << contours.size());
   
   //Service respond with dot count
   res.object_value = dotCount;
   
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img_msg;
   
   //Publishes cropped image for debugging
	std_msgs::Header header;
	header.seq = 0;
	header.stamp = ros::Time::now();
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cropped);
	img_bridge.toImageMsg(img_msg);
	debug_image.publish(img_msg);
	
	return true;
}

//Setup node
int main(int argc, char* argv[]) {
   ros::init(argc, argv, "object_checker");
   
   ROS_INFO("Starting object checker");
   ObjectChecker objectChecker;
   
   ROS_INFO("Spinning...");
   ros::spin();
   return 0;
}
