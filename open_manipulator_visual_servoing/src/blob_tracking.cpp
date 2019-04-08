#include "open_manipulator_visual_servoing/blob_tracking.h"

BlobTracker::BlobTracker() {
   image_sub = _nh.subscribe("kinect2/sd/image_color_rect", 10, &BlobTracker::onImageCb, this);
   blob_service = _nh.advertiseService("visual_servoing/color/blob_finder", &BlobTracker::locateBlobCb, this);
}

BlobTracker::~BlobTracker() {

}

//Copies image from sensor_msg::Image type to cv::Mat
void BlobTracker::onImageCb(const sensor_msgs::ImageConstPtr& msg) {
   cv_bridge::CvImagePtr cvImagePtr;
   try {
      cvImagePtr = cv_bridge::toCvCopy(msg);
   } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }
   cvImageMat = cvImagePtr->image;
}

bool BlobTracker::locateBlobCb(open_manipulator_visual_servoing::LocateBlob::Request& req, 
                  open_manipulator_visual_servoing::LocateBlob::Response& res) {

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
   
   double maxArea = 0;
   cv::Rect boundingRect;
   for (int i = 0; i < contours.size(); i++) {
      double area = cv::contourArea(contours[i], false);
      if (area > maxArea) {
         maxArea = area;
         boundingRect = cv::boundingRect(contours[i]);
      }
   }
   int cX = boundingRect.x + boundingRect.width / 2;
   int cY = boundingRect.y + boundingRect.height / 2;
   
   //return coordinate of largest contour
   res.x = cX;
   res.y = cY;
   return true;
}


//Setup node
int main(int argc, char** argv) {
  ros::init(argc, argv, "blob_tracker");

  ROS_INFO("Starting blob tracker");
  BlobTracker blobTracker;

  ROS_INFO("Spinning...");
  ros::spin();

  return 0;
}
