#include "open_manipulator_visual_servoing/color_finder.h"

//Handles changes in trackbars
static void onTrackbar(int value, void* data) {
   ColorFinder *finder = (ColorFinder*)data;
   
   finder->lower = cv::Scalar(cv::getTrackbarPos("lowB", "Tools"),
                              cv::getTrackbarPos("lowG", "Tools"),
                              cv::getTrackbarPos("lowR", "Tools"));
   finder->upper = cv::Scalar(cv::getTrackbarPos("highB", "Tools"),
                              cv::getTrackbarPos("highG", "Tools"),
                              cv::getTrackbarPos("highR", "Tools"));
}

//Handles mouse actions in cv windows
static void onMouse(int event, int x, int y, int flags, void* data) {
   if( event == cv::EVENT_LBUTTONDOWN) {
      ColorFinder *finder = (ColorFinder*)data;
      ROS_INFO_STREAM("("  << cv::getTrackbarPos("lowB", "Tools") << ", "
                           << cv::getTrackbarPos("lowG", "Tools") << ", "
                           << cv::getTrackbarPos("lowR", "Tools")  << ") -> ("
                           << cv::getTrackbarPos("highB", "Tools") << ", "
                           << cv::getTrackbarPos("highG", "Tools") << ", "
                           << cv::getTrackbarPos("highR", "Tools") << ")");
   }
}

ColorFinder::ColorFinder() {
   image_sub = _nh.subscribe("kinect2/sd/image_color_rect", 10, &ColorFinder::onImageCb, this);
   cv::namedWindow("Tools");
   
   lower = cv::Scalar(0, 0, 0);
   upper = cv::Scalar(255, 255, 255);
   
   //Setup trackbars
   int track0 = 0;
   int track255 = 255;
   cv::createTrackbar("lowB", "Tools", &track0, 255, onTrackbar, this);
   cv::createTrackbar("lowG", "Tools", &track0, 255, onTrackbar,  this);
   cv::createTrackbar("lowR", "Tools", &track0, 255, onTrackbar, this);
   cv::createTrackbar("highB", "Tools", &track255, 255, onTrackbar, this);
   cv::createTrackbar("highG", "Tools", &track255, 255, onTrackbar, this);
   cv::createTrackbar("highR", "Tools", &track255, 255, onTrackbar, this);
   //Setup mouse callback
   cv::setMouseCallback("Tools", onMouse, this);
}

ColorFinder::~ColorFinder() {

}

//Displays camera view and masked camera view
void ColorFinder::onImageCb(const sensor_msgs::ImageConstPtr& msg) {
   cv_bridge::CvImagePtr cvImagePtr;
   try {
      cvImagePtr = cv_bridge::toCvCopy(msg);
   } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }
   cv::Mat cvImageMat = cvImagePtr->image;
   cv::Mat maskMat;
   
   cv::inRange(cvImageMat, lower, upper, maskMat);
   
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
   
   //Debug
   cv::imshow("Mask", maskMat);
   cv::rectangle(cvImageMat, boundingRect, cv::Scalar(255, 255, 255), 3);
   
   cv::Mat palette(50, 500, CV_8UC3);
   
   cv::Point one(0, 0);
   cv::Point two(250, 50);
   cv::Point three(500, 0);
   cv::rectangle(palette, one, two, upper, -1);
   cv::rectangle(palette, two, three, lower, -1);
   
   int cX = boundingRect.x + boundingRect.width / 2;
   int cY = boundingRect.y + boundingRect.height / 2;
   cv::Point center(cX, cY);
   
   cv::circle(cvImageMat, center, 10, cv::Scalar(255, 255, 255), -1);
   cv::imshow("image", cvImageMat);
   cv::imshow("Tools", palette);
   cv::waitKey(1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "color_finder");

  ROS_INFO("Starting color finder");
  ColorFinder blobTracker;

  ROS_INFO("Spinning...");
  ros::spin();

  return 0;
}
