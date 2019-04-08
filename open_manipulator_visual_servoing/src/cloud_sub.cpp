#include "open_manipulator_visual_servoing/cloud_sub.h"

CloudSub::CloudSub() {
   cloud_sub = _nh.subscribe("kinect2/sd/points", 10, &CloudSub::onCloudCb, this);
   indexer_service = _nh.advertiseService("visual_servoing/depth_cloud/indexer", &CloudSub::indexerCb, this);
   }

CloudSub::~CloudSub() {

}

//Loads point cloud from message
void CloudSub::onCloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
   pcl::fromROSMsg(*msg, cloud);
}

//Returns xyz for specified uv image coordinate
bool CloudSub::indexerCb(open_manipulator_visual_servoing::IndexPointCloud::Request& req, 
                         open_manipulator_visual_servoing::IndexPointCloud::Response& res) {
   pcl::PointXYZ p = cloud.at(req.x, req.y);
   res.point.x = p.x;
   res.point.y = p.y;
   res.point.z = p.z;
   return true;
}

//Setup node
int main(int argc, char* argv[]) {
   ros::init(argc, argv, "cloud_subscriber");
   
   ROS_INFO("Starting cloud subscriber");
   CloudSub cloudSub;
   
   ROS_INFO("Spinning...");
   ros::spin();
   return 0;
}
