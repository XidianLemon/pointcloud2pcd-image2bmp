#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"

#include <cv_bridge/cv_bridge.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

ros::Publisher cloud_pub;

cv::Mat rect_view, map1, map2;
bool map_initialised = false;

Eigen::Matrix4d sensorTransform;
Eigen::Matrix3d tmat;

ros::Publisher rimg_pub;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());

void cameraCallback(const sensor_msgs::Image::ConstPtr& img)
{
	
  double timeimg = img->header.stamp.toSec();

  char str[256];
  sprintf(str, "%lf", timeimg);

  std::string s_timescan = str;

  cv_bridge::CvImagePtr cv_cam = cv_bridge::toCvCopy(img, "8UC3");

  imwrite("some where"+s_timescan+".bmp", cv_cam->image);

	ROS_INFO("imagewancheng!");
}

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& scan)
{
	
  double timescan = scan->header.stamp.toSec();
	
  char str[256];
  sprintf(str, "%lf", timescan);

  std::string s_timescan = str;
	
  //std::string pcd_name(s_timescan);
	
  pcl::PointCloud<pcl::PointXYZ> cloud;  
  pcl::fromROSMsg(*scan, cloud);
  pcl::io::savePCDFileASCII ("somewhere"+s_timescan+".pcd", cloud);
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "scan_archiver");

 ros::NodeHandle n;

 ros::Subscriber velodyne = n.subscribe("/yourlidartopic", 2, lidarCallback);
	
 ros::Subscriber sub1 = n.subscribe("/yourcameratopic", 2, cameraCallback);

 ros::spin();

 return 0;
}

