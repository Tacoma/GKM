#include "pointcloud.h"
// pcl
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// sync
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
//#include <sensor_msgs/CameraInfo.h>

#include <iostream> // Debug

PointCloudCreator::PointCloudCreator()
{
    message_filters::Subscriber<sensor_msgs::Image> img_rgb_sub(nh_, "camera/rgb/image_color", 1);
//    message_filters::Subscriber<CameraInfo> camera_info_rbg_sub(nh_, "camera/rgb/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::Image> img_depth_sub(nh_, "camera/depth/image", 1);
//    message_filters::Subscriber camera_info_depth_sub(nh_, "camera/depth/camera_info", 1);
//    message_filters::Subscriber tf_frame_sub(nh_, "/tf", 1);

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(img_rgb_sub, img_depth_sub, 10);
    sync.registerCallback(boost::bind(&CloudCb, _1, _2));

    std::cout << "pointcloud constructor" << std::endl;
    ros::spin();
}

PointCloudCreator::~PointCloudCreator()
{

}

void PointCloudCreator::CloudCb(const sensor_msgs::ImageConstPtr& img_rgb, const sensor_msgs::ImageConstPtr& img_depth)
{
    std::cout << "callback triggered" << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "PointCloudCreator");
	PointCloudCreator pcc;
	return 0;
}
