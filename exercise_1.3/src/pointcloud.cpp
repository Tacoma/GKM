#include "pointcloud.h"
// pcl
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// sync
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream> // Debug

using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloudCreator::PointCloudCreator()
{
	pc_pub_ = nh_.advertise<PointCloud> ("pointCloud", 1);

    message_filters::Subscriber<Image> img_rgb_sub(nh_, "camera/rgb/image_color", 1);
    message_filters::Subscriber<Image> img_depth_sub(nh_, "camera/depth/image", 1);
//    message_filters::Subscriber tf_frame_sub(nh_, "/tf", 1);

	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_rgb_sub, img_depth_sub);
	sync.registerCallback(boost::bind(&PointCloudCreator::CloudCb, this, _1, _2));

    std::cout << "pointcloud constructor" << std::endl;
    ros::spin();
}

void PointCloudCreator::CloudCb(const sensor_msgs::ImageConstPtr& img_rgb, const sensor_msgs::ImageConstPtr& img_depth)
{
    std::cout << "callback triggered" << std::endl;
    
    PointCloud::Ptr msg (new PointCloud);
	msg->header.frame_id = "some_tf_frame"; // TODO
	msg->header.stamp = ros::Time::now().toNSec();
	msg->height = msg->width = 1;
	msg->points.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
	
	pc_pub_.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "PointCloudCreator");
	PointCloudCreator pcc;
	return 0;
}
