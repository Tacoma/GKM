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
//#include <sstream>

using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloudCreator::PointCloudCreator() :
    counter_(0)
{
	pc_pub_ = nh_.advertise<PointCloud> ("pointCloud", 1);

    message_filters::Subscriber<Image> img_rgb_sub(nh_, "camera/rgb/image_color", 1);
    message_filters::Subscriber<Image> img_depth_sub(nh_, "camera/depth/image", 1);
//    message_filters::Subscriber tf_frame_sub(nh_, "/tf", 1);

	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_rgb_sub, img_depth_sub);
	sync.registerCallback(boost::bind(&PointCloudCreator::CloudCb, this, _1, _2));

    std::cout << "PointCloudCreator started..." << std::endl;
    ros::spin();
}

// TODO: tf of pc and pc point color
void PointCloudCreator::CloudCb(const sensor_msgs::ImageConstPtr& img_rgb, const sensor_msgs::ImageConstPtr& img_depth)
{
    //std::cout << "callback triggered" << std::endl;

    // create new message and fill it
    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = "world";
    //msg->header.stamp = ros::Time::now().toNSec();
    msg->width = 1;
    msg->height = 10;

    pcl::PointXYZ points[10];
    points[0] = pcl::PointXYZ(0,0,0);
    points[1] = pcl::PointXYZ(1,0,0);
    points[2] = pcl::PointXYZ(0,1,0);
    points[3] = pcl::PointXYZ(0,0,1);
    points[4] = pcl::PointXYZ(1,1,0);
    points[5] = pcl::PointXYZ(1,0,1);
    points[6] = pcl::PointXYZ(1,1,1);
    points[7] = pcl::PointXYZ(2,0,0);
    points[8] = pcl::PointXYZ(0,2,0);
    points[9] = pcl::PointXYZ(0,0,2);

    for(int i=0; i<10; i++)
    {
        msg->points.push_back(points[i]);
    }

	pc_pub_.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "PointCloudCreator");
	PointCloudCreator pcc;
	return 0;
}
