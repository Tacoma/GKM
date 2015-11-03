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
// opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

#include <iostream> // Debug
//#include <sstream>

using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

PointCloudCreator::PointCloudCreator()
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
    cv_bridge::CvImagePtr cv_rgb;
    cv_bridge::CvImagePtr cv_depth;
	try
	{
        cv_rgb = cv_bridge::toCvCopy(img_rgb, sensor_msgs::image_encodings::BGR8);
        cv_depth = cv_bridge::toCvCopy(img_depth, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
  		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

    // create new message and fill it
    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = "world";
    //msg->header.stamp = ros::Time::now().toNSec();
    msg->width = cv_rgb->image.cols;
    msg->height = cv_rgb->image.rows;
    
    for(int y=0; y<cv_rgb->image.rows; y++)
    {
    	for(int x=0; x<cv_rgb->image.cols; x++)
    	{
    		int b = cv_rgb->image.at<cv::Vec3b>(y,x)[0];
    		int g = cv_rgb->image.at<cv::Vec3b>(y,x)[1];
    		int r = cv_rgb->image.at<cv::Vec3b>(y,x)[2];
    		pcl::PointXYZRGB point = pcl::PointXYZRGB(r,g,b);
    		point.x = (float)(x-cv_rgb->image.cols/2)/(float)cv_rgb->image.cols ;
    		point.y = cv_depth->image.at<float>(y,x);
    		point.z = (float)(cv_rgb->image.rows/2-y)/(float)cv_rgb->image.rows;
    		msg->points.push_back(point);
    	}
    }
    
	pc_pub_.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "PointCloudCreator");
	PointCloudCreator pcc;
	return 0;
}
