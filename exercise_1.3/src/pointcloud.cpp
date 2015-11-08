#include "pointcloud.h"
// pcl
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// sync
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

#include <iostream> // Debug

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
typedef pcl::PointCloud<pcl::PointXYZRGB> MyPointCloud;

PointCloudCreator::PointCloudCreator() :
    fx(525.0),
    fy(525.0),
    cx(319.0),
    cy(239.5)
{
    pc_pub_ = nh_.advertise<MyPointCloud> ("pointCloud", 1);

    message_filters::Subscriber<sensor_msgs::Image> img_rgb_sub(nh_, "camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> img_depth_sub(nh_, "camera/depth/image", 1);

	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_rgb_sub, img_depth_sub);
	sync.registerCallback(boost::bind(&PointCloudCreator::CloudCb, this, _1, _2));
    std::cout << "PointCloudCreator started..." << std::endl;
    ros::spin();
}

void PointCloudCreator::CloudCb(const sensor_msgs::ImageConstPtr& img_rgb,
                                const sensor_msgs::ImageConstPtr& img_depth)
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
    MyPointCloud pc;
    pc.width = cv_rgb->image.cols;
    pc.height = cv_rgb->image.rows;
    pc.points.resize(pc.width*pc.height);


    for(int y=0; y<pc.height; y++)
    {
        for(int x=0; x<pc.width; x++)
    	{
            int idx = y*cv_rgb->image.cols + x;
            float depth = cv_depth->image.at<float>(y,x);
            cv::Vec3b colors_bgr = cv_rgb->image.at<cv::Vec3b>(y,x);

            pc.points[idx].x = (x - fx) * depth / fx;
            pc.points[idx].y = (y - cy) * depth / fy;
            pc.points[idx].z = depth;
            pc.points[idx].b = colors_bgr[0];
            pc.points[idx].g = colors_bgr[1];
            pc.points[idx].r = colors_bgr[2];
        }
    }

    try
    {
        tf_listener_.waitForTransform("/world", "/kinect", ros::Time::now(), ros::Duration(8.0f));
    }
    catch(tf::TransformException &e)
    {
        ROS_ERROR("transform exception: %s", e.what());
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pc,msg);
    msg.header.frame_id = "kinect";
    pcl_ros::transformPointCloud("/world", msg, msg, tf_listener_);
    msg.header.frame_id = "world";

	pc_pub_.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "PointCloudCreator");
    PointCloudCreator pcc;
	return 0;
}
