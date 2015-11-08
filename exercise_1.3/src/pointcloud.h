#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

class PointCloudCreator
{
public:
	PointCloudCreator();
	~PointCloudCreator() {}
	
    void CloudCb(const sensor_msgs::ImageConstPtr& img_rgb, const sensor_msgs::ImageConstPtr& img_depth);

private:
	ros::NodeHandle nh_;
	ros::Publisher pc_pub_;
    tf::TransformListener tf_listener_;

    // camera parameter
    float fx;
    float fy;
    float cx;
    float cy;
};

#endif // POINTCLOUD_H
