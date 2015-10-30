#ifndef UNDISTORT_NODE_H
#define UNDISTORT_NODE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>


class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher  image_pub_;
	
public:
	ImageConverter();
	~ImageConverter();
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif // UNDISTORT_NODE_H
