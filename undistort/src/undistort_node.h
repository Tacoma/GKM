#ifndef UNDISTORT_NODE_H
#define UNDISTORT_NODE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>


class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher  image_pub_;
    image_geometry::PinholeCameraModel cam_model_;
	
public:
    ImageConverter(int calibration);
	~ImageConverter();
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif // UNDISTORT_NODE_H
