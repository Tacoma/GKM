#ifndef EXERCISE_1_2_NODE_H
#define EXERSICE_1_2_NODE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>


class ImageConverter
{
	ros::NodeHandle nh_;
    bool saved;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher  image_pub_;
	image_geometry::PinholeCameraModel cam_model_;
	int calibration_;
	
public:
    ImageConverter(int calibration);
	~ImageConverter();
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif // EXERSICE_1_2_NODE_H
