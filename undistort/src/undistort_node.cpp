#include "undistort_node.h"


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";


ImageConverter::ImageConverter()
: it_(nh_)
{
	// Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
		&ImageConverter::imageCb, this);
	image_pub_ = it_.advertise("/image_converter/output_video", 1);

    // Create camera model
    sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo());
    info_msg->width = 640;
    info_msg->height = 480;
    // 	fx      fy      cx      cy      d0      d1      d2      d3      d4
    // 517.3 	516.5 	318.6 	255.3 	0.2624	-0.9531	-0.0054	0.0026 	1.1633
    info_msg->K[0] = 517.3; info_msg->K[1] =     0; info_msg->K[2] = 318.6;
    info_msg->K[3] =     0; info_msg->K[4] = 516.5; info_msg->K[5] = 255.3;
    info_msg->K[6] =     0; info_msg->K[7] =     0; info_msg->K[8] = 1;

    info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info_msg->D.resize(5);
    info_msg->D[0] = 0.2624;
    info_msg->D[1] = -0.9531;
    info_msg->D[2] = -0.0054;
    info_msg->D[3] = 0.0026;
    info_msg->D[4] = 1.1633;

    info_msg->R[0] = 1; info_msg->R[1] = 0; info_msg->R[2] = 0;
    info_msg->R[3] = 0; info_msg->R[4] = 1; info_msg->R[5] = 0;
    info_msg->R[6] = 0; info_msg->R[7] = 0; info_msg->R[8] = 1;

    info_msg->P[0] = info_msg->K[0];  info_msg->P[1] = 0;              info_msg->P[2]  = info_msg->K[2];  info_msg->P[3]  = 0;
    info_msg->P[4] = 0;              info_msg->P[5] = info_msg->K[4];  info_msg->P[6]  = info_msg->K[5];  info_msg->P[7]  = 0;
    info_msg->P[8] = 0;              info_msg->P[9] = 0;              info_msg->P[10] = 1;              info_msg->P[11] = 0;
    cam_model_.fromCameraInfo(info_msg);

	cv::namedWindow(OPENCV_WINDOW);
}

ImageConverter::~ImageConverter()
{
	cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
	try
	{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
  		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

    cam_model_.rectifyImage ( cv_ptr->image, cv_ptr->image, CV_INTER_LINEAR);

	// Update GUI Window
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(3);

	// Output modified video stream
	image_pub_.publish(cv_ptr->toImageMsg());
}

// entry point
int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}

