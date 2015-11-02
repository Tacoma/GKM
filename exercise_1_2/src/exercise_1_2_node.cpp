#include "exercise_1_2_node.h"


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

static const std::string OPENCV_WINDOW = "Image window";


ImageConverter::ImageConverter(int calibration)
: it_(nh_),
  calibration_(calibration),
  saved(false)
{
	// Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
		&ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/exercise_1_2/output_video", 1);


    // Create camera model
    sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo());
    info_msg->width =  640;
    info_msg->height = 480;
    float fx, fy, cx, cy, d0, d1, d2, d3, d4;
    float pfx, pfy, pcx, pcy;

    if (calibration_ == 1) {
        // Own calibration
        fx = 518.720225; fy = 517.949304; cx = 319.811982; cy = 254.756201;
        pfx = 527.474243; pfy = 530.145081; pcx = 321.418330; pcy = 252.374343;
        d0 = 0.192988; d1 = -0.389868; d2 = -0.005619; d3 = 0.003466; d4 = 0.000000;
    } else { // Default case
        // Website calibration
        fx = 517.306408; fy = 516.469215; cx = 318.643040; cy = 255.313989;
        pfx = 546.024414; pfy = 542.211182; pcx = 319.711258; pcy = 251.374926;
        d0 = 0.262383; d1 = -0.953104; d2 = -0.005358; d3 = 0.002628; d4 = 1.163314;
    }

    // Camera intrinsic
    info_msg->K[0] = fx; info_msg->K[1] =  0; info_msg->K[2] = cx;
    info_msg->K[3] =  0; info_msg->K[4] = fy; info_msg->K[5] = cy;
    info_msg->K[6] =  0; info_msg->K[7] =  0; info_msg->K[8] =  1;

    // Distortion
    info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info_msg->D.resize(5);
    info_msg->D[0] = d0; info_msg->D[1] = d1; info_msg->D[2] = d2;
    info_msg->D[3] = d3; info_msg->D[4] = d4;

    // Rectification
    info_msg->R[0] = 1; info_msg->R[1] = 0; info_msg->R[2] = 0;
    info_msg->R[3] = 0; info_msg->R[4] = 1; info_msg->R[5] = 0;
    info_msg->R[6] = 0; info_msg->R[7] = 0; info_msg->R[8] = 1;

    // Projection
    info_msg->P[0] = pfx; info_msg->P[1] =   0; info_msg->P[2]  = pcx; info_msg->P[3]  = 0;
    info_msg->P[4] =   0; info_msg->P[5] = pfy; info_msg->P[6]  = pcy; info_msg->P[7]  = 0;
    info_msg->P[8] =   0; info_msg->P[9] =   0; info_msg->P[10] =   1; info_msg->P[11] = 0;
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
    if (saved == false) {
       cv::imwrite( "input.jpg", cv_ptr->image );
    }

    if (calibration_ != 2) {
        cam_model_.rectifyImage ( cv_ptr->image, cv_ptr->image, CV_INTER_LINEAR);
    }

	// Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(3);

    if (saved == false) {
        cv::imwrite( "output.jpg", cv_ptr->image );
        saved = true;
    }

	// Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}

// entry point
int main(int argc, char** argv)
{
    ros::init(argc, argv, "exercise_1_2");

    // Argument parsing
    int i = 2;
    std::string opt; int calibration;
    if (i <= argc) { // Check that we haven't finished parsing already
        if (std::string(argv[i-1]).compare("-c") == 0) {
            opt = argv[i];
        }
    } else {
        ROS_ERROR_STREAM("Usage: -c [website|own|none]");
        ros::shutdown();
        return 0;
    }
    if (opt.compare("website") == 0) {
        calibration = 0;
    } else if (opt.compare("own") == 0) {
        calibration = 1;
    } else if (opt.compare("none") == 0) {
        calibration = 2;
    } else {
        ROS_ERROR_STREAM("Argument \"" << opt << "\" not recognized");
        ros::shutdown();
        return 0;
    }
    ROS_INFO_STREAM("Using " << opt << " calibration");

    ImageConverter ic(calibration);
    ros::spin();

	return 0;
}

