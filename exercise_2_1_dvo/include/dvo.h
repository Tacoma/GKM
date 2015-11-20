// This source code is intended for use in the teaching course "Vision-Based Navigation" in summer term 2015 at Technical University Munich only. 
// Copyright 2015 Robert Maier, Joerg Stueckler, Technical University Munich

#ifndef DVO_H__
#define DVO_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <ctime>

#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

void alignImages( Eigen::Matrix4f& transform, const cv::Mat& grayRef, const cv::Mat& depthRef, const cv::Mat& grayCur, const cv::Mat& depthCur, const Eigen::Matrix3f& cameraMatrix );

bool saveTrajectory(const std::string &filename, const tf::StampedTransform transform, const float timestamp);

visualization_msgs::Marker getMarker();
geometry_msgs::PoseWithCovarianceStamped getMsg();

#endif // DVO_H__

