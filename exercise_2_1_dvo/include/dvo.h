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


void alignImages( Eigen::Matrix4f& transform, const cv::Mat& grayRef, const cv::Mat& depthRef, const cv::Mat& grayCur, const cv::Mat& depthCur, const Eigen::Matrix3f& cameraMatrix );

bool saveTrajectory(const std::string &filename, const tf::StampedTransform transform, const ros::Time timestamp);

#endif // DVO_H__

