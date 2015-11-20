// This source code is intended for use in the teaching course "Vision-Based Navigation" in summer term 2015 at TU Munich only. 
// Copyright 2015 Robert Maier, Joerg Stueckler, TUM

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "cv_bridge/cv_bridge.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "opencv2/opencv.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>

#include <tf/transform_broadcaster.h>

#include <dvo.h>

#include <fstream>


cv::Mat grayRef, depthRef;
ros::Publisher pub_pointcloud;
ros::Publisher pub_vis_covariance;
ros::Publisher pub_vis_covariance2;

boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_; // pointer to delay creation 
boost::shared_ptr<tf::TransformListener> tf_listener_;
tf::StampedTransform tf_integrated_transform_;
Eigen::Matrix4f integrated_transform_ = Eigen::Matrix4f::Identity();
bool isFirstIter_ = true;
std::string cam_ref_tf_name_ = "/openni_rgb_optical_frame";


void imagesToPointCloud( const cv::Mat& img_rgb, const cv::Mat& img_depth, pcl::PointCloud< pcl::PointXYZRGB >::Ptr& cloud, unsigned int downsampling = 1 ) {

  cloud->is_dense = true;
  cloud->height = img_depth.rows / downsampling;
  cloud->width = img_depth.cols / downsampling;
  cloud->sensor_origin_ = Eigen::Vector4f( 0.f, 0.f, 0.f, 1.f );
  cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
  cloud->points.resize( cloud->height*cloud->width );

  const float invfocalLength = 1.f / 525.f;
  const float centerX = 319.5f;
  const float centerY = 239.5f;
  const float depthscale = 1.f;

  const float* depthdata = reinterpret_cast<const float*>( &img_depth.data[0] );
  const unsigned char* colordata = &img_rgb.data[0];
  int idx = 0;
  for( unsigned int y = 0; y < img_depth.rows; y++ ) {
    for( unsigned int x = 0; x < img_depth.cols; x++ ) {

      if( x % downsampling != 0 || y % downsampling != 0 ) {
        colordata += 3;
        depthdata++;
        continue;
      }

      pcl::PointXYZRGB& p = cloud->points[idx];

      if( *depthdata == 0.f || isnan(*depthdata) ) { //|| factor * (float)(*depthdata) > 10.f ) {
        p.x = std::numeric_limits<float>::quiet_NaN();
        p.y = std::numeric_limits<float>::quiet_NaN();
        p.z = std::numeric_limits<float>::quiet_NaN();
      }
      else {
        float xf = x;
        float yf = y;
        float dist = depthscale * (float)(*depthdata);
        p.x = (xf-centerX) * dist * invfocalLength;
        p.y = (yf-centerY) * dist * invfocalLength;
        p.z = dist;
      }

      depthdata++;

      int b = (*colordata++);

      int g = (*colordata++);
      int r = (*colordata++);

      int rgb = ( r << 16 ) + ( g << 8 ) + b;
      p.rgb = * ( reinterpret_cast< float* > ( &rgb ) );

      idx++;
    }
  }

}



void callback(const sensor_msgs::ImageConstPtr& image_rgb, const sensor_msgs::ImageConstPtr& image_depth)
{
  
    Eigen::Matrix3f cameraMatrix;
    cameraMatrix <<    525.0, 0.0, 319.5,
                         0.0, 525.0, 239.5,
                         0.0, 0.0, 1.0;
    
    cv_bridge::CvImageConstPtr img_rgb_cv_ptr = cv_bridge::toCvShare( image_rgb, "bgr8" );
    cv_bridge::CvImageConstPtr img_depth_cv_ptr = cv_bridge::toCvShare( image_depth, "32FC1" );
    
//    cv::imshow("img_rgb", img_rgb_cv_ptr->image );
//    cv::imshow("img_depth", 0.2*img_depth_cv_ptr->image );
//    cv::waitKey(10);
    
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    
    cv::Mat grayCurInt;
    cv::cvtColor( img_rgb_cv_ptr->image.clone(), grayCurInt, CV_BGR2GRAY);
    cv::Mat grayCur;
    grayCurInt.convertTo(grayCur, CV_32FC1, 1.f/255.f);
    
    cv::Mat depthCur = img_depth_cv_ptr->image.clone();
    
    
    if( !grayRef.empty() ) {
        alignImages( transform, grayRef, depthRef, grayCur, depthCur, cameraMatrix );
//        pub_vis_covariance.publish(getMarker());
        pub_vis_covariance2.publish(getMsg());
    }

    grayRef = grayCur.clone();
    depthRef = depthCur.clone();

#if DEBUG_OUTPUT
    ROS_ERROR_STREAM( "transform: " << transform);
#endif

    // TODO: dump trajectory for evaluation / integrated_transform
    // get ground truth  in first iteration
    if (isFirstIter_) {
        isFirstIter_ = false;
        // get tf
        try{
            tf_listener_->lookupTransform("/world", cam_ref_tf_name_, ros::Time(0), tf_integrated_transform_);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        // TODO: save tf::StampedTransform tf_integrated_transform_ in Eigen::Matrix4f integrated_transform_
    }
        
    integrated_transform_ = integrated_transform_ * transform.inverse();
    
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );
    imagesToPointCloud( img_rgb_cv_ptr->image, img_depth_cv_ptr->image, cloud );
    
    cloud->header = pcl_conversions::toPCL( image_rgb->header );
    
//    cloud->header.frame_id = "/odometry";\
//    cloud->header.frame_id = cam_ref_tf_name_;
    cloud->header.frame_id = "/world";
    pcl::transformPointCloud( *cloud, *cloud, integrated_transform_ );
        
    pub_pointcloud.publish( *cloud );

    // convert and broadcast integrated_tf
    tf::Vector3 origin;
    origin.setValue(integrated_transform_(0,3), integrated_transform_(1,3), integrated_transform_(2,3));
    tf::Matrix3x3 rot;
    rot.setValue( integrated_transform_(0,0), integrated_transform_(0,1), integrated_transform_(0,2),
                  integrated_transform_(1,0), integrated_transform_(1,1), integrated_transform_(1,2),
                  integrated_transform_(2,0), integrated_transform_(2,1), integrated_transform_(2,2));

    tf::Quaternion tfqt;
    rot.getRotation(tfqt);

    tf_integrated_transform_.setOrigin(origin);
    tf_integrated_transform_.setRotation(tfqt);

    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_integrated_transform_, ros::Time::now(), "world", "odometry"));

    saveTrajectory("test.txt", tf_integrated_transform_, image_depth->header.stamp.toSec());    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sheet2_dvo_node");

  ros::NodeHandle nh("~");
  tf_broadcaster_.reset(new tf::TransformBroadcaster());
  tf_listener_.reset( new tf::TransformListener());
  
  message_filters::Subscriber<sensor_msgs::Image> image_rgb_sub(nh, "image_rgb", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(nh, "image_depth", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_rgb_sub, image_depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  pub_pointcloud = nh.advertise< pcl::PointCloud< pcl::PointXYZRGB > >( "pointcloud", 1 );
  pub_vis_covariance = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0 );
  pub_vis_covariance2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("covariance_marker", 0 );

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
  
  return 0;
}




