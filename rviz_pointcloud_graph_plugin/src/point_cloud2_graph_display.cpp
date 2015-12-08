/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/time.h>
// rviz
#include "rviz/default_plugin/point_cloud_transformers.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/int_property.h"
#include "rviz/validate_floats.h"
//pcl
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "sophus/sim3.hpp"
#include "point_cloud2_graph_display.h"


const Eigen::Vector3f debugColors[] = { Eigen::Vector3f(128,0,0),
                                        Eigen::Vector3f(0,128,0),
                                        Eigen::Vector3f(0,0,128),
                                        Eigen::Vector3f(128,128,0),
                                        Eigen::Vector3f(128,0,128),
                                        Eigen::Vector3f(0,128,128),
                                        Eigen::Vector3f(128,128,128)};

namespace rviz_cloud2_graph_display
{

PointCloud2GraphDisplay::PointCloud2GraphDisplay() {
    pc_topic_property_ =
        new rviz::RosTopicProperty("Pointcloud Topic", "",
                                   QString::fromStdString("lsd_slam_viewer/keyframeMsg"),
                                   "lsd_slam/keyframe topic to subscribe to", this,
                                   SLOT(updateTopic()));

    oculus_cam_follow_topic_property_ =
        new rviz::RosTopicProperty("Follow camera topic", "",
                                   QString::fromStdString("lsd_slam_viewer/keyframeMsg"),
                                   "lsd_slam/liveframes topic to in order to publish the camera position as tf-frame \"camera\".", this,
                                   SLOT(updateTopic()));

    graph_topic_property_ =
        new rviz::RosTopicProperty("Graph Topic", "",
                                   QString::fromStdString("lsd_slam_viewer/keyframeGraphMsg"),
                                   "lsd_slam/graph topic to subscribe to", this,
                                   SLOT(updateTopic()));
}

PointCloud2GraphDisplay::~PointCloud2GraphDisplay() {
    point_cloud_common_vector_.clear();
}

void PointCloud2GraphDisplay::onInitialize() {
    Display::onInitialize(); // needed?
}

void PointCloud2GraphDisplay::fixedFrameChanged() {
    // TODO
}

void PointCloud2GraphDisplay::reset()
{
    Display::reset();
    point_cloud_common_vector_.clear();
}

void PointCloud2GraphDisplay::onEnable() {
    subscribe();
}

void PointCloud2GraphDisplay::onDisable() {
    unsubscribe();
}

void PointCloud2GraphDisplay::updateTopic() {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender(); // request new render
}

void PointCloud2GraphDisplay::subscribe() {
    if (!isEnabled()) {
        return;
    }
    try {
        std::string pc_topic = pc_topic_property_->getTopicStd();
        if (!pc_topic.empty()) {
            liveframes_sub_ = threaded_nh_.subscribe(pc_topic, 1, &PointCloud2GraphDisplay::processMessage, this);
        }

        std::string oculus_cam_topic = oculus_cam_follow_topic_property_->getTopicStd();
        if (!oculus_cam_topic.empty()) {
            oculus_cam_sub_ = threaded_nh_.subscribe(oculus_cam_topic, 1, &PointCloud2GraphDisplay::processMessage, this);
        }

        std::string graph_topic = graph_topic_property_->getTopicStd();
        if (!graph_topic.empty()) {
            graph_sub_ = threaded_nh_.subscribe(graph_topic, 1, &PointCloud2GraphDisplay::processGraphMessage, this);
        }
    } catch (ros::Exception& e) {
        setStatus(rviz::StatusProperty::Error, "Message",
                  QString("Error subscribing: ") + e.what());
    }
}

void PointCloud2GraphDisplay::unsubscribe() {
    try {
        liveframes_sub_.shutdown();
        oculus_cam_sub_.shutdown();
        graph_sub_.shutdown();
    } catch (ros::Exception& e) {
        setStatus(rviz::StatusProperty::Error, "Message",
                  QString("Error unsubscribing: ") + e.what());
    }
}

void PointCloud2GraphDisplay::processMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg) {

    if (context_->getFrameManager()->getPause()) {
        return;
    }

    if (msg->isKeyframe) {
        // add message to queue
        MyPointcloud::Ptr cloud = processPointcloud(msg);
        cloud = findPlanes(cloud);
        if(cloud) {
            createCloudVisual(cloud);
        }
    } else {
        // check for reset
        if (last_frame_id > msg->id) {
            ROS_INFO_STREAM("detected backward-jump in id (" << last_frame_id << " to " << msg->id << "), resetting!");
            reset();
        }
        last_frame_id = msg->id;
    }
}

MyPointcloud::Ptr PointCloud2GraphDisplay::processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg) {

    Sophus::Sim3f camToWorld_;
    memcpy(camToWorld_.data(), msg->camToWorld.data(), 7*sizeof(float));

    float fx_ = msg->fx;
    float fy_ = msg->fy;
    float cx_ = msg->cx;
    float cy_ = msg->cy;

    float fxi_ = 1.0/fx_;
    float fyi_ = 1.0/fy_;
    float cxi_ = -cx_ / fx_;
    float cyi_ = -cy_ / fy_;

    int width_  = msg->width;
    int height_ = msg->height;
//    unsigned int id_ = msg->id;
//    double time_ = msg->time;

    InputPointDense* originalInput_ = 0;

    if(msg->pointcloud.size() != width_*height_*sizeof(InputPointDense)) {
        if(msg->pointcloud.size() != 0)
            ROS_WARN_STREAM("PC with points, but number of points not right!");
    } else {
        originalInput_ = new InputPointDense[width_*height_];
        memcpy(originalInput_, msg->pointcloud.data(), width_*height_*sizeof(InputPointDense));
    }

    if(originalInput_ == 0) {
        ROS_ERROR_STREAM("originalInput_ was null");
        return 0;
    }

    // TODO parameterize
    const float scaledDepthVarTH = pow(10.0f,-3.0f);
    const float absDepthVarTH = pow(10.0f, 1.0f);
    const int minNearSupport = 7;
    const int sparsifyFactor = 1;

    float worldScale = camToWorld_.scale();

    MyPointcloud::Ptr pc = boost::make_shared<MyPointcloud>();
    pc->resize(width_*height_);
    int numPoints = 0;
    for(int y=1; y<height_-1; y++) {
        for(int x=1; x<width_-1; x++) {
            if(originalInput_[x+y*width_].idepth <= 0)
                continue;

            if(sparsifyFactor > 1 && rand()%sparsifyFactor != 0)
                continue;

            float depth = 1 / originalInput_[x+y*width_].idepth;
            float depth4 = depth*depth;
            depth4*= depth4;

            if(originalInput_[x+y*width_].idepth_var * depth4 > scaledDepthVarTH)
                continue;

            if(originalInput_[x+y*width_].idepth_var * depth4 * worldScale*worldScale > absDepthVarTH)
                continue;

            if(minNearSupport > 1) {
                int nearSupport = 0;
                for(int dx=-1; dx<2; dx++) {
                    for(int dy=-1; dy<2; dy++) {
                        int idx = x+dx+(y+dy)*width_;
                        if(originalInput_[idx].idepth > 0) {
                            float diff = originalInput_[idx].idepth - 1.0f / depth;
                            if(diff*diff < 2*originalInput_[x+y*width_].idepth_var)
                                nearSupport++;
                        }
                    }
                }

                if(nearSupport < minNearSupport)
                    continue;
            }

            MyPoint point;
            point.x = (x*fxi_ + cxi_) * depth;
            point.y = (y*fyi_ + cyi_) * depth;
            point.z = depth;
            point.r = originalInput_[x+y*width_].color[2];
            point.g = originalInput_[x+y*width_].color[1];
            point.b = originalInput_[x+y*width_].color[0];
            pc->points[numPoints] = point;

            numPoints++;
        }
    }
    // refit pointcloud and search for planes
    pc->resize(numPoints);
    pcl::transformPointCloud(*pc,*pc,camToWorld_.matrix());
//    findPlanes(pc);

    delete[] originalInput_;

    return pc;
}

void PointCloud2GraphDisplay::createCloudVisual(MyPointcloud::Ptr cloud) {
    sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud, *msg);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "world";

    rviz::PointCloudCommon* point_cloud_common_instance = new rviz::PointCloudCommon(this);
    point_cloud_common_instance->initialize(context_, scene_node_);

    point_cloud_common_instance->addMessage(msg);
    point_cloud_common_vector_.push_back(point_cloud_common_instance);
}

MyPointcloud::Ptr PointCloud2GraphDisplay::findPlanes(MyPointcloud::Ptr cloud_in) {
    MyPointcloud::Ptr union_cloud(new MyPointcloud);

    MyPointcloud::Ptr cropped_cloud(new MyPointcloud(*cloud_in));
    MyPointcloud::Ptr cloud_f(new MyPointcloud);
    MyPointcloud::Ptr cloud_filtered(new MyPointcloud);
    MyPointcloud::Ptr cloud_plane(new MyPointcloud);

    // create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<MyPoint> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);

    // extract the planar inliers from the input cloud
    pcl::ExtractIndices<MyPoint> extract;
    int i=0;
    int nr_points = (int)cropped_cloud->points.size();
    while (cropped_cloud->points.size() > 0.3*nr_points) {
        // segment the largest planar component from the cropped cloud
        seg.setInputCloud(cropped_cloud);
        seg.segment(*inliers, *coefficients);
        if(inliers->indices.size () == 0) {
            ROS_WARN_STREAM ("Could not estimate a planar model for the given pointcloud data");
            break;
        }

        // extract the inliers
        extract.setInputCloud(cropped_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        // get the points associated with the planar surface
        extract.filter(*cloud_plane);

        // recolor the extracted pointcloud for debug visualization
        colorPointcloud(*cloud_plane, debugColors[i%(sizeof(debugColors)/sizeof(Eigen::Vector3f))]);
        // add newly created pointcloud in the union pointcloud
        *union_cloud += *cloud_plane;

        // create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
        cropped_cloud.swap(cloud_filtered);

        i++;
    }
    // add non planar point clouds to the union pointcloud with a white color
    *union_cloud += *cloud_filtered;

    return union_cloud;
}

inline void PointCloud2GraphDisplay::colorPointcloud(MyPointcloud& cloud_in, Eigen::Vector3f color) {
    for(int i=0; i<cloud_in.points.size(); i++) {
        cloud_in.points[i].r = color.x();
        cloud_in.points[i].g = color.y();
        cloud_in.points[i].b = color.z();
    }
}

void PointCloud2GraphDisplay::processGraphMessage(const lsd_slam_msgs::keyframeGraphMsgConstPtr msg) {
    // TODO
}

void PointCloud2GraphDisplay::update( float wall_dt, float ros_dt )
{
    for ( auto& point_cloud_common : point_cloud_common_vector_) {
        point_cloud_common->update( wall_dt, ros_dt );
    }
}

} // namespace rviz_cloud2_graph_display

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_cloud2_graph_display::PointCloud2GraphDisplay, rviz::Display )
