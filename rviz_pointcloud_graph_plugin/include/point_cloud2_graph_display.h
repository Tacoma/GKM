/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_POINT_CLOUD2_DISPLAY_H
#define RVIZ_POINT_CLOUD2_DISPLAY_H

#include <sensor_msgs/PointCloud2.h>

// rivz
#include "rviz/message_filter_display.h"
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/quaternion_property.h>
#include "rviz/default_plugin/point_cloud_common.h"
// msgs
#include <lsd_slam_msgs/keyframeMsg.h>
#include <lsd_slam_msgs/keyframeGraphMsg.h>
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef unsigned char uchar;
typedef pcl::PointXYZRGB MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointcloud;

struct InputPointDense
{
    float idepth;
    float idepth_var;
    uchar color[4];
};


namespace rviz_cloud2_graph_display
{
/**
 * \class PointCloud2GraphDisplay
 * \brief Displays a point cloud of type sensor_msgs::PointCloud2
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */
class PointCloud2GraphDisplay: public rviz::Display
{
Q_OBJECT
public:
    PointCloud2GraphDisplay();
    ~PointCloud2GraphDisplay();

    virtual void update( float wall_dt, float ros_dt );

private Q_SLOTS:
    virtual void updateTopic();

protected:
    virtual void onInitialize();
    virtual void reset();
    virtual void fixedFrameChanged();
    void onEnable();
    void onDisable();

private:
    void processMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg);
    void processGraphMessage(const lsd_slam_msgs::keyframeGraphMsgConstPtr msg);

    void subscribe();
    void unsubscribe();

    MyPointcloud::Ptr processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg);
    void createCloudVisual(MyPointcloud::Ptr cloud);
    MyPointcloud::Ptr findPlanes(MyPointcloud::Ptr cloud_in);

    inline void colorPointcloud(MyPointcloud& cloud_in, Eigen::Vector3f color);

    // ROS image subscription & synchronization
    ros::Subscriber liveframes_sub_; // keyframes messages
    ros::Subscriber graph_sub_;      // graph messages
    ros::Subscriber oculus_cam_sub_; // live frame messages

    // ROS properties
    rviz::IntProperty*        depth_subsample_property_;
    rviz::RosTopicProperty*   pc_topic_property_;
    rviz::RosTopicProperty*   graph_topic_property_;
    rviz::RosTopicProperty*   oculus_cam_follow_topic_property_;
    rviz::BoolProperty*       cam_marker_visible_property_;
    rviz::FloatProperty*      scaledDepthVarTH_property_;
    rviz::IntProperty*        minNearSupp_property_;
    rviz::QuaternionProperty* orientation_property_;
    rviz::BoolProperty*       button_property_;
    rviz::BoolProperty*       delete_original_msgs_property_;

    std::vector<rviz::PointCloudCommon*> point_cloud_common_vector_;
    std::vector<MyPointcloud::Ptr> clouds_vector_;

    int last_frame_id;
};

} // namespace rviz_cloud2_graph_display

#endif // RVIZ_POINT_CLOUD2_DISPLAY_H
