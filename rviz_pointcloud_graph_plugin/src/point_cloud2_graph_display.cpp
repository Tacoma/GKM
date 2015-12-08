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

namespace rviz_cloud2_graph_display
{

PointCloud2GraphDisplay::PointCloud2GraphDisplay()
    : point_cloud_common_template_( new rviz::PointCloudCommon( this )) {
    pc_topic_property_ =
        new rviz::RosTopicProperty("Pointcloud Topic", "",
                                   QString::fromStdString("lsd_slam_viewer/keyframeMsg"),
                                   "lsd_slam/keyframe topic to subscribe to", this,
                                   SLOT(updateTopic()));

    oculus_cam_follow_topic_property_ =
        new rviz::RosTopicProperty("Oculus camera topic", "",
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
    delete point_cloud_common_template_;
    point_cloud_common_vector_.clear();
}

void PointCloud2GraphDisplay::onInitialize() {
    Display::onInitialize(); // needed?
    point_cloud_common_template_->initialize( context_, scene_node_ );
}

void PointCloud2GraphDisplay::fixedFrameChanged() {
    // TODO
}

void PointCloud2GraphDisplay::reset()
{
    Display::reset();
    point_cloud_common_vector_.clear();
//    point_cloud_common_->reset();
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
            //TODO: use boost version of callback
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

/*
void PointCloud2GraphDisplay::processMessage( const lsd_slam_msgs::keyframeMsgConstPtr& cloud ) {
     // Filter any nan values out of the cloud.  Any nan values that make it through to PointCloudBase
     // will get their points put off in lala land, but it means they still do get processed/rendered
     // which can be a big performance hit
     sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);
     int32_t xi = findChannelIndex(cloud, "x");
     int32_t yi = findChannelIndex(cloud, "y");
     int32_t zi = findChannelIndex(cloud, "z");

     if (xi == -1 || yi == -1 || zi == -1)
     {
         return;
     }

     const uint32_t xoff = cloud->fields[xi].offset;
     const uint32_t yoff = cloud->fields[yi].offset;
     const uint32_t zoff = cloud->fields[zi].offset;
     const uint32_t point_step = cloud->point_step;
     const size_t point_count = cloud->width * cloud->height;

     if( point_count * point_step != cloud->data.size() )
     {
         std::stringstream ss;
         ss << "Data size (" << cloud->data.size() << " bytes) does not match width (" << cloud->width
            << ") times height (" << cloud->height << ") times point_step (" << point_step << ").  Dropping message.";
         setStatusStd( StatusProperty::Error, "Message", ss.str() );
         return;
     }

     filtered->data.resize(cloud->data.size());
     if (point_count == 0)
     {
         return;
     }

     uint8_t* output_ptr = &filtered->data.front();
     const uint8_t* ptr = &cloud->data.front(), *ptr_end = &cloud->data.back(), *ptr_init;
     size_t points_to_copy = 0;
     for (; ptr < ptr_end; ptr += point_step)
     {
         float x = *reinterpret_cast<const float*>(ptr + xoff);
         float y = *reinterpret_cast<const float*>(ptr + yoff);
         float z = *reinterpret_cast<const float*>(ptr + zoff);
         if (validateFloats(x) && validateFloats(y) && validateFloats(z))
         {
             if (points_to_copy == 0)
             {
                 // Only memorize where to start copying from
                 ptr_init = ptr;
                 points_to_copy = 1;
             }
             else
             {
                 ++points_to_copy;
             }
         }
         else
         {
             if (points_to_copy)
             {
                 // Copy all the points that need to be copied
                 memcpy(output_ptr, ptr_init, point_step*points_to_copy);
                 output_ptr += point_step*points_to_copy;
                 points_to_copy = 0;
             }
         }
     }
     // Don't forget to flush what needs to be copied
     if (points_to_copy)
     {
         memcpy(output_ptr, ptr_init, point_step*points_to_copy);
         output_ptr += point_step*points_to_copy;
     }
     uint32_t output_count = (output_ptr - &filtered->data.front()) / point_step;

     filtered->header = cloud->header;
     filtered->fields = cloud->fields;
     filtered->data.resize(output_count * point_step);
     filtered->height = 1;
     filtered->width = output_count;
     filtered->is_bigendian = cloud->is_bigendian;
     filtered->point_step = point_step;
     filtered->row_step = output_count;


     rviz::PointCloudCommon* point_cloud_common_instance = new rviz::PointCloudCommon( this );
     point_cloud_common_instance->initialize( context_, scene_node_);

     point_cloud_common_instance->addMessage( filtered );
     point_cloud_common_vector_.push_back(point_cloud_common_instance);
}
*/

void PointCloud2GraphDisplay::processMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg) {

    if (context_->getFrameManager()->getPause()) {
        return;
    }

    if (msg->isKeyframe) {
        // add message to queue
        MyPointcloud::Ptr cloud = processPointcloud(msg);
        createCloudVisual(cloud);
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
    unsigned int id_ = msg->id;
    double time_ = msg->time;

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

    MyPointcloud::Ptr pc;
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
//    pcl::transformPointCloud(*pc,*pc,camToWorld_.matrix());
//    findPlanes(pc);

    delete[] originalInput_;

    return pc;
}

void PointCloud2GraphDisplay::createCloudVisual(MyPointcloud::Ptr cloud) {
    sensor_msgs::PointCloud2::Ptr msg;
    pcl::toROSMsg(*cloud, *msg);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "world";

    rviz::PointCloudCommon* point_cloud_common_instance = new rviz::PointCloudCommon(this);
    point_cloud_common_instance->initialize(context_, scene_node_);

    point_cloud_common_instance->addMessage(msg);
    point_cloud_common_vector_.push_back(point_cloud_common_instance);
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
