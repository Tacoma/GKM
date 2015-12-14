#ifndef PC_MESH_BUILDER_H
#define PC_MESH_BUILDER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include "sophus/sim3.hpp"
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// msgs
#include <lsd_slam_msgs/keyframeMsg.h>
#include <lsd_slam_msgs/keyframeGraphMsg.h>
// tf
#include <tf/transform_broadcaster.h>

#include <vector>

// Dynamic Reconfiguration

#include "project/projectConfig.h"
#include <dynamic_reconfigure/server.h>


typedef unsigned char uchar;
typedef pcl::PointXYZRGB MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointcloud;

struct InputPointDense
{
    float idepth;
    float idepth_var;
    uchar color[4];
};


class PcMeshBuilder {
public:
    PcMeshBuilder();
    ~PcMeshBuilder();

    void processMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg);
    void processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg, MyPointcloud::Ptr cloud, Sophus::Sim3f &pose);
    MyPointcloud::Ptr findPlanes(const MyPointcloud::Ptr cloud_in, const Sophus::Sim3f &pose, unsigned int num_planes=3);
    void publishPointclouds();
    void reset();

    inline void colorPointcloud(MyPointcloud& cloud_in, Eigen::Vector3f color);

    bool showOnlyPlanarPointclouds_;
    bool showOnlyCurrent_;
    bool showOnlyColorCurrent_;

    void configCallback(project::projectConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_keyframes_;     // lsd_slam/keyframes
    ros::Subscriber sub_liveframes_;    // lsd_slam/liveframes
    ros::Publisher pub_pc_;     // maybe need a method to publish meshes for ros

    std::vector<MyPointcloud::Ptr> pointcloud_planar_vector_;
    std::vector<Sophus::Sim3f> pointcloud_pose_vector_;
    MyPointcloud::Ptr pointcloud_union_planar_;
    MyPointcloud::Ptr pointcloud_union_non_planar_;
    MyPointcloud::Ptr pointcloud_non_planar_;

    unsigned int last_frame_id_;

    // Parameters
    float scaledDepthVarTH;
    float absDepthVarTH;
    int minNearSupport;
    int sparsifyFactor;
    
    dynamic_reconfigure::Server<project::projectConfig> server_;
};

#endif // PC_MESH_BUILDER_H
