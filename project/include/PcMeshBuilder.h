#ifndef PC_MESH_BUILDER_H
#define PC_MESH_BUILDER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <vector>
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// msgs
#include <lsd_slam_msgs/keyframeMsg.h>
#include <lsd_slam_msgs/keyframeGraphMsg.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
// Dynamic Reconfiguration
#include "project/projectConfig.h"
#include <dynamic_reconfigure/server.h>
//
#include "sophus/sim3.hpp"
#include "plane.hpp"

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
    void removeKnownPlanes(MyPointcloud::Ptr cloud);
    void findPlanes(MyPointcloud::Ptr cloud, const Sophus::Sim3f &pose, unsigned int num_planes=3);
    void publishPointclouds();
    void reset();

    inline void colorPointcloud(MyPointcloud::Ptr cloud_in, Eigen::Vector3f color);
    inline void getProcessWindow(Eigen::Vector2i &min, Eigen::Vector2i &max, int width, int height);
    inline int clamp(int x, int min, int max);

    void configCallback(project::projectConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_keyframes_;     // lsd_slam/keyframes
    ros::Subscriber sub_liveframes_;    // lsd_slam/liveframes
    ros::Publisher pub_pc_;     // maybe need a method to publish meshes for ros
    ros::Publisher pub_markers_;
    dynamic_reconfigure::Server<project::projectConfig> server_;

    MyPointcloud::Ptr pointcloud_planar_;
    MyPointcloud::Ptr pointcloud_non_planar_;
    MyPointcloud::Ptr pointcloud_debug_;
    
    std::vector<Plane::Ptr> planes_;
    

    unsigned int last_frame_id_;
    lsd_slam_msgs::keyframeMsgConstPtr last_msg_;

    // Parameters
    float scaledDepthVarTH_;
    float absDepthVarTH_;
    int minNearSupport_;
    int sparsifyFactor_;
    float distanceThreshold_;
    int windowSize_;
    int windowPosY_;
    
    int minPointsForEstimation_;
    float noisePercentage_;
    int maxPlanesPerCloud_;
    
    
    int nextColor_;
};

#endif // PC_MESH_BUILDER_H
