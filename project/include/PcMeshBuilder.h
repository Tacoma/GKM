#ifndef PC_MESH_BUILDER_H
#define PC_MESH_BUILDER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <project/keyframeMsg.h>
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
//#include <tf/transform_broadcaster.h>

#include <vector>

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
    MyPointcloud::Ptr processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg);
    MyPointcloud::Ptr findPlanes(const MyPointcloud::Ptr cloud_in, unsigned int num_planes=3);
    void publishPointclouds();
    void reset();

    inline void colorPointcloud(MyPointcloud& cloud_in, Eigen::Vector3f color);

    bool showOnlyPlanarPointclouds_;
    bool showOnlyCurrent_;
    bool showOnlyColorCurrent_;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_keyframes_;     // lsd_slam/keyframes
    ros::Subscriber sub_liveframes_;    // lsd_slam/liveframes
    ros::Publisher pub_pc_;     // maybe need a method to publish meshes for ros

    std::vector<MyPointcloud::Ptr> pointcloud_planar_vector_;
    MyPointcloud::Ptr pointcloud_union_planar_;
    MyPointcloud::Ptr pointcloud_union_non_planar_;

    unsigned int last_frame_id_;
};

#endif // PC_MESH_BUILDER_H
