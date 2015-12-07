#ifndef PC_MESH_BUILDER_H
#define PC_MESH_BUILDER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <project/keyframeMsg.h>
#include "sophus/sim3.hpp"
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef unsigned char uchar;
typedef pcl::PointXYZRGB MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

struct MyVertex
{
    float point[3];
    uchar color[4];
};
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

    void setFrom(project::keyframeMsgConstPtr msg);
    void refreshPC();
    void findPlanes(const MyPointCloud& cloud_in);
    void initMarker();
    inline void colorPC(MyPointCloud& cloud_in, Eigen::Vector3f color);

    Sophus::Sim3f camToWorld_;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_pc_;    // lsd_slam/keyframes
    ros::Publisher pub_pc_;     // maybe need a method to publish meshes for ros

    float fx_,fy_,cx_,cy_;
    float fxi_,fyi_,cxi_,cyi_;
    int width_;
    int height_;
    unsigned int id_;
    double time_;

    InputPointDense* originalInput_;
};

#endif // PC_MESH_BUILDER_H
