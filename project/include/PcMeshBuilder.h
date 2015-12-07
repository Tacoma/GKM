#ifndef PC_MESH_BUILDER_H
#define PC_MESH_BUILDER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <project/keyframeMsg.h>
#include "sophus/sim3.hpp"

typedef unsigned char uchar;

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

    Sophus::Sim3f camToWorld_;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_pc_;
    ros::Publisher pub_pc_; // maybe need a method to publish meshes for ros

    float fx_,fy_,cx_,cy_;
    float fxi_,fyi_,cxi_,cyi_;
    int width_;
    int height_;
    unsigned int id_;
    double time_;

    InputPointDense* originalInput_;
};

#endif // PC_MESH_BUILDER_H
