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
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
// Dynamic Reconfiguration
#include "project/projectConfig.h"
#include <dynamic_reconfigure/server.h>
//
#include "sophus/sim3.hpp"
#include "plane.hpp"

//#define VISUALIZE // debug visualization

struct InputPointDense
{
    float idepth;
    float idepth_var;
    uchar color[4];
};

// ------------------------- SurfaceDetection -------------------------
class SurfaceDetection {
// --------------- functions ---------------
public:
    SurfaceDetection();
    ~SurfaceDetection();

private:
    void setStickToSurface(const std_msgs::Bool::ConstPtr& msg);
    void processMessageStickToSurface(const lsd_slam_msgs::keyframeMsgConstPtr msg);
    void processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg, MyPointcloud::Ptr cloud);
    void processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg, MyPointcloud::Ptr cloud,
                           Eigen::Vector2i min, Eigen::Vector2i max);
    void refinePlane(MyPointcloud::Ptr cloud);
    void findPlanes(MyPointcloud::Ptr cloud, unsigned int num_planes=3);
    void publishPointclouds();
    void publishPolygons();
    void publishPlane();
    void reset();

    inline void colorPointcloud(MyPointcloud::Ptr cloud_in, Eigen::Vector3f color);
    inline void getProcessWindow(Eigen::Vector2i &min_out, Eigen::Vector2i &max_out, 
				 float windowSize, float windowOffset, 
				 int width, int height);
    inline void clampProcessWindow(Eigen::Vector2i &min_inout, Eigen::Vector2i &max_inout, int width, int height);
    inline int clamp(int x, int min, int max);

    void configCallback(project::projectConfig &config, uint32_t level);


// --------------- members ---------------
public:

private:
    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_keyframes_;     // lsd_slam/keyframes
    ros::Subscriber sub_liveframes_;    // lsd_slam/liveframes
    ros::Subscriber sub_stickToSurface_;      // gets sticking bool from joystick
    ros::Publisher pub_pc_;     // maybe need a method to publish meshes for ros
    ros::Publisher pub_markers_;
    ros::Publisher pub_tf_;	// publish wall position


#ifdef VISUALIZE
    MyPointcloud::Ptr pointcloud_planar_;
    MyPointcloud::Ptr pointcloud_non_planar_;
#endif
    MyPointcloud::Ptr pointcloud_debug_;

    // Planes
    SimplePlane::Ptr plane_;
    bool searchPlane_;
    bool planeExists_;

    // Parameters
    dynamic_reconfigure::Server<project::projectConfig> server_;
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

    // Stuff
    int nextColor_;
    lsd_slam_msgs::keyframeMsgConstPtr last_msg_;
    unsigned int last_frame_id_;
    Sophus::Sim3f last_pose_;
    Sophus::Sim3f current_pose_;
    Eigen::Matrix4f opticalToSensor_;
    Eigen::Affine3f sensorToMav_;
    std::string mavTFName_;
    
    int status;
};

#endif // PC_MESH_BUILDER_H
