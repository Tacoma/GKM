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
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
// Dynamic Reconfiguration
#include "te_surface_detection/generalConfig.h"
#include <dynamic_reconfigure/server.h>
//
#include <tf/transform_listener.h>
#include "sophus/sim3.hpp"
#include "plane.hpp"
#include "cylinder.hpp"

#define VISUALIZE // debug visualization

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
    void setSearchPlane(const std_msgs::Bool::ConstPtr& msg);
    void processMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg);
    void processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg, MyPointcloud::Ptr cloud);
    void processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg, MyPointcloud::Ptr cloud,
                           Eigen::Vector2i min, Eigen::Vector2i max);
    void findPlanes(MyPointcloud::Ptr cloud, unsigned int numSurfaces=1);
    void refinePlane(MyPointcloud::Ptr cloud);
    void findCylinder(MyPointcloud::Ptr cloud, unsigned int num_cylinders=1);
    void refineCylinder(MyPointcloud::Ptr cloud);
    
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
    void configCallback(te_surface_detection::generalConfig &config, uint32_t level);


// --------------- members ---------------
public:

private:
    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener subTf_;

    ros::Subscriber subKeyframes_;     // lsd_slam/keyframes
    ros::Subscriber subLiveframes_;    // lsd_slam/liveframes
    ros::Subscriber subStickToSurface_;      // gets sticking bool from joystick
    ros::Publisher pubPc_;     // maybe need a method to publish meshes for ros
    ros::Publisher pubMarkers_;
    ros::Publisher pubTf_;	// publish wall position


#ifdef VISUALIZE
	// in world frame
    MyPointcloud::Ptr pcVisSurfaces_;
    MyPointcloud::Ptr pcVisOutliers_;
#endif
	// in world frame
    MyPointcloud::Ptr pcVisDebug_;

    // Planes
    Plane::Ptr plane_;
    bool searchPlane_;
    bool planeExists_;
    Cylinder::Ptr cylinder_;

    // Parameters
    dynamic_reconfigure::Server<te_surface_detection::generalConfig> server_;
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
    lsd_slam_msgs::keyframeMsgConstPtr lastMsg_;
    unsigned int lastFrameId_;
    
    // Transforms
    Sophus::Sim3f lastPose_;
    Sophus::Sim3f currentPose_;
    Eigen::Matrix4f opticalToSensor_;
    Eigen::Affine3f sensorToMav_;
    Eigen::Affine3f mavToWorld_;
    std::string mavTFName_;
    
    int status;
};

#endif // PC_MESH_BUILDER_H
