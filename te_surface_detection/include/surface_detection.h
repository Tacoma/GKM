#ifndef PC_MESH_BUILDER_H
#define PC_MESH_BUILDER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <vector>
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// msgs
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
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
    void update();
    void setSearchPlane(const std_msgs::Bool::ConstPtr& msg);
    void setSurfaceType(const std_msgs::Int32::ConstPtr& msg);
    void processMessage(const pcl::PointCloud<MyPoint>::ConstPtr& msg);
    void findPlanes(boost::shared_ptr<MyPointcloud> cloud, unsigned int numSurfaces=1);
    void refinePlane(boost::shared_ptr<MyPointcloud> cloud);
    void findCylinder(boost::shared_ptr<MyPointcloud> cloud, unsigned int num_cylinders=1);
    void refineCylinder(boost::shared_ptr<MyPointcloud> cloud);
    
    void publishPointclouds();
    void publishPolygons();
    void publishPlane();
    void publishCylinder();
    void reset();

    void updateCamToWorld();
    void updateMavToWorld();
    inline void colorPointcloud(boost::shared_ptr<MyPointcloud> cloud_in, Eigen::Vector3f color);
    inline void clampProcessWindow(Eigen::Vector2i &min_inout, Eigen::Vector2i &max_inout, int width, int height);
    inline int clamp(int x, int min, int max);


// --------------- members ---------------
public:

private:
    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener subTf_;

    ros::Subscriber subLiveframes_;         // lsd_slam/liveframes
    ros::Subscriber subStickToSurface_;     // gets sticking bool from joystick
    ros::Subscriber subSurfaceType_;        // gets surface type from joystick
    ros::Publisher pubPc_;                  // debug pointclouds
    ros::Publisher pubTf_;                  // publish surface position
    ros::Publisher pubCylinder_;            // publish surface marker

    boost::shared_ptr<MyPointcloud> pcLastCloud_;
#ifdef VISUALIZE
	// in world frame
    boost::shared_ptr<MyPointcloud> pcVisSurfaces_;
    boost::shared_ptr<MyPointcloud> pcVisOutliers_;
#endif
	// in world frame
    boost::shared_ptr<MyPointcloud> pcVisDebug_;

    // Planes
    int surfaceType_; // 1 = plane, 2 = cylinder
    bool searchSurface_;
    bool surfaceExists_;
    Plane::Ptr plane_;
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
    unsigned int lastFrameId_;
    
    // Transforms
    Eigen::Matrix4f lastPose_;
    Eigen::Matrix4f currentPose_;
    Eigen::Matrix4f opticalToSensor_;
    Eigen::Affine3f sensorToMav_;
    Eigen::Affine3f mavToWorld_;
    Eigen::Affine3f camToWorld_;
    std::string mavTFName_;
    std::string mavTFCameraName_;
    
    int status;
};

#endif // PC_MESH_BUILDER_H
