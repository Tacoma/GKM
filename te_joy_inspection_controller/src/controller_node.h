#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include "sophus/sim3.hpp"
#include "deadzone.h"
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// buttons are either 0 or 1
#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

// stick axes go from 0 to +/-1
// button axes go from 0 to -1
#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

static bool buttonDown_[17] = {false};
static bool buttonChanged_[17] = {false};

enum SurfaceType { plane=1, cylinder=2, NUM_SURFACE_TYPES=3 };
static const Eigen::Vector3f forward = Eigen::Vector3f(1,0,0);

class Controller
{
public:
    Controller();
    ~Controller() {}

    void callback(const sensor_msgs::Joy::ConstPtr& joy);
    void setMocapPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void takeoffAndHover();

private:
    void processSurfaceMsg(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void testPlanes();
    void testCylinders();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // ros subscriber, publisher and broadcaster
    ros::Subscriber sub_joy_;
    ros::Subscriber sub_mocap_pose_;
    ros::Subscriber sub_surface_tf_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_stickToSurface_;
    ros::Publisher pub_surfaceType_;
    tf::TransformBroadcaster br_tf_;

    tf::Transform sensorToMav_;
    tf::Transform mavToWorld_;
    tf::Transform transform_;

    bool goal_reached_;
    tf::Transform hover_goal_tf_;

    SurfaceType surfaceType_;
    bool search_for_surface_;
    bool stick_to_surface_;
    float sticking_distance_;
    float cylinder_radius_;
    float correction_speed_;
    tf::Transform surface_tf_;
    tf::Transform snap_goal_tf_;

    // stick deadzone
    Deadzone<float> stick_deadzone_ = Deadzone<float>(0.1f);
    bool is_active_;

    // ros parameters
    float speedY_;      // speedRightLeft
    float speedX_;      // speedForwardBackward
    float speedZ_;      // speedUpDown
    float speedYaw_;    // speedYaw
};

#endif // CONTROLLER_H_
