#include "controller_node.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <iostream>

#define STICK_DEADZONE 0.1f

#include <fstream>
static std::ofstream filestream_;

Controller::Controller() :
    goal_reached_(true),
    surfaceType_(plane),
    search_for_surface_(false),
    stick_to_surface_(false),
    sticking_distance_(0.5f),
    correction_speed_(0.5f),
    is_active_(false)
{
    std::cout << "Controller node started..." << std::endl;
    
    nh_ = ros::NodeHandle("");
    private_nh_ = ros::NodeHandle("~");

    // set stick deadzone
    stick_deadzone_ = Deadzone<float>(STICK_DEADZONE);

    // get Ros parameters and set variables
    private_nh_.param("speedForward", speedX_, 0.5f);
    private_nh_.param("speedRight", speedY_, 0.5f);
    private_nh_.param("speedUp", speedZ_, 0.5f);
    private_nh_.param("speedYaw", speedYaw_, 0.05f);

    // set subscriber and publisher
    sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Controller::callback, this);
    sub_mocap_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("estimated_transform", 10, &Controller::setMocapPose, this);
    sub_surface_tf_ = nh_.subscribe<surface_detection_msgs::Surface>("te_surface_detection/surface", 10, &Controller::processSurfaceMsg, this);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
    pub_stickToSurface_ = nh_.advertise<std_msgs::Bool>("controller/stickToSurface", 10);
    pub_surfaceType_ = nh_.advertise<std_msgs::Int32>("controller/surfaceType", 10);

    std::cout << "speeds: " << speedX_ << " " << speedY_ << " " << speedZ_ << " " << speedYaw_ << std::endl;

    // identity rotation matrix as quaternion
    tf::Quaternion q;
    q.setRPY(0,0,0);

    // init mocap_tf
    mavToWorld_.setOrigin( tf::Vector3(0,0,0) );
    mavToWorld_.setRotation(q);

    // init transform
    transform_.setOrigin( tf::Vector3(0,0,0) );
    transform_.setRotation(q);

    // TBD if we should be able to change this via launch file
    // init hover transform
    hover_goal_tf_.setOrigin( tf::Vector3(0,0,1) );
    hover_goal_tf_.setRotation(q);

    // init plane_tf
    surface_tf_.setOrigin(tf::Vector3(2,0,0));
    tf::Quaternion surface_q;
    surface_q.setRPY(0,0,-M_PI/4.f);
    surface_tf_.setRotation(surface_q);
    cylinder_radius_ = 0.5f;

    // init snap_goal_tf
    snap_goal_tf_.setOrigin(tf::Vector3(0,0,0));
    snap_goal_tf_.setRotation(q);

    // hardcoded sensorToMav transform
    tf::Transform mavToWorld;
    mavToWorld.setOrigin(tf::Vector3(0,0,0.117));
    mavToWorld.setRotation(tf::Quaternion(0,0,0,1));
    tf::Transform sensorToWorld;
    sensorToWorld.setOrigin(tf::Vector3(0.133,0,0.0605));
    sensorToWorld.setRotation(tf::Quaternion(0,0.17365,0,0.98481));
    sensorToMav_ = mavToWorld.inverse() * sensorToWorld;
}

void Controller::setMocapPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    mavToWorld_.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
    mavToWorld_.setRotation( tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) );

    /// takeoff and hover blocks all other inputs until goal position is reached
    if(!goal_reached_) {
        tf::Transform diff = mavToWorld_.inverse() * hover_goal_tf_;
        if(diff.getOrigin().length() < 0.1f) {
            goal_reached_ = true;
        } else {
            transform_ = diff;
        }
    }
    
    if (!is_active_) {
        return;
    }
    is_active_ = false;

    // world transform
    tf::Transform curr_transform;
    /// stick to plane
    if(stick_to_surface_ && goal_reached_) {
        switch(surfaceType_) {
            case plane:
                testPlanes();
                break;
            case cylinder:
                testCylinders();
                break;
            default:
            std::cout << "No snapping goal set..." << std::endl;
                break;
        }
        tf::Vector3 diff = snap_goal_tf_.getOrigin() - mavToWorld_.getOrigin();
        curr_transform.setOrigin(mavToWorld_.getOrigin() + correction_speed_*diff);
        //curr_transform.setOrigin(snap_goal_tf_.getOrigin());
        curr_transform.setRotation(snap_goal_tf_.getRotation());
    } else {
        curr_transform = mavToWorld_ * transform_;
    }

    /// convert tf into pose and publish the pose
    tf::Vector3 desired_pos = curr_transform.getOrigin();
    tf::Quaternion desired_rot = curr_transform.getRotation();

    geometry_msgs::PoseStamped pose;
    // set header
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    // set pose
    pose.pose.position.x = desired_pos.x();
    pose.pose.position.y = desired_pos.y();
    pose.pose.position.z = desired_pos.z();
    pose.pose.orientation.x = desired_rot.x();
    pose.pose.orientation.y = desired_rot.y();
    pose.pose.orientation.z = desired_rot.z();
    pose.pose.orientation.w = desired_rot.w();
    pub_pose_.publish(pose);

    // rviz debug
    br_tf_.sendTransform( tf::StampedTransform(curr_transform, ros::Time::now(), "world", "controller") );
}

void Controller::callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    /// buttons
    // poll button states and check for changed buttons
    for(int i=0; i<17; i++) { // PS3 controller has 17 seperate buttons
        if(joy->buttons[i] != buttonDown_[i])
            buttonChanged_[i] = true;
        else
            buttonChanged_[i] = false;

        buttonDown_[i] = joy->buttons[i];
    }

    // take off
    if(joy->buttons[PS3_BUTTON_PAIRING] || joy->buttons[PS3_BUTTON_START]) {
        goal_reached_ = false;
        takeoffAndHover();
    }
    // change surface type
    if(joy->buttons[PS3_BUTTON_SELECT] && buttonChanged_[PS3_BUTTON_SELECT]) {
        surfaceType_ = surfaceType_ == plane ? cylinder : plane;
        std_msgs::Int32 type;
        type.data = surfaceType_;
        pub_surfaceType_.publish(type);
    }
    // set search for surface
    if(joy->buttons[PS3_BUTTON_REAR_RIGHT_2] && buttonChanged_[PS3_BUTTON_REAR_RIGHT_2]) {
        search_for_surface_ = true;
        std_msgs::Bool sticking;
        sticking.data = search_for_surface_;
        pub_stickToSurface_.publish(sticking);
    }
    if(joy->buttons[PS3_BUTTON_REAR_LEFT_2] && buttonChanged_[PS3_BUTTON_REAR_LEFT_2]) {
        search_for_surface_ = false;
        std_msgs::Bool sticking;
        sticking.data = search_for_surface_;
        pub_stickToSurface_.publish(sticking);
    }
    // enable or disable sticking to the plane
    if(joy->buttons[PS3_BUTTON_REAR_RIGHT_1] && !stick_to_surface_) {
        stick_to_surface_ = true;
        is_active_ = true;

        // set current distance to surface as sticking distance
        Eigen::Vector3f plane_pos = Eigen::Vector3f( surface_tf_.getOrigin().x(), surface_tf_.getOrigin().y(), surface_tf_.getOrigin().z() );
        tf::Quaternion plane_q = surface_tf_.getRotation();
        Eigen::Quaternionf plane_rot = Eigen::Quaternionf(plane_q.w(),plane_q.x(),plane_q.y(),plane_q.z());
        Eigen::Vector3f normal = plane_rot*forward;
        normal.normalize();
        Eigen::Vector3f curr_pos = Eigen::Vector3f(mavToWorld_.getOrigin().x(), mavToWorld_.getOrigin().y(), mavToWorld_.getOrigin().z());
        sticking_distance_ = -normal.dot(curr_pos-plane_pos);
    }
    if(joy->buttons[PS3_BUTTON_REAR_LEFT_1] && stick_to_surface_) {
        stick_to_surface_ = false;
    }
    // sticking distance
    if(joy->buttons[PS3_BUTTON_CROSS_UP] && abs(sticking_distance_) > 0.5f) {
        sticking_distance_ -= 0.005f;
        is_active_ = true;
    }
    if(joy->buttons[PS3_BUTTON_CROSS_DOWN]) {
        sticking_distance_ += 0.005f;
        is_active_ = true;
    }

    /// axes
    if( std::abs(joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS]) > STICK_DEADZONE ||
        std::abs(joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS]) > STICK_DEADZONE ||
        std::abs(joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS]) > STICK_DEADZONE ||
        std::abs(joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]) > STICK_DEADZONE) {
        is_active_ = true;
    }
    /// translation from controller axis
    float jx = speedX_ * stick_deadzone_(joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS]);
    float jy = speedY_ * stick_deadzone_(joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS]);
    float jz = speedZ_ * stick_deadzone_(joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS]);
    // yaw from axis
    float jr = speedYaw_ * stick_deadzone_(joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]);

    // save only the latest relative transform in global variable transform_
    transform_.setOrigin( tf::Vector3(jx,jy,jz) );
    tf::Quaternion q;
    q.setRPY(0,0,jr);
    transform_.setRotation(q);
}

void Controller::takeoffAndHover()
{
    std_srvs::Empty::Request request;
    std_srvs::Empty::Response response;
    if(ros::service::call("euroc2/takeoff", request, response)) {
        tf::Vector3 desired_pos = tf::Vector3( mavToWorld_.getOrigin().x(), mavToWorld_.getOrigin().y(), 1.0f);
        tf::Quaternion desired_rot = mavToWorld_.getRotation();

        hover_goal_tf_.setOrigin(desired_pos);
        hover_goal_tf_.setRotation(desired_rot);
    }
}

void Controller::processSurfaceMsg(const surface_detection_msgs::Surface::ConstPtr& msg)
{
    if(surfaceType_ == plane) {
        // realtive plane transform
        // planeToCam = surface_tf
        surface_tf_.setOrigin( tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z) );
        surface_tf_.setRotation( tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w) );

        // plane forward is z should be x, conversion of coordinate systems
        // camToSensor
        tf::Quaternion q1; q1.setRPY(0,M_PI/2.0,0);
        tf::Quaternion q2; q2.setRPY(-M_PI/2.0,0,0);
        tf::Quaternion opticalToSensor = q2*q1;
        surface_tf_.setOrigin( tf::Matrix3x3(opticalToSensor)*surface_tf_.getOrigin() );
        surface_tf_.setRotation( opticalToSensor*surface_tf_.getRotation() );
        //br_tf_.sendTransform( tf::StampedTransform(surface_tf_, ros::Time::now(), "euroc_hex/vi_sensor/ground_truth", "plane_Sensor") );

        // correct the relative camera offset
        // planeToMav = surface_tf
        surface_tf_ = sensorToMav_*surface_tf_;
    }  else if(surfaceType_ == cylinder){
        // realtive cylinder transform
        // cylinderToCam = surface_tf
        surface_tf_.setOrigin( tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z) );
        surface_tf_.setRotation( tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w) );
        cylinder_radius_ = msg->radius;
    } else {
        std::cout << "Unknown surface type selected." << std::endl;
        return;
    }

    /// transform into global coordinate system and send debug transform
    // transform into global coordinates
    // planeToWorld = surface_tf_
    surface_tf_ = mavToWorld_*surface_tf_;

    // rviz debug
    br_tf_.sendTransform( tf::StampedTransform(surface_tf_, ros::Time::now(), "world", "controller/surface") );
}

void Controller::testPlanes()
{
    //// mav
    // predicted mav tf
    tf::Transform mav_tf = mavToWorld_ * transform_;

    // Eigen conversions
    Eigen::Vector3f mav_pos = Eigen::Vector3f( mav_tf.getOrigin().x(),
                                               mav_tf.getOrigin().y(),
                                               mav_tf.getOrigin().z());

    //// plane
    Eigen::Vector3f plane_pos = Eigen::Vector3f( surface_tf_.getOrigin().x(), surface_tf_.getOrigin().y(), surface_tf_.getOrigin().z() );
    tf::Quaternion plane_q = surface_tf_.getRotation();
    Eigen::Quaternionf plane_rot = Eigen::Quaternionf(plane_q.w(),plane_q.x(),plane_q.y(),plane_q.z());

    //// calculations
    Eigen::Vector3f normal = plane_rot * forward;
    normal.normalize();
    // determine if mav is in front or behind plane normal, take current pos to avoid switching of sides by wrong predictions in mav_tf
    Eigen::Vector3f curr_pos = Eigen::Vector3f(mavToWorld_.getOrigin().x(), mavToWorld_.getOrigin().y(), mavToWorld_.getOrigin().z());

    int facing = normal.dot(curr_pos-plane_pos) >= 0 ? 1 : -1;
    // calculate projected position of the mav onto the plane
    Eigen::Vector3f proj_pos = mav_pos + (facing*sticking_distance_ - normal.dot(mav_pos-plane_pos))*normal;

    /// set snapping goal tf
    snap_goal_tf_.setOrigin( tf::Vector3(proj_pos.x(), proj_pos.y(), proj_pos.z()) );
    // TODO: fix rotation to only change the yaw of the rotation
    snap_goal_tf_.setRotation(surface_tf_.getRotation());

    // rviz debug
    br_tf_.sendTransform( tf::StampedTransform(snap_goal_tf_, ros::Time::now(), "world", "controller/proj_pos") );
}

void Controller::testCylinders()
{
    //// mav
    // predicted mav tf
    tf::Transform mav_tf = mavToWorld_ * transform_;

    // Eigen conversions, predicted mav pos
    Eigen::Vector3f mav_pos = Eigen::Vector3f( mav_tf.getOrigin().x(),
                                               mav_tf.getOrigin().y(),
                                               mav_tf.getOrigin().z());

    //// cylinder
    Eigen::Vector3f cylinder_pos = Eigen::Vector3f( surface_tf_.getOrigin().x(), surface_tf_.getOrigin().y(), surface_tf_.getOrigin().z() );
    tf::Quaternion cylinder_q = surface_tf_.getRotation();
    Eigen::Quaternionf cylinder_rot = Eigen::Quaternionf(cylinder_q.w(),cylinder_q.x(),cylinder_q.y(),cylinder_q.z());

    //// calculations
    Eigen::Vector3f normal = cylinder_rot * forward;
    normal.normalize();

    // determine if mav is in front or behind plane normal, take current pos to avoid switching of sides by wrong predictions in mav_tf
    Eigen::Vector3f curr_pos = Eigen::Vector3f(mavToWorld_.getOrigin().x(), mavToWorld_.getOrigin().y(), mavToWorld_.getOrigin().z());
    int facing = normal.dot(curr_pos-cylinder_pos) >= 0 ? 1 : -1;
    // calculate projected position of the mav onto the cylinder
    // TODO: change radius depending on the angle, right now treat cylinder at this point as sphere
    Eigen::Vector3f proj_pos = mav_pos + (facing*(sticking_distance_ + cylinder_radius_) - normal.dot(mav_pos-cylinder_pos))*normal;

    /// set snapping goal tf
    snap_goal_tf_.setOrigin( tf::Vector3(proj_pos.x(), proj_pos.y(), proj_pos.z()) );
    // TODO: fix rotation to only change the yaw of the rotation
    snap_goal_tf_.setRotation(surface_tf_.getRotation());

    // rviz debug
    br_tf_.sendTransform( tf::StampedTransform(snap_goal_tf_, ros::Time::now(), "world", "controller/proj_pos") );
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "te_joy_controller");
    Controller rc;
    ros::spin();
}
