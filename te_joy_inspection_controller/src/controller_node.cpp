#include "controller_node.h"
#include <std_srvs/Empty.h>
#include <iostream>

Controller::Controller() :
    speedX_(0.05f), speedY_(0.05f), speedZ_(0.05f), speedYaw_(0.01f), goal_reached_(true),
    stick_to_plane_(false), sticking_distance_(1.0f), correction_speed_(0.5f) {

    std::cout << "Controller node started..." << std::endl;

    // get Ros parameters and set variables
    nh_.param("speedForward", speedX_, speedX_);
    nh_.param("speedRight", speedY_, speedY_);
    nh_.param("speedUp", speedZ_, speedZ_);
    nh_.param("speedYawn", speedYaw_, speedYaw_);

    // set subscriber and publisher
    sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Controller::Callback, this);
    sub_mocap_pose_ = nh_.subscribe<geometry_msgs::TransformStamped>("vrpn_client/estimated_transform", 10, &Controller::SetMocapPose, this);
    sub_plane_tf_ = nh_.subscribe<geometry_msgs::TransformStamped>("planes", 10, &Controller::ProcessPlaneMsg, this);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);

    // identity rotation matrix as quaternion
    tf::Quaternion q;
    q.setRPY(0,0,0);

    // init mocap_tf
    mocap_tf_.setOrigin( tf::Vector3(0,0,0) );
    mocap_tf_.setRotation(q);

    // init transform
    transform_.setOrigin( tf::Vector3(0,0,0) );
    transform_.setRotation(q);

    // TBD if we should be able to change this via launch file
    // init hover transform
    hover_goal_tf_.setOrigin( tf::Vector3(0,0,1) );
    hover_goal_tf_.setRotation(q);

    // init plane_tf
    plane_tf_.setOrigin(tf::Vector3(2,0,0));
    plane_tf_.setRotation(q);

    // init snap_goal_tf
    snap_goal_tf_.setOrigin(tf::Vector3(0,0,0));
    snap_goal_tf_.setRotation(q);
}

void Controller::SetMocapPose(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    mocap_tf_.setOrigin( tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z) );
    mocap_tf_.setRotation( tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w) );

    if(!goal_reached_) {
        tf::Vector3 diff = hover_goal_tf_.getOrigin() - mocap_tf_.getOrigin();
        if(diff.length() < 0.6f) {
            goal_reached_ = true;
        } else {
            if(diff.length() > 1.0f) {
                diff.normalize();
            }
            transform_.setOrigin(diff);
        }
    }

    testPlanes();

    // world transform
    tf::Transform curr_transform = mocap_tf_ * transform_;

    // convert tf into pose and publish the pose
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

//void Controller::Callback(const sensor_msgs::Joy::ConstPtr& joy) {
//    prev_msg_ = joy;

//    // translation from controller axis
//    float jx = speedX_ * joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
//    float jy = speedY_ * joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
//    float jz = speedZ_ * joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
//    // yaw from axis
//    float jr = speedYaw_ * joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];

//    // save only the latest relative transform in global transform
//    transform_.setOrigin( tf::Vector3(jx,jy,jz) );
//    tf::Quaternion q;
//    q.setRPY(0, 0, jr);
//    transform_.setRotation(q);

//    // listen for take off button pressed
//    if(joy->buttons[PS3_BUTTON_PAIRING] || joy->buttons[PS3_BUTTON_START]) {
//        goal_reached_ = false;
//        TakeoffAndHover();
//    }
//}

void Controller::Callback(const sensor_msgs::Joy::ConstPtr& joy) {
    prev_msg_ = joy;

    // translation from controller axis
    float jx = speedX_ * joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
    float jy = speedY_ * joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
    float jz = speedZ_ * joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
    // yaw from axis
    float jr = speedYaw_ * joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];

    // save only the latest relative transform in global transform
    transform_.setOrigin( tf::Vector3(jx,jy,jz) );
    tf::Quaternion q;
    q.setRPY(0, 0, jr);
    transform_.setRotation(q);

    // buttons
    if(joy->buttons[PS3_BUTTON_REAR_RIGHT_1]) {
        stick_to_plane_ = true;
    }
    if(joy->buttons[PS3_BUTTON_REAR_LEFT_1]) {
        stick_to_plane_ = false;
    }

    if(joy->buttons[PS3_BUTTON_CROSS_UP]) {
        sticking_distance_ -= 0.005f;
    }
    if(joy->buttons[PS3_BUTTON_CROSS_DOWN]) {
        sticking_distance_ += 0.005f;
    }

    if(stick_to_plane_) {
        Eigen::Vector3f proj_pos = testPlanes();
        tf::Vector3 diff = snap_goal_tf_.getOrigin() - mocap_tf_.getOrigin();
        if(diff.length() > 1.0f) {
            diff.normalize();
        }
        transform_.setOrigin( correction_speed_ * diff );
    }

    mocap_tf_ *= transform_;

    // convert tf into pose and publish the pose
    tf::Vector3 desired_pos = mocap_tf_.getOrigin();
    tf::Quaternion desired_rot = mocap_tf_.getRotation();

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
    br_tf_.sendTransform( tf::StampedTransform(mocap_tf_, ros::Time::now(), "world", "controller") );
}

void Controller::TakeoffAndHover() {
    std_srvs::Empty::Request request;
    std_srvs::Empty::Response response;
    if(ros::service::call("euroc2/takeoff", request, response)) {
        tf::Vector3 desired_pos = hover_goal_tf_.getOrigin();
        tf::Quaternion desired_rot = hover_goal_tf_.getRotation();

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
    }
}

void Controller::ProcessPlaneMsg(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    plane_tf_.setOrigin( tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z) );
    plane_tf_.setRotation( tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w) );
}

// TODO rename
// TBD maybe use tf instead of eigen for simplicity
Eigen::Vector3f Controller::testPlanes() {
    // mav
    tf::Transform mav_tf = mocap_tf_ * transform_;

    // Eigen conversions
    Eigen::Vector3f mav_pos = Eigen::Vector3f( mav_tf.getOrigin().x(),
                                               mav_tf.getOrigin().y(),
                                               mav_tf.getOrigin().z());
    Eigen::Quaternionf mav_rot = Eigen::Quaternionf( mav_tf.getRotation().x(),
                                                     mav_tf.getRotation().y(),
                                                     mav_tf.getRotation().z(),
                                                     mav_tf.getRotation().w());

    // rviz debug
    br_tf_.sendTransform( tf::StampedTransform(mav_tf, ros::Time::now(), "world", "mav") );

    // plane
    Eigen::Vector3f plane_pos = Eigen::Vector3f( plane_tf_.getOrigin().x(), plane_tf_.getOrigin().y(), plane_tf_.getOrigin().z() );
    tf::Quaternion plane_q = plane_tf_.getRotation();
    Eigen::Quaternionf plane_rot = Eigen::Quaternionf(plane_q.w(),plane_q.x(),plane_q.y(),plane_q.z());

    // rviz debug
    br_tf_.sendTransform( tf::StampedTransform(plane_tf_, ros::Time::now(), "world", "normal") );

    // calculations
    Eigen::Vector3f normal = plane_rot*forward;
    normal.normalize();
    Eigen::Vector3f proj_pos = mav_pos + (sticking_distance_-normal.dot(mav_pos-plane_pos))*normal;

    snap_goal_tf_.setOrigin( tf::Vector3(proj_pos.x(), proj_pos.y(), proj_pos.z()) );
    snap_goal_tf_.setRotation( mocap_tf_.getRotation() * transform_.getRotation() );

    return proj_pos;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "te_joy_controller");
    Controller rc;
    ros::spin();
}
