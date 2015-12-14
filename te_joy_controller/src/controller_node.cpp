#include "controller_node.h"
#include <std_srvs/Empty.h>
#include <iostream>

Controller::Controller() :
    speedX_(0.05f), speedY_(0.05f), speedZ_(0.05f), speedYaw_(0.01f), goal_reached_(true),
    stick_to_plane_(false), sticking_distance_(1.0f) {

    std::cout << "Controller node started..." << std::endl;

    // get Ros parameters and set variables
    nh_.param("speedForward", speedX_, speedX_);
    nh_.param("speedRight", speedY_, speedY_);
    nh_.param("speedUp", speedZ_, speedZ_);
    nh_.param("speedYawn", speedYaw_, speedYaw_);

    // set subscriber and publisher
    sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Controller::Callback, this);
    sub_mocap_pose_ = nh_.subscribe<geometry_msgs::TransformStamped>("euroc2/vrpn_client/estimated_transform", 10, &Controller::SetMocapPose, this);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("euroc2/command/pose", 1);

    // identity rotation matrix as quaternion
    tf::Quaternion q;
    q.setRPY(0,0,0);

    // init tf_mocap
    tf_mocap_.setOrigin( tf::Vector3(0,0,0) );
    tf_mocap_.setRotation(q);

    // init transform
    transform_.setOrigin( tf::Vector3(0,0,0) );
    transform_.setRotation(q);

    // TBD if we should be able to change this via launch file
    // init hover transform
    transform_hover_goal_.setOrigin( tf::Vector3(0,0,1) );
    transform_hover_goal_.setRotation(q);
}

void Controller::SetMocapPose(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    tf_mocap_.setOrigin( tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z) );
    tf_mocap_.setRotation( tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w) );

    if(!goal_reached_) {
        tf::Vector3 diff = transform_hover_goal_.getOrigin() - tf_mocap_.getOrigin();
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
    tf::Transform curr_transform = tf_mocap_ * transform_;

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
        testPlanes();
    } else {
        // world transform
        tf_mocap_ *= transform_;
    }

    // convert tf into pose and publish the pose
    tf::Vector3 desired_pos = tf_mocap_.getOrigin();
    tf::Quaternion desired_rot = tf_mocap_.getRotation();

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
    br_tf_.sendTransform( tf::StampedTransform(tf_mocap_, ros::Time::now(), "world", "controller") );
}

void Controller::TakeoffAndHover() {
    std_srvs::Empty::Request request;
    std_srvs::Empty::Response response;
    if(ros::service::call("euroc2/takeoff", request, response)) {
        tf::Vector3 desired_pos = transform_hover_goal_.getOrigin();
        tf::Quaternion desired_rot = transform_hover_goal_.getRotation();

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

void Controller::processPlaneMsg() {
    // TODO set active plane
}

// TODO rename
void Controller::testPlanes() {
    // mav
    tf::Transform mav_tf = tf_mocap_ * transform_;

    Eigen::Vector3f mav_pos = Eigen::Vector3f( mav_tf.getOrigin().x(),
                                              mav_tf.getOrigin().y(),
                                              mav_tf.getOrigin().z());
    Eigen::Quaternionf mav_rot = Eigen::Quaternionf( mav_tf.getRotation().x(),
                                                     mav_tf.getRotation().y(),
                                                     mav_tf.getRotation().z(),
                                                     mav_tf.getRotation().w());

    mav_tf.setOrigin( tf::Vector3(mav_pos.x(), mav_pos.y(), mav_pos.z()) );
    br_tf_.sendTransform( tf::StampedTransform(mav_tf, ros::Time::now(), "world", "mav") );

    // plane
    Eigen::Vector3f plane_pos = Eigen::Vector3f(2,0,0);
    tf::Quaternion plane_q;
    plane_q.setRPY(0,0,180.0f);
    Eigen::Quaternionf plane_rot = Eigen::Quaternionf(plane_q.x(),plane_q.y(),plane_q.z(),plane_q.w());

    tf::Transform plane_tf;
    plane_tf.setOrigin( tf::Vector3(plane_pos.x(), plane_pos.y(), plane_pos.z()) );
    plane_tf.setRotation(plane_q);
    br_tf_.sendTransform( tf::StampedTransform(plane_tf, ros::Time::now(), "world", "normal") );

    // calculations
    Eigen::Vector3f normal = plane_rot*forward;
    Eigen::Vector3f proj_pos = mav_pos + (sticking_distance_ - normal.dot(mav_pos-plane_pos)) * normal;

    // world transform
    tf_mocap_.setOrigin( tf::Vector3(proj_pos.x(), proj_pos.y(), proj_pos.z()) );
    tf_mocap_.setRotation( tf_mocap_.getRotation() * transform_.getRotation() );
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "te_joy_controller");
    Controller rc;
    ros::spin();
}
