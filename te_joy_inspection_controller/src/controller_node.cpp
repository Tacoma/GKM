#include "controller_node.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <iostream>

Controller::Controller() :
    speedX_(1.05f), speedY_(1.05f), speedZ_(1.05f), speedYaw_(1.01f), goal_reached_(true),
    search_for_plane_(false), stick_to_plane_(false), sticking_distance_(0.5f), correction_speed_(0.5f) {

    std::cout << "Controller node started..." << std::endl;

    // get Ros parameters and set variables
    nh_.param("speedForward", speedX_, speedX_);
    nh_.param("speedRight", speedY_, speedY_);
    nh_.param("speedUp", speedZ_, speedZ_);
    nh_.param("speedYaw", speedYaw_, speedYaw_);

    // set subscriber and publisher
    sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Controller::callback, this);
    sub_mocap_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("vrpn_client/estimated_transform", 10, &Controller::setMocapPose, this);
    sub_plane_tf_ = nh_.subscribe<geometry_msgs::TransformStamped>("plane", 10, &Controller::processPlaneMsg, this);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
    pub_stickToSurface_ = nh_.advertise<std_msgs::Bool>("stickToSurface", 10);

    std::cout << "speeds: " << speedX_ << " " << speedY_ << " " << speedZ_ << " " << speedYaw_ << std::endl;

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
    tf::Quaternion plane_q;
    plane_q.setRPY(0,0,-M_PI/4.f);
    plane_tf_.setRotation(plane_q);

    // init snap_goal_tf
    snap_goal_tf_.setOrigin(tf::Vector3(0,0,0));
    snap_goal_tf_.setRotation(q);
}

void Controller::setMocapPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    mocap_tf_.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
    mocap_tf_.setRotation( tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) );

    if(!goal_reached_) {
        tf::Transform diff = mocap_tf_.inverse() * hover_goal_tf_;
        if(diff.getOrigin().length() < 0.05f) {
            goal_reached_ = true;
        }
        else {
            transform_ = diff;
        }
    }

    // world transform
    tf::Transform curr_transform;
    /// stick to plane
    if(stick_to_plane_) {
        testPlanes();
        tf::Vector3 diff = snap_goal_tf_.getOrigin() - mocap_tf_.getOrigin();
        curr_transform.setOrigin(mocap_tf_.getOrigin() + correction_speed_*diff);
        curr_transform.setRotation(snap_goal_tf_.getRotation());
    } else {
        curr_transform = mocap_tf_ * transform_;
    }

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

void Controller::callback(const sensor_msgs::Joy::ConstPtr& joy) {
    /// translation from controller axis
    float jx = speedX_ * joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
    float jy = speedY_ * joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
    float jz = speedZ_ * joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
    // yaw from axis
    float jr = speedYaw_ * joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];

    // save only the latest relative transform in global variable transform_
    transform_.setOrigin( tf::Vector3(jx,jy,jz) );
    tf::Quaternion q;
    q.setRPY(0,0,jr);
    transform_.setRotation(q);

    /// buttons
    // tell PcMeshBuilder to search for a plane
    if(joy->buttons[PS3_BUTTON_REAR_RIGHT_1]) {
        search_for_plane_ = true;
        std_msgs::Bool sticking;
        sticking.data = search_for_plane_;
        pub_stickToSurface_.publish(sticking);
    }
    if(joy->buttons[PS3_BUTTON_REAR_LEFT_1]) {
        search_for_plane_ = false;
        std_msgs::Bool sticking;
        sticking.data = search_for_plane_;
        pub_stickToSurface_.publish(sticking);
    }
    // enable or disable sticking to the plane
    if(joy->buttons[PS3_BUTTON_REAR_RIGHT_2] && search_for_plane_) {
        stick_to_plane_ = true;
    }
    if(joy->buttons[PS3_BUTTON_REAR_LEFT_2]) {
        stick_to_plane_ = false;
    }
    // sticking distance
    if(joy->buttons[PS3_BUTTON_CROSS_UP] && sticking_distance_ > 0.05f) {
        sticking_distance_ -= 0.005f;
    }
    if(joy->buttons[PS3_BUTTON_CROSS_DOWN]) {
        sticking_distance_ += 0.005f;
    }
}

void Controller::takeoffAndHover() {
    std_srvs::Empty::Request request;
    std_srvs::Empty::Response response;
    ros::service::call("euroc2/takeoff", request, response);

    tf::Vector3 desired_pos = tf::Vector3( mocap_tf_.getOrigin().x(), mocap_tf_.getOrigin().y(), 1.0f);
    tf::Quaternion desired_rot = mocap_tf_.getRotation();

    hover_goal_tf_.setOrigin(desired_pos);
    hover_goal_tf_.setRotation(desired_rot);
}

// TODO add constant offset transform for camera
void Controller::processPlaneMsg(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    // realtive plane transform to mav
    plane_tf_.setOrigin( tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z) );
    plane_tf_.setRotation( tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w) );

    // transform into global coordinates
    plane_tf_ = plane_tf_*mocap_tf_;
}

void Controller::testPlanes() {
    //// mav
    // predicted mav tf
    tf::Transform mav_tf = mocap_tf_ * transform_;

    // Eigen conversions
    Eigen::Vector3f mav_pos = Eigen::Vector3f( mav_tf.getOrigin().x(),
                                               mav_tf.getOrigin().y(),
                                               mav_tf.getOrigin().z());

    //// plane
    Eigen::Vector3f plane_pos = Eigen::Vector3f( plane_tf_.getOrigin().x(), plane_tf_.getOrigin().y(), plane_tf_.getOrigin().z() );
    tf::Quaternion plane_q = plane_tf_.getRotation();
    Eigen::Quaternionf plane_rot = Eigen::Quaternionf(plane_q.w(),plane_q.x(),plane_q.y(),plane_q.z());

    // rviz debug
    br_tf_.sendTransform( tf::StampedTransform(plane_tf_, ros::Time::now(), "world", "normal") );

    //// calculations
    Eigen::Vector3f normal = plane_rot*forward;
    normal.normalize();
    normal = -normal;
    // determine if mav is in front or behind plane normal, take current pos to avoid switching of sides by wrong predictions in mav_tf
    Eigen::Vector3f curr_pos = Eigen::Vector3f(mocap_tf_.getOrigin().x(), mocap_tf_.getOrigin().y(), mocap_tf_.getOrigin().z());

    int facing = normal.dot(curr_pos-plane_pos) >= 0 ? 1 : -1;
    // calculate projected position of the mav onto the plane
    Eigen::Vector3f proj_pos = mav_pos + (facing*sticking_distance_ - normal.dot(mav_pos-plane_pos))*normal;

    // set snapping goal tf
    snap_goal_tf_.setOrigin( tf::Vector3(proj_pos.x(), proj_pos.y(), proj_pos.z()) );
    snap_goal_tf_.setRotation(plane_tf_.getRotation());
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "te_joy_controller");
    Controller rc;
    ros::spin();
}
