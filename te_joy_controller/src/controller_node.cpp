#include "controller_node.h"
#include <std_srvs/Empty.h>
#include <iostream>

Controller::Controller() :
    speedX_(1.0f), speedY_(1.0f), speedZ_(1.0f), speedYawn_(0.5f),
    goal_reached_(true)
{
    // get Ros parameters and set variables
    nh_.param("speedForward", speedX_, speedX_);
    nh_.param("speedRight", speedY_, speedY_);
    nh_.param("speedUp", speedZ_, speedZ_);
    nh_.param("speedYawn", speedYawn_, speedYawn_);

    // set subscriber and publisher
    sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Controller::Callback, this);
    sub_mocap_pose_ = nh_.subscribe<geometry_msgs::TransformStamped>("vrpn_client/estimated_transform", 10, &Controller::SetMocapPose, this);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);

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

void Controller::SetMocapPose(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
    tf_mocap_.setOrigin( tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z) );
    tf_mocap_.setRotation( tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w) );

    if(!goal_reached_)
    {
        tf::Vector3 diff = transform_hover_goal_.getOrigin() - tf_mocap_.getOrigin();
        if(diff.length() < 0.6f)
        {
            goal_reached_ = true;
        }
        else
        {
            if(diff.length() > 1.0f)
            {
                diff.normalize();
            }
            transform_.setOrigin(diff);
        }
    }

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

void Controller::Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    prev_msg_ = joy;

    // translation from controller axis
    float jx = speedX_ * joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
    float jy = speedY_ * joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
    float jz = speedZ_ * joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
    // yawn from axis
    float jr = speedYawn_ * joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];

    // save only the latest relative transform in global transform
    transform_.setOrigin( tf::Vector3(jx,jy,jz) );
    tf::Quaternion q;
    q.setRPY(0, 0, jr);
    transform_.setRotation(q);

    // listen for take off button pressed
    if(joy->buttons[PS3_BUTTON_PAIRING] || joy->buttons[PS3_BUTTON_START])
    {
        goal_reached_ = false;
        TakeoffAndHover();
    }
}

// TBD response might be to strong because we send the mav the distance
void Controller::TakeoffAndHover()
{
    std_srvs::Empty::Request request;
    std_srvs::Empty::Response response;
    if(ros::service::call("euroc2/takeoff", request, response))
    {
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "te_joy_controller");
    Controller rc;
    ros::spin();
}
