// This source code is intended for use in the teaching course "Vision-Based Navigation" in summer term 2015 at Technical University Munich only.
// Copyright 2015 Vladyslav Usenko, Joerg Stueckler, Technical University Munich

#ifndef UAV_CONTROLLER_H_
#define UAV_CONTROLLER_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sophus/se3.hpp>
#include <eigen_conversions/eigen_msg.h>

#include <mav_msgs/CommandAttitudeThrust.h>
#include <mav_msgs/CommandRateThrust.h>
#include <mav_msgs/CommandMotorSpeed.h>

#include <boost/thread/mutex.hpp>

#include <se3ukf.hpp>

#include <list>
#include <fstream>

#define INTEGRAL_RING_BUFFER_SIZE 200

#define KP 1
#define KD 0.5
#define KI 0


template<typename _Scalar>
class UAVController {

private:

    typedef Sophus::SE3Group<_Scalar> SE3Type;
    typedef Sophus::SO3Group<_Scalar> SO3Type;
    typedef Eigen::Quaternion<_Scalar> Quaternion;

    typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<_Scalar, 6, 1> Vector6;
    typedef Eigen::Matrix<_Scalar, 12, 1> Vector12;
    typedef Eigen::Matrix<_Scalar, 15, 1> Vector15;

    typedef Eigen::Matrix<_Scalar, 3, 3> Matrix3;
    typedef Eigen::Matrix<_Scalar, 6, 6> Matrix6;
    typedef Eigen::Matrix<_Scalar, 12, 12> Matrix12;
    typedef Eigen::Matrix<_Scalar, 15, 15> Matrix15;


    ros::Publisher command_pub;
    ros::Subscriber imu_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber ground_truth_sub;


    // Switch between ground truth and ukf for controller
    bool use_ground_thruth_data;

    // Ground thruth data
    SE3Type ground_truth_pose;
    Vector3 ground_truth_linear_velocity;
    double ground_truth_time;
    
    // UKF data
    SE3UKF<_Scalar> *ukf;
    double previous_time;

    // Constants
    _Scalar g;
    _Scalar m;
    SE3Type T_imu_cam;
    SE3Type  initial_pose;
    Matrix15 initial_state_covariance;
    Matrix3 gyro_noise;
    Matrix3 accel_noise;
    Matrix6 measurement6d_noise;

    // Pose to hoover at
    SE3Type desired_pose;

    mav_msgs::CommandAttitudeThrust command;

    // position integral ring buffer
    Vector3 position_integral[INTEGRAL_RING_BUFFER_SIZE];
    unsigned int integral_idx;

    /**
     * @brief send a new control signal every time new gyro and acceleration measurements arrive
     * TODO: does not use imu measurements
     * 
     * @param msg the new imu message
     */
    void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
        Eigen::Vector3d accel_measurement, gyro_measurement;
        tf::vectorMsgToEigen(msg->angular_velocity, gyro_measurement);
	double dt;
	if (previous_time < 0) {
	    previous_time = msg->header.stamp.toSec();	    
	}
	dt = msg->header.stamp.toSec() - previous_time;
	previous_time = msg->header.stamp.toSec();
        tf::vectorMsgToEigen(msg->linear_acceleration, accel_measurement);
	ukf->predict(accel_measurement, gyro_measurement, dt, accel_noise, gyro_noise);

        sendControlSignal(dt);
    }

    /**
     * @brief saves the ground_truth pose and velocity in global variables
     * 
     * @param msg
     * @return ground_truth_pose current pose
     * @return ground_truth_linear_velocity current velocity
     * @return ground_truth_time current time
     */
    void groundTruthPoseCallback(
        const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
        Eigen::Quaterniond orientation;
        Eigen::Vector3d position;

        tf::pointMsgToEigen(msg->pose.pose.position, position);
        tf::quaternionMsgToEigen(msg->pose.pose.orientation, orientation);

        SE3Type pose(orientation.cast<_Scalar>(), position.cast<_Scalar>());

        ground_truth_linear_velocity = (pose.translation()
                                        - ground_truth_pose.translation())
                                       / (msg->header.stamp.toSec() - ground_truth_time);

        ground_truth_pose = pose;
        ground_truth_time = msg->header.stamp.toSec();
    }

    /**
     * @brief TODO: fuse pose and imu information
     * 
     * @param msg PoseWithCovarianceStampedConstPtr message
     */
    void pose1Callback(
        const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
        Eigen::Quaterniond orientation;
        Eigen::Vector3d position;
	
	Matrix6 covariance;// = Eigen::Map<Matrix6>(msg->pose.covariance);
	
	for (int i = 0; i < 36; i++) {
	    covariance.data()[i] = msg->pose.covariance[i];
	}

        tf::pointMsgToEigen(msg->pose.pose.position, position);
        tf::quaternionMsgToEigen(msg->pose.pose.orientation, orientation);

        SE3Type pose(orientation, position);
	ukf->measurePose(pose, covariance);
    }

    // TODO: exercise 1 d)
    /**
     *  @brief calculates the attitude and thrust from the desired control force
     * 
     *  @param control_force the desired force, that the rotors should apply
     *  @return the CommandAttitudeThrust message 
     */
    mav_msgs::CommandAttitudeThrust computeCommandFromForce(
        const Vector3 & control_force, const SE3Type & pose,
        double delta_time) {
        float yaw = 0.0f;
        // f = m * a    ==>     a = f / m
        Vector3 a = control_force/m;
        float roll =  1.0f/g * (a.x()*sin(yaw) - a.y()*cos(yaw));
        float pitch = 1.0f/g * (a.x()*cos(yaw) + a.y()*sin(yaw));
        float thrust = control_force.z() + m*g;

        mav_msgs::CommandAttitudeThrust msg;
        msg.roll = roll;    // [rad]
        msg.pitch = pitch;  // [rad]
        msg.yaw_rate = yaw; // [rad/s]
        msg.thrust = thrust;
        return msg;
    }

    // TODO: exercise 1 c)
    
    /**
     * @brief compute the desired force with a simply PID
     * 
     * @param curr_pose/curr_velocity gets current pose and velocity automatically
     * @param pose desired position and orientation
     * @param linear_velocity desired velocity
     * @param delta_time timestep for the integration part of the PID
     * @return the desired force
     */
    Vector3 computeDesiredForce(const SE3Type & pose, const Vector3 & linear_velocity,
                                double delta_time) {
        SE3Type curr_pose;
        Vector3 curr_velocity;
        getPoseAndVelocity(curr_pose, curr_velocity);

        const float kp = KP; // proportional gains of the PID controller
        const float kd = KD; // differential gains of the PID controller
        const float ki = KI; // integral gains of the PID controller

        // x'' = kp*(xd-x) + kd*(xd'-x') + ki*integral(xd-x, delta_time)
//        position_integral += delta_time*(pose.translation() - curr_pose.translation());
        position_integral[integral_idx] = delta_time*(pose.translation() - curr_pose.translation());
        integral_idx = (integral_idx+1) % INTEGRAL_RING_BUFFER_SIZE;

        // calculate integral
        Vector3 integral = Vector3(0,0,0);
        for(int i=0; i<INTEGRAL_RING_BUFFER_SIZE; i++) {
            integral += position_integral[i];
        }

        // calculate acceleration for force calculation
        Vector3 a = kp*(pose.translation() - curr_pose.translation()) +
                    kd*(linear_velocity - curr_velocity) +
                    ki*integral;

        return m*a; // f = m * a
    }

    // TODO: exercise 1 b)
    
    /**
     * @brief returns the current pose and velocity (either ground truth or estimated)
     * 
     * @param pose return current pose
     * @param linear_velocity return current velocity
     */
    void getPoseAndVelocity(SE3Type & pose, Vector3 & linear_velocity) {
        if (use_ground_thruth_data) {
            pose = ground_truth_pose;
            linear_velocity = ground_truth_linear_velocity;
        } else {
	    pose = ukf->get_pose();
	    linear_velocity = ukf->get_linear_velocity();
        }
    }

public:

    typedef boost::shared_ptr<UAVController> Ptr;

    UAVController(ros::NodeHandle & nh) :
        ground_truth_time(0),
        previous_time(-1),
        integral_idx(0) {

        use_ground_thruth_data = true;

        // ========= Constants ===================================//
        g = 9.8; // in rate_controller g = 9.81
        m = 1.55; // in rate_controller m = 1.56779
        initial_state_covariance = Matrix15::Identity() * 0.001;
        gyro_noise = Matrix3::Identity() * 0.0033937;
        accel_noise = Matrix3::Identity() * 0.04;
        measurement6d_noise = Matrix6::Identity() * 0.01;
        initial_pose.translation() << 0, 0, 0.08;

        // Set simulated camera to IMU transformation
        Eigen::AngleAxisd rollAngle(0.2, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(-0.1, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(0.3, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        T_imu_cam.setQuaternion(q);
        T_imu_cam.translation() << 0.03, -0.07, 0.1;
	
	// Init uncsented kallman filter
	ukf = new SE3UKF<_Scalar>(initial_pose, Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0), initial_state_covariance);

        // Init subscribers and publishers
        imu_sub = nh.subscribe("imu", 10, &UAVController<_Scalar>::imuCallback,
                               this);
        pose_sub = nh.subscribe("pose1", 10,
                                &UAVController<_Scalar>::pose1Callback, this);
        ground_truth_sub = nh.subscribe("ground_truth/pose", 10,
                                        &UAVController<_Scalar>::groundTruthPoseCallback, this);

        command_pub = nh.advertise<mav_msgs::CommandAttitudeThrust>(
                          "command/attitude", 10);

        // initialize position integral
        for(int i=0; i<INTEGRAL_RING_BUFFER_SIZE; i++) {
            position_integral[i] = Vector3(0,0,0);
        }

        // Wake up simulation, from this point on, you have a few seconds to initialize
        // everything and fly to the evaluation position (x=0m y=0m z=1m).
        ROS_INFO("Waking up simulation ... ");
        std_srvs::Empty srv;
        bool ret = ros::service::call("/gazebo/unpause_physics", srv);

        if (ret)
            ROS_INFO("... ok");
        else {
            ROS_FATAL("could not wake up gazebo");
            exit(-1);
        }
    }

    ~UAVController() {
	delete ukf;
    }

    // TODO: exercise 1 e)
    /**
     * @brief calculates and publishes a CommandAttitudeThrust message from the desired_pose
     */
    void sendControlSignal(double delta_time) {
        Vector3 desired_velocity = Vector3(0,0,0);

        Vector3 dforce = computeDesiredForce(desired_pose, desired_velocity, delta_time);

        command = computeCommandFromForce(dforce, desired_pose, delta_time);
        command_pub.publish(command);
    }


    void setDesiredPose(const SE3Type & p) {
        desired_pose = p;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* UAV_CONTROLLER_H_ */
