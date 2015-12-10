#ifndef DEPTH_MESH_DISPLAY_H
#define DEPTH_MESH_DISPLAY_H

#include <rviz/message_filter_display.h>

#include <ros/ros.h>

#include "lsd_slam_msgs/keyframeMsg.h"
#include "lsd_slam_msgs/keyframeGraphMsg.h"

#include "texture_visual.h"
#include "buffer_visual.h"

namespace Ogre {
class SceneNode;
class Quaternion;
}

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
class QuaternionProperty;
}

namespace lsd_slam_rviz_plugins {

class KeyframeDisplay: public rviz::Display {
Q_OBJECT
public:

    KeyframeDisplay();
    virtual ~KeyframeDisplay();

	virtual void update( float wall_dt, float ros_dt );

protected:
	virtual void onInitialize();
	virtual void reset();
	void onEnable();
	void onDisable();
	virtual void fixedFrameChanged();

private Q_SLOTS:
	virtual void updateTopic();
    virtual void updateCamMarker();
	virtual void updateDepthSubsample();
    virtual void updateTriangleSideTH();
    virtual void updateScaledDepthVarTH();
//    virtual void updateAbsDepthVarTH();
    virtual void updateMinNearSupp();
    virtual void updateOrientation();

private:

    void processMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg);


	void subscribe();
	void unsubscribe();

	// ROS image subscription & synchronization
    ros::Subscriber liveframes_sub_; // keyframes
	boost::mutex cam_info_mutex_;

    rviz::IntProperty*          depth_subsample_property_;
    rviz::FloatProperty*        triangle_side_threshold_property_;
    rviz::RosTopicProperty*     pc_topic_property_;
    rviz::BoolProperty*         cam_marker_visible_property_;
    rviz::FloatProperty*        scaledDepthVarTH_property_;
    rviz::FloatProperty*        absDepthVarTH_property_;
    rviz::IntProperty*          minNearSupp_property_;
    rviz::QuaternionProperty*   orientation_property_;


	u_int32_t depth_subsample_;
	u_int32_t messages_received_;
    u_int32_t displayed_points_;
	float triangle_side_threshold_;
    bool cam_marker_visible_;
    float scaledDepthVarTH_;
    float absDepthVarTH_;
    int minNearSupp_;
    Ogre::Quaternion orientation_;

    KeyframeVisual::Ptr kfv;

};

} // end namespace lsd_slam_rviz_plugins

#endif // DEPTH_MESH_DISPLAY_H
