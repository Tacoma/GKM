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


struct GraphConstraint
{
    int from;
    int to;
    float err;
};

struct GraphConstraintPt
{
    KeyframeVisual::Ptr from;
    KeyframeVisual::Ptr to;
    float err;
};

struct GraphFramePose
{
    int id;
    float camToWorld[7];
};



//--- KeyframeGraphDisplay ---//

class KeyframeGraphDisplay: public rviz::Display {
    Q_OBJECT
public:

    KeyframeGraphDisplay();
    virtual ~KeyframeGraphDisplay();

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
    virtual void updateOrientation();
    virtual void updateDepthSubsample();
    virtual void updateTriangleSideThreshold();
    virtual void updateScaledDepthVarTH();
    virtual void updateAbsDepthVarTH();
    virtual void updateMinNearSupp();
    virtual void printInformation();
    virtual void updateDeleteOriginalMsgs();

private:
    void printSceneNodes(Ogre::Node* node, int depth);

    void processMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg);
    void processGraphMessage(const lsd_slam_msgs::keyframeGraphMsgConstPtr msg);

    void subscribe();
    void unsubscribe();

    // ROS image subscription & synchronization
    ros::Subscriber liveframes_sub_; // keyframes messages
    ros::Subscriber graph_sub_;      // graph messages
    ros::Subscriber oculus_cam_sub_; // live frame messages

    // ROS properties
    rviz::IntProperty*      depth_subsample_property_;
    rviz::RosTopicProperty* pc_topic_property_;
    rviz::RosTopicProperty* graph_topic_property_;
    rviz::RosTopicProperty* oculus_cam_follow_topic_property_;
    rviz::BoolProperty*     cam_marker_visible_property_;
    rviz::FloatProperty*    scaledDepthVarTH_property_;
    rviz::IntProperty*      minNearSupp_property_;
    rviz::QuaternionProperty* orientation_property_;
    rviz::BoolProperty*     button_property_;
    rviz::BoolProperty*	    delete_original_msgs_property_;

    // dead code / delete
    rviz::FloatProperty*    triangle_side_threshold_property_;
    rviz::FloatProperty*    absDepthVarTH_property_;
    float absDepthVarTH_;
    float triangle_side_threshold_;
    
    u_int32_t depth_subsample_;
    u_int32_t messages_received_;
    u_int32_t displayed_points_;
    int last_frame_id;
    bool cam_marker_visible_;
    float scaledDepthVarTH_;
    bool delete_original_msgs_;
    
    int minNearSupp_;
    Ogre::Quaternion orientation_;

private:
    // Graph data
    std::map<int, KeyframeVisual::Ptr> keyframe_visual_map_;
    std::queue<lsd_slam_msgs::keyframeMsgConstPtr> msg_queue_;
    boost::mutex keyframe_mutex_;

//    std::vector<GraphConstraintPt> constraints;

};

} // end namespace lsd_slam_rviz_plugins

#endif // DEPTH_MESH_DISPLAY_H
