//new pointcloud graph display

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMatrix4.h>

//#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>

#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/quaternion_property.h>
//#include <rviz/properties/color_property.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "sophus/sim3.hpp"

#include "keyframe_graph_display.h"

namespace lsd_slam_rviz_plugins {

KeyframeGraphDisplay::KeyframeGraphDisplay() :
    messages_received_(0),
    displayed_points_(0),
    last_frame_id(0),
    cam_marker_visible_(true),
    depth_subsample_(1),
    triangle_side_threshold_(0.05),
    scaledDepthVarTH_(0.001),
    absDepthVarTH_(0.1),
    minNearSupp_(7),
    delete_original_msgs_(false) {


    //TODO: include original messages from lsd_slam_msgs instead of using copy (rosmake->catkin)
    // and change "lsd_slam_msgs/keyframeMsg" to << ros::message_traits::datatype<lsd_slam_msgs::keyframeMsg>() >>
    pc_topic_property_ =
        new rviz::RosTopicProperty("Pointcloud Topic", "",
                                   QString::fromStdString("lsd_slam_msgs/keyframeMsg"),
                                   "lsd_slam/keyframe topic to subscribe to", this,
                                   SLOT(updateTopic()));

    oculus_cam_follow_topic_property_ =
        new rviz::RosTopicProperty("Oculus camera topic", "",
                                   QString::fromStdString("lsd_slam_msgs/keyframeMsg"),
                                   "lsd_slam/liveframes topic to in order to publish the camera position as tf-frame \"camera\".", this,
                                   SLOT(updateTopic()));

    graph_topic_property_ =
        new rviz::RosTopicProperty("Graph Topic", "",
                                   QString::fromStdString("lsd_slam_msgs/keyframeGraphMsg"),
                                   "lsd_slam/graph topic to subscribe to", this,
                                   SLOT(updateTopic()));

    cam_marker_visible_property_ =
        new rviz::BoolProperty("Display camera", cam_marker_visible_, "Display camera dummy at the pose of each keyframe.", this, SLOT(updateCamMarker()));

    orientation_property_ =
        new rviz::QuaternionProperty("Orientation", Ogre::Quaternion::IDENTITY,
                                     "Quaternion rotating the whole scene, is normalized before application.",
                                     this, SLOT(updateOrientation()));

    depth_subsample_property_ =
        new rviz::IntProperty("Depth Subsample", depth_subsample_,
                              "Render only every nth point, usefull to speed up large pointclouds. Range between 1 and 8.",
                              this, SLOT(updateDepthSubsample()));
    depth_subsample_property_->setMin(1);
    depth_subsample_property_->setMax(8);

//     // Used for old mesh based rendering - delete
//     triangle_side_threshold_property_ =
//         new rviz::FloatProperty("Triangle side threshold", triangle_side_threshold_,
//                                 "Only used for mesh display\n"
//                                 "Range: [0, 1]\n"
//                                 "Advanced: If triangle side exceeds this threshold the triangle is discarded.",
//                                 this, SLOT(updateTriangleSideThreshold()));
//     triangle_side_threshold_property_->setMin(0);
//     triangle_side_threshold_property_->setMax(1);

    scaledDepthVarTH_property_ =
        new rviz::FloatProperty("Scaled Threshold", log10(scaledDepthVarTH_),
                                "Scaled depth variance display threshold. Only points with a variance less than exp(ScaledThreshold) are displayed. Range between -10 and 1.",
                                this, SLOT(updateScaledDepthVarTH()));
    scaledDepthVarTH_property_->setMin(-10);
    scaledDepthVarTH_property_->setMax(1);

//    // TODO: add absolute depth variance threshold
//    absDepthVarTH_property_ =
//        new rviz::FloatProperty("Absolute Threshold", log10(absDepthVarTH_),
//            "Absolute depth variance display threshold\n"
//            "Range: [-10, 1]\n"
//            "Advanced: log 10 of point's variance, in absolute scale",
//            this, SLOT(updateAbsDepthVarTH()));
//        absDepthVarTH_property_->setMin(-10);
//        absDepthVarTH_property_->setMax(1);

    minNearSupp_property_ =
        new rviz::IntProperty("Minimum Near Support", minNearSupp_,
                              "Only displays points with this many neighbors close to it (closer than twice its variance). Range between 0 and 9.",
                              this, SLOT(updateMinNearSupp()));
    minNearSupp_property_->setMin(0);
    minNearSupp_property_->setMax(9);

    delete_original_msgs_property_ =
        new rviz::BoolProperty("Delete Original Pointcloud", delete_original_msgs_,
                               "Delete original pointcloud message after creation to save space. If used, all properties (Depth Subsample etc.) "
                               "only apply to new messages.",
                               this, SLOT(updateDeleteOriginalMsgs()));

    button_property_ =
        new rviz::BoolProperty("Information", false, "Displays and prints debug information.", this, SLOT(printInformation()));

}

void KeyframeGraphDisplay::onInitialize() {
}

KeyframeGraphDisplay::~KeyframeGraphDisplay() {
    reset();
    // has to be inverse order of creation (otherwise segmentation fault)
    delete button_property_;
    delete delete_original_msgs_property_;
    delete minNearSupp_property_;
//    delete absDepthVarTH_property_;
    delete scaledDepthVarTH_property_;
//    delete triangle_side_threshold_property_;
    delete depth_subsample_property_;
    delete orientation_property_;
    delete cam_marker_visible_property_;
    delete graph_topic_property_;
    delete pc_topic_property_;
}

void KeyframeGraphDisplay::reset() {
    //boost::mutex::scoped_lock lock(keyframe_mutex_);
    OGRE_LOCK_MUTEX(context_->getSceneManager()->sceneGraphMutex);
    Display::reset();
    messages_received_ = 0;
    displayed_points_ = 0;
    keyframe_visual_map_.clear();
    while(!msg_queue_.empty()) {
        msg_queue_.pop();
    }
    setStatus(rviz::StatusProperty::Ok, "Message", "Ok");
}

void KeyframeGraphDisplay::onEnable() {
    subscribe();
    for (auto& kfv : keyframe_visual_map_) {
        kfv.second->setDepthSubsample(depth_subsample_);
        kfv.second->setTriangleSideTH(triangle_side_threshold_);
        kfv.second->setScaledDepthVarTH(scaledDepthVarTH_);
        kfv.second->setAbsDepthVarTH(absDepthVarTH_);
        kfv.second->setMinNearSupp(minNearSupp_);
    }
}

void KeyframeGraphDisplay::onDisable() {
    unsubscribe();
}

//TODO: change to lsd_slam_msgs:: once the rosmake->catkin include works
void KeyframeGraphDisplay::processMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg) {

    if (context_->getFrameManager()->getPause()) {
        return;
    }

    if (msg->isKeyframe) {
        // add message to queue
        if (keyframe_visual_map_.count(msg->id) == 0) {
            msg_queue_.push(msg);
            displayed_points_+= msg->width*msg->height;
            messages_received_++;
        }
    } else {
        // check for reset
        if (last_frame_id > msg->id) {
            ROS_INFO_STREAM("detected backward-jump in id (" << last_frame_id << " to " << msg->id << "), resetting!");
            reset();
        }
        last_frame_id = msg->id;

        // TBD broadcast new transform for oculus or call methods in a new oculus cam class
        // copy data form msg into variable pose
        Sophus::Sim3f pose;
        memcpy(pose.data(), msg->camToWorld.data(), 7*sizeof(float));

        // get translation, rotation and scale from pose
        Sophus::Vector3f t = pose.translation();
        Sophus::Quaternionf o = pose.quaternion();

        // get rviz orientation from property and convert to tf quaternion for simpler multiplication
        Ogre::Quaternion rviz_o = orientation_property_->getQuaternion();
        tf::Quaternion rviz_rot = tf::Quaternion(rviz_o.x, rviz_o.y, rviz_o.z, rviz_o.w);

        // TBD if the init of broadcaster has a bit overhead, pull out the tf broadcaster into header
        // broadcast pose as tf
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        // apply rviz rot to translation and rotation
        transform.setOrigin( tf::Matrix3x3(rviz_rot) * tf::Vector3(t(0),t(1),t(2)) ); // y and z is switched in tf vs sophus
        tf::Quaternion rot = rviz_rot * tf::Quaternion(o.x(),o.y(),o.z(),o.w());
        transform.setRotation( rot );

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera"));
    }

    setStatus(rviz::StatusProperty::Ok, "Keyframe Positions", QString::number(messages_received_) + " keyframe positions received");
    setStatus(rviz::StatusProperty::Ok, "Message", "Ok");
    setStatus(rviz::StatusProperty::Ok, "Displayed Points (~)", QString::number(displayed_points_/depth_subsample_));
    setStatus(rviz::StatusProperty::Ok, "Last Message ID", QString::number(msg->id));
}


void KeyframeGraphDisplay::processGraphMessage(const lsd_slam_msgs::keyframeGraphMsgConstPtr msg) {

    // copy graph constraints into constraints
//    constraints.resize(msg->numConstraints);
//    assert(msg->constraintsData.size() == sizeof(GraphConstraint)*msg->numConstraints);
//    GraphConstraint* constraintsIn = (GraphConstraint*)msg->constraintsData.data();
//    for(int i=0;i<msg->numConstraints;i++)
//    {
//        constraints[i].err = constraintsIn[i].err;
//        constraints[i].from = 0;
//        constraints[i].to = 0;

//        if(keyframe_visual_map_.count(constraintsIn[i].from) != 0) {
//            constraints[i].from = keyframe_visual_map_[constraintsIn[i].from];
//        }

//        if(keyframe_visual_map_.count(constraintsIn[i].to) != 0) {
//            constraints[i].to = keyframe_visual_map_[constraintsIn[i].to];
//        }
//    }


    // apply constraint poses
    GraphFramePose* graphPoses = (GraphFramePose*)msg->frameData.data();
    int numGraphPoses = msg->numFrames;
    assert(msg->frameData.size() == sizeof(GraphFramePose)*msg->numFrames);

    if (numGraphPoses > 0) {
        //boost::mutex::scoped_lock lock(keyframe_mutex_);
        OGRE_LOCK_MUTEX(context_->getSceneManager()->sceneGraphMutex);
        for(int i=0; i<numGraphPoses; i++)
        {
            if(keyframe_visual_map_.count(graphPoses[i].id) != 0)
            {
                // copy over campose.
                Sophus::Sim3f camToWorld;
                memcpy(camToWorld.data(), graphPoses[i].camToWorld, 7*sizeof(float));
                keyframe_visual_map_[graphPoses[i].id]->setFramePose(camToWorld);
            }
        }
    } // Lock
}

void KeyframeGraphDisplay::updateTopic() {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender(); // request new render
}

void KeyframeGraphDisplay::updateCamMarker() {
    cam_marker_visible_ = cam_marker_visible_property_->getBool();
    for (auto& kfv : keyframe_visual_map_) {
        kfv.second->setCameraVisibility( cam_marker_visible_ );
    }
}

void KeyframeGraphDisplay::updateOrientation() {
    Ogre::Quaternion orientation = orientation_property_->getQuaternion();
    scene_node_->setOrientation(orientation);
}

void KeyframeGraphDisplay::updateDepthSubsample() {
    depth_subsample_ = depth_subsample_property_->getInt();
    setStatus(rviz::StatusProperty::Ok, "Displayed Points", QString::number(displayed_points_/depth_subsample_));
    for (auto& kfv : keyframe_visual_map_) {
        kfv.second->setDepthSubsample(depth_subsample_);
    }
}

// dead code / delete
void KeyframeGraphDisplay::updateTriangleSideThreshold() {
    triangle_side_threshold_ = triangle_side_threshold_property_->getFloat();
    for (auto& kfv : keyframe_visual_map_) {
        kfv.second->setTriangleSideTH(triangle_side_threshold_);
    }
}

void KeyframeGraphDisplay::updateScaledDepthVarTH() {
    scaledDepthVarTH_ = scaledDepthVarTH_property_->getFloat();
    scaledDepthVarTH_ = pow(10.0f,scaledDepthVarTH_);
    for (auto& kfv : keyframe_visual_map_) {
        kfv.second->setScaledDepthVarTH(scaledDepthVarTH_);
    }
}

// dead code / delete
void KeyframeGraphDisplay::updateAbsDepthVarTH() {
    absDepthVarTH_ = absDepthVarTH_property_->getFloat();
    absDepthVarTH_ = pow(10.0f,absDepthVarTH_);
    for (auto& kfv : keyframe_visual_map_) {
        kfv.second->setAbsDepthVarTH(absDepthVarTH_);
    }
}

void KeyframeGraphDisplay::updateMinNearSupp() {
    minNearSupp_ = minNearSupp_property_->getInt();
    for (auto& kfv : keyframe_visual_map_) {
        kfv.second->setMinNearSupp(minNearSupp_);
    }
}

void KeyframeGraphDisplay::updateDeleteOriginalMsgs() {
    delete_original_msgs_ = delete_original_msgs_property_->getBool();
    // can't get deleted messages back
    if (delete_original_msgs_) {
        for (auto& kfv : keyframe_visual_map_) {
            kfv.second->setDeleteOriginalMsg(delete_original_msgs_);
        }
    }
}

void KeyframeGraphDisplay::printInformation() {
    bool info = button_property_->getBool();
    Ogre::SceneManager* sm = context_->getSceneManager();
    //sm->setDisplaySceneNodes(info);
    sm->showBoundingBoxes(info);

    if (info) {
        ROS_INFO_STREAM("Scene Manager: \n\tName: " << sm->getName() << "\n\tType: " << sm->getTypeName());
        printSceneNodes(sm->getRootSceneNode(), 0);
    }
}

void KeyframeGraphDisplay::printSceneNodes(Ogre::Node* node, int depth) {
    Ogre::SceneNode::ChildNodeIterator it = node->getChildIterator();

    std::stringstream ss;
    for (int i = 0; i < depth; i++) {
        ss << " ";
    }
    std::string space = ss.str();

    while (it.hasMoreElements()) {
        Ogre::Node* node = it.getNext();
        Ogre::String name = node->getName();
        ROS_INFO_STREAM(space << name);
        printSceneNodes(node, depth+1);
    }

}

void KeyframeGraphDisplay::subscribe() {
    if (!isEnabled()) {
        return;
    }
    try {
        std::string pc_topic = pc_topic_property_->getTopicStd();
        if (!pc_topic.empty()) {
            //TODO: use boost version of callback
            liveframes_sub_ = threaded_nh_.subscribe(pc_topic, 1, &KeyframeGraphDisplay::processMessage, this);
        }

        std::string oculus_cam_topic = oculus_cam_follow_topic_property_->getTopicStd();
        if (!oculus_cam_topic.empty()) {
            oculus_cam_sub_ = threaded_nh_.subscribe(oculus_cam_topic, 1, &KeyframeGraphDisplay::processMessage, this);
        }

        std::string graph_topic = graph_topic_property_->getTopicStd();
        if (!graph_topic.empty()) {
            graph_sub_ = threaded_nh_.subscribe(graph_topic, 1, &KeyframeGraphDisplay::processGraphMessage, this);
        }
    } catch (ros::Exception& e) {
        setStatus(rviz::StatusProperty::Error, "Message",
                  QString("Error subscribing: ") + e.what());
    }
}

void KeyframeGraphDisplay::unsubscribe() {
    try {
        liveframes_sub_.shutdown();
        oculus_cam_sub_.shutdown();
        graph_sub_.shutdown();
    } catch (ros::Exception& e) {
        setStatus(rviz::StatusProperty::Error, "Message",
                  QString("Error unsubscribing: ") + e.what());
    }
}

void KeyframeGraphDisplay::fixedFrameChanged() {
    Display::reset();
}

void KeyframeGraphDisplay::update(float wall_dt, float ros_dt) {
    // Create new visuals from the queued  messages
    //boost::mutex::scoped_lock lock(keyframe_mutex_);
    OGRE_LOCK_MUTEX(context_->getSceneManager()->sceneGraphMutex);

    while(!msg_queue_.empty()) {
        lsd_slam_msgs::keyframeMsgConstPtr msg = msg_queue_.front();
        msg_queue_.pop();
        KeyframeVisual::Ptr visual( new BufferVisual(context_->getSceneManager(), scene_node_, this));
        visual->init(scaledDepthVarTH_, absDepthVarTH_, minNearSupp_, depth_subsample_, cam_marker_visible_, delete_original_msgs_);
        visual->setFrom(msg);
        keyframe_visual_map_[msg->id] = visual;
    }
    for (auto& kfv : keyframe_visual_map_) {
        kfv.second->update();
    }
}

} // end namespace lsd_slam_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lsd_slam_rviz_plugins::KeyframeGraphDisplay,
                       rviz::Display)

