//single pointcloud display from lsd_slam/keyframe messages

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMatrix4.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/quaternion_property.h>


#include "keyframe_display.h"

namespace lsd_slam_rviz_plugins {

KeyframeDisplay::KeyframeDisplay() :
    depth_subsample_(1), messages_received_(0), displayed_points_(0), triangle_side_threshold_(0.05),
    cam_marker_visible_(true), scaledDepthVarTH_(0.001), absDepthVarTH_(0.1), minNearSupp_(7) {

    //TODO: include original messages from lsd_slam_msgs instead of using copy (rosmake->catkin)
    // and change "lsd_slam_msgs/keyframeMsg" to << ros::message_traits::datatype<lsd_slam_msgs::keyframeMsg>() >>
    pc_topic_property_ =
        new rviz::RosTopicProperty("Pointcloud Topic", "",
                                   QString::fromStdString("lsd_slam_msgs/keyframeMsg"),
                                   "lsd_slam/keyframe topic to subscribe to", this,
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


//     triangle_side_threshold_property_ =
//         new rviz::FloatProperty("Triangle side threshold", triangle_side_threshold_,
//             "Only used for mesh display\n"
//             "Range: [0, 1]\n"
//             "Advanced: If triangle side exceeds this threshold the triangle is discarded.",
//             this, SLOT(updateTriangleSideTH()));
//         triangle_side_threshold_property_->setMin(0);
//         triangle_side_threshold_property_->setMax(1);

    scaledDepthVarTH_property_ =
        new rviz::FloatProperty("Scaled Threshold", log10(scaledDepthVarTH_),
                                "Scaled depth variance display threshold. Only points with a variance less than exp(ScaledThreshold) are displayed. Range between -10 and 1.",
                                this, SLOT(updateScaledDepthVarTH()));
    scaledDepthVarTH_property_->setMin(-10);
    scaledDepthVarTH_property_->setMax(1);

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

}

void KeyframeDisplay::onInitialize() {
    /*Ogre::StringVector groups = Ogre::ResourceGroupManager::getSingleton().getResourceGroups();
    for (Ogre::StringVector::iterator it = groups.begin(); it != groups.end(); it++) {
        ROS_INFO_STREAM(*it);
    }
    Ogre::ResourceGroupManager::ResourceGroup resource = Ogre::ResourceGroupManager::getSingleton().getResourceGroup("keyframeVisual");
    for (Ogre::ResourceGroupManager::ResourceGroup::iterator it = resource.begin(); it != resource.end(); it++)
    {
        ROS_INFO_STREAM(*it);
    }*/

    kfv.reset(new BufferVisual(context_->getSceneManager(), scene_node_, this));
    kfv->init(scaledDepthVarTH_, absDepthVarTH_, minNearSupp_, depth_subsample_, cam_marker_visible_);
}

KeyframeDisplay::~KeyframeDisplay() {
    reset();
    // has to be inverse order of creation (otherwise segmentation fault)
    delete minNearSupp_property_;
    //delete absDepthVarTH_property_;
    delete scaledDepthVarTH_property_;
    //delete triangle_side_threshold_property_;
    delete depth_subsample_property_;
    delete orientation_property_;
    delete cam_marker_visible_property_;
    delete pc_topic_property_;
}

void KeyframeDisplay::reset() {
    messages_received_ = 0;
    displayed_points_ = 0;
    setStatus(rviz::StatusProperty::Ok, "Message", "Ok");
}

void KeyframeDisplay::onEnable() {
    subscribe();
    kfv->setDepthSubsample(depth_subsample_);
    kfv->setTriangleSideTH(triangle_side_threshold_);
    kfv->setScaledDepthVarTH(scaledDepthVarTH_);
    kfv->setAbsDepthVarTH(absDepthVarTH_);
    kfv->setMinNearSupp(minNearSupp_);

}

void KeyframeDisplay::onDisable() {
    unsubscribe();
}

//TODO: change to lsd_slam_msgs:: once the rosmake->catkin include works
void KeyframeDisplay::processMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg) {
    if (!msg->isKeyframe) {
        // logic for live frames
    }
    else {
        kfv->setFrom(msg);
        displayed_points_= msg->width*msg->height;
        messages_received_++;
    }
    setStatus(rviz::StatusProperty::Ok, "Keyframe Positions",
              QString::number(messages_received_)
              + " keyframe positions received");
    setStatus(rviz::StatusProperty::Ok, "Message", "Ok");
    setStatus(rviz::StatusProperty::Ok, "Displayed Points", QString::number(displayed_points_/depth_subsample_));

}

void KeyframeDisplay::updateTopic() {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender(); // request new render
}

void KeyframeDisplay::updateCamMarker() {
    cam_marker_visible_ = cam_marker_visible_property_->getBool();
    kfv->setCameraVisibility( cam_marker_visible_ );
}

void KeyframeDisplay::updateOrientation() {
    Ogre::Quaternion orientation = orientation_property_->getQuaternion();
    scene_node_->setOrientation(orientation);
}

void KeyframeDisplay::updateDepthSubsample() {
    depth_subsample_ = depth_subsample_property_->getInt();
    setStatus(rviz::StatusProperty::Ok, "Displayed Points", QString::number(displayed_points_/depth_subsample_));
    kfv->setDepthSubsample(depth_subsample_);
}

void KeyframeDisplay::updateTriangleSideTH() {
    triangle_side_threshold_ = triangle_side_threshold_property_->getFloat();
    kfv->setTriangleSideTH(triangle_side_threshold_);
}

void KeyframeDisplay::updateScaledDepthVarTH() {
    scaledDepthVarTH_ = scaledDepthVarTH_property_->getFloat();
    scaledDepthVarTH_ = pow(10.0f,scaledDepthVarTH_);
    kfv->setScaledDepthVarTH(scaledDepthVarTH_);
}

//void KeyframeDisplay::updateAbsDepthVarTH() {
//    absDepthVarTH_ = absDepthVarTH_property_->getFloat();
//    absDepthVarTH_ = pow(10.0f,absDepthVarTH_);
//    kfv->setAbsDepthVarTH(absDepthVarTH_);
//}

void KeyframeDisplay::updateMinNearSupp() {
    minNearSupp_ = minNearSupp_property_->getInt();
    kfv->setMinNearSupp(minNearSupp_);
}

void KeyframeDisplay::subscribe() {
    if (!isEnabled()) {
        return;
    }

    try {
        std::string pc_topic = pc_topic_property_->getTopicStd();
        if (!pc_topic.empty()) {
            //TODO: use boost version of callback
            liveframes_sub_ = threaded_nh_.subscribe(pc_topic,1, &KeyframeDisplay::processMessage, this);
        }
    } catch (ros::Exception& e) {
        setStatus(rviz::StatusProperty::Error, "Message",
                  QString("Error subscribing: ") + e.what());
    }
}

void KeyframeDisplay::unsubscribe() {
    try {
        liveframes_sub_.shutdown();
    } catch (ros::Exception& e) {
        setStatus(rviz::StatusProperty::Error, "Message",
                  QString("Error unsubscribing: ") + e.what());
    }
}

void KeyframeDisplay::fixedFrameChanged() {
    Display::reset();
}

void KeyframeDisplay::update(float wall_dt, float ros_dt) {
    kfv->update();
}


} // end namespace lsd_slam_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lsd_slam_rviz_plugins::KeyframeDisplay,
                       rviz::Display)

