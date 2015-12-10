// point cloud base class
// author: bengrzimek@online.de

#include <rviz/ogre_helpers/arrow.h>
#include <ros/package.h>

#include "keyframe_visual.h"
#include "ros_depth_texture.h" // InputPointDense

//#include <ros/ros.h>
//#include <tf/transform_broadcaster.h>

namespace lsd_slam_rviz_plugins {


KeyframeVisual::KeyframeVisual(Ogre::SceneManager *sceneManager, Ogre::SceneNode *parentNode, rviz::Display *display) {
    sceneManager_ = sceneManager;
    if ( !parentNode )
    {
        parentNode = sceneManager_->getRootSceneNode();
    }

    static uint32_t count = 0;
    std::stringstream ss;
    ss << "PointCloud" << count++;
    name_ = ss.str();

    sceneNode_ = parentNode->createChildSceneNode(name_);
    display_ = display;

    // add shader location if shader does not exist
    if(!Ogre::MaterialManager::getSingleton().resourceExists("Test/Basic")) {

        std::string shaders_path = ros::package::getPath(
                                       "lsd_slam_rviz_plugins") + "/shaders/";

        Ogre::ResourceGroupManager::getSingleton().createResourceGroup(
            "KeyframeVisual");
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            shaders_path, "FileSystem", "KeyframeVisual");
        Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(
            "KeyframeVisual");
    }

    cameraVisual_ = new CameraVisual(sceneManager_, sceneNode_);
}


KeyframeVisual::~KeyframeVisual() {

    delete cameraVisual_;

//     ROS_INFO_STREAM("Deleting SceneNode " << sceneNode_->getName()  << " with objects: ");
//     Ogre::SceneNode::ObjectIterator it = sceneNode_->getAttachedObjectIterator();
//     while (it.hasMoreElements()) {
//         Ogre::MovableObject* node = it.getNext();
//         Ogre::String name = node->getName();
//         ROS_INFO_STREAM("--> " << name);
//     }

    //sceneNode_->detachAllObjects();
    if (sceneNode_) {
	sceneManager_->destroySceneNode(sceneNode_);
	sceneNode_ = nullptr;
    }
}


// interprets pointcloud-message and sets frame pose and camera parameter
void KeyframeVisual::setFrom(const lsd_slam_msgs::keyframeMsgConstPtr msg)
{
    // copy over campose.
    Sophus::Sim3f camToWorld;
    memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
    setFramePose(camToWorld);

    // camera intrinsic
    Ogre::Matrix4 unproj = Ogre::Matrix4( msg->fx, 0.0f,    msg->cx, 0.0f,
                                          0.0f,    msg->fy, msg->cy, 0.0f,
                                          0.0f,    0.0f,    1.0f,    0.0f,
                                          0.0f,    0.0f,    0.0f,    1.0f  ).inverse();
    setUnprojectionMatrix(unproj);
    setImageSize(msg->width, msg->height);

    cameraVisual_->setValues(msg->width, msg->height, msg->fx, msg->fy, msg->cx, msg->cy);

    // check if each pixel has an associated depth value
    if(msg->pointcloud.size() != msg->width*msg->height*sizeof(InputPointDense))
    {
        if(msg->pointcloud.size() != 0)
        {
            ROS_INFO("WARNING: PC with points, but number of points not right! (is %zu, should be %lu*%dx%d=%lu)\n",
                     msg->pointcloud.size(), sizeof(InputPointDense), msg->width, msg->height, msg->width*msg->height*sizeof(InputPointDense));
        }
    }
}


void KeyframeVisual::update() {
    cameraVisual_->update();
}


void KeyframeVisual::setFramePose(const Sophus::Sim3f& pose) {
    Sophus::Vector3f t = pose.translation();
    Sophus::Quaternionf o = pose.quaternion();
    float s = pose.scale();

    sceneNode_->setPosition(t(0), t(1), t(2));
    sceneNode_->setOrientation(o.w(), o.x(),o.y(),o.z());
    sceneNode_->setScale(s, s, s);
}


void KeyframeVisual::setCameraVisibility(bool isVisible) {
    cameraVisual_->setVisible( isVisible);
}


} // end namespace lsd_slam_rviz_plugins
