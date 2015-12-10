#ifndef KEYFRAME_VISUAL_H
#define KEYFRAME_VISUAL_H

#include <ros/ros.h>
#include <rviz/display.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "sophus/sim3.hpp"
#include "lsd_slam_msgs/keyframeMsg.h"
#include "camera_visual.h"


namespace lsd_slam_rviz_plugins {

//--- structures ---//

struct InputDepth
{
    float depth;
};

struct InputRGB
{
    unsigned char color[4];
};

//--- KeyframeVisual ---//
class KeyframeVisual {
	
public:
    typedef boost::shared_ptr<KeyframeVisual> Ptr;

    KeyframeVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode, rviz::Display *display);
    virtual ~KeyframeVisual();

    virtual void init(float scaledDepthVarTH = 0.001, float absDepthVarTH = 0.1,
                  int minSupp = 7, int depthSubsampleStep = 1, bool isCameraVisible = true, bool deleteOriginalMsg = false) = 0;

    virtual void setFrom(const lsd_slam_msgs::keyframeMsgConstPtr msg);
    virtual void update();

    virtual void setImageSize(int width, int height) = 0;
    virtual void setUnprojectionMatrix(const Ogre::Matrix4& unproj) = 0;
    virtual void setDepthSubsample(int subsample) = 0;
    virtual void setTriangleSideTH(float threshold) = 0;
    virtual void setScaledDepthVarTH(float threshold) = 0;
    virtual void setAbsDepthVarTH(float threshold) = 0;
    virtual void setMinNearSupp(int support) = 0;
    virtual void setDeleteOriginalMsg(bool deleteOriginalMsg) = 0;

    void setFramePose(const Sophus::Sim3f& pose);
    void setCameraVisibility(bool isVisible);

    //void getWorldTransforms(Ogre::Matrix4* xform) const;

protected:
    //debug
    rviz::Display *display_;
    std::string name_;

    // camera parameter
    Sophus::Sim3f camToWorld_;
    CameraVisual* cameraVisual_;

    Ogre::SceneNode *sceneNode_;
    Ogre::SceneManager *sceneManager_;

};


} // end namespace lsd_slam_rviz_plugins

#endif // KEYFRAME_VISUAL_H
