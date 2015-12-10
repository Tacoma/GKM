// pointcloud object rendering unprojected vertices from a VertexBuffer
// with Ogre::Mesh / Ogre::Entity
// author: bengrzimek@online.de

#ifndef BUFFER_VISUAL_H
#define BUFFER_VISUAL_H

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMesh.h>
#include <OGRE/OgreSubMesh.h>

#include "keyframe_visual.h"

namespace lsd_slam_rviz_plugins {


class BufferVisual : public KeyframeVisual {

public:
    BufferVisual(Ogre::SceneManager *sceneManager, Ogre::SceneNode *parentNode, rviz::Display *display);
    ~BufferVisual();

    void init(float scaledDepthVarTH = 0.001, float absDepthVarTH = 0.1,
                  int minSupp = 7, int depthSubsampleStep = 1, bool isCameraVisible = true, bool deleteOriginalMsg = false);

    void setFrom(const lsd_slam_msgs::keyframeMsgConstPtr msg);
    void update();
    void updateVertexPositionsAndColors(int size, float *points, Ogre::RGBA *colors); // helper for update, handles buffers

    void setImageSize(int width, int height);
    void setUnprojectionMatrix(const Ogre::Matrix4& unproj);
    void setDepthSubsample(int subsample);
    void setTriangleSideTH(float threshold);
    void setScaledDepthVarTH(float threshold);
    void setAbsDepthVarTH(float threshold);
    void setMinNearSupp(int support);
    void setDeleteOriginalMsg(bool deleteOriginalMsg);

private:
    Ogre::MeshPtr mesh_;
    Ogre::Entity *entity_;
    Ogre::HardwareVertexBufferSharedPtr vbuf_; // vertex buffer
    Ogre::HardwareVertexBufferSharedPtr cbuf_; // color buffer

    Ogre::Matrix4 unprojection_;
    int depthSubsample_;

    // buffer variables - analogue to ros_depth_texture
    lsd_slam_msgs::keyframeMsgConstPtr currentMsg_;
    int width_, height_;
    bool newMsg_;
    bool deleteOriginalMsg_;

    float scaledTH_;
    float absTH_;
    int minNearSupp_;


};


} // end namespace lsd_slam_rviz_plugins

#endif // BUFFER_VISUAL_H
