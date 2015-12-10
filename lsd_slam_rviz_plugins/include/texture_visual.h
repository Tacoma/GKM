// pointcloud object rendering a depth texture and color texture, unprojection on the shader
// Uses Ogre::SimpleRenderable /
// author: bengrzimek@online.de

#ifndef TEXTURE_VISUAL_H
#define TEXTURE_VISUAL_H

#include <OGRE/OgreManualObject.h>

#include "keyframe_visual.h"
#include "ros_depth_texture.h"

namespace lsd_slam_rviz_plugins {


//---------- TextureRenderable ----------//
class TextureVisual;

class TextureRenderable: public Ogre::SimpleRenderable {
public:
    TextureRenderable(TextureVisual *textureVisual);

    ~TextureRenderable();

    virtual Ogre::Real getBoundingRadius(void) const;
    virtual Ogre::Real getSquaredViewDepth(const Ogre::Camera* cam) const;
    virtual unsigned short getNumWorldTransforms() const {
        return 1;
    }
    virtual void getWorldTransforms(Ogre::Matrix4* xform) const;
    void setVertexCount(int vc);

private:
    TextureVisual * textureVisual_;

};


//---------- TextureVisual ----------//
class TextureVisual : public KeyframeVisual {

public:
    TextureVisual(Ogre::SceneManager *sceneManager, Ogre::SceneNode *parentNode, rviz::Display *display);
    ~TextureVisual();

    void init(float scaledDepthVarTH = 0.001, float absDepthVarTH = 0.1,
                  int minSupp = 7, int depthSubsampleStep = 1, bool isCameraVisible = true, bool deleteOriginalMsg = false);

    void setFrom(const lsd_slam_msgs::keyframeMsgConstPtr msg);
    void update();

    void setImageSize(int width, int height);
    void setUnprojectionMatrix(const Ogre::Matrix4& unproj);
    void setDepthSubsample(int subsample);
    void setTriangleSideTH(float threshold);
    void setScaledDepthVarTH(float threshold);
    void setAbsDepthVarTH(float threshold);
    void setMinNearSupp(int support);
    void setDeleteOriginalMsg(bool deleteOriginalMsg);

    void getWorldTransforms(Ogre::Matrix4* xform) const; // forward pose to TextureRenderable

private:
    TextureRenderable *textureRenderable_;
    ROSDepthTexture depthTexture_;
    ROSDepthTexture rgbTexture_;

    int step_;
};


} // end namespace lsd_slam_rviz_plugins

#endif // TEXTURE_VISUAL_H
