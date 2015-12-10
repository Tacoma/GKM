#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "texture_visual.h"

namespace lsd_slam_rviz_plugins {


//---------- TextureVisual ----------//
TextureVisual::TextureVisual(Ogre::SceneManager *sceneManager, Ogre::SceneNode *parentNode, rviz::Display *display)
                : KeyframeVisual(sceneManager, parentNode, display), depthTexture_(false, display), rgbTexture_(true, display) {

    // clone material
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "Test/KeyFrameMat" << count++;
    std::string matName = ss.str();
    Ogre::MaterialPtr baseMaterial(
            Ogre::MaterialManager::getSingleton().getByName("Test/Basic"));
    baseMaterial->clone(matName);
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName(matName);

    // create Renderable with material
    textureRenderable_ = new TextureRenderable(this);
    textureRenderable_->setBoundingBox(Ogre::AxisAlignedBox(-10, -10, 0, 10, 10, 10));
    textureRenderable_->setMaterial(matName);

    // Add rgb-texture to material
    Ogre::TextureUnitState* tu = textureRenderable_->getMaterial()->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(rgbTexture_.getTexture()->getName());
    tu->setTextureFiltering(Ogre::TFO_NONE);
    //ROS_INFO_STREAM("MATERIAL: Binding " <<  rgbTexture_.getTexture()->getName()
    //                << " to " << textureRenderable_->getMaterial()->getName() << " texture unit: " << tu->getName());
    //display_->setStatus(rviz::StatusProperty::Ok,
    //    QString::fromStdString(rgbTexture_.getTexture()->getName()), QString::fromStdString(textureRenderable_->getMaterial()->getName()));

    // Add depth-texture to material
    tu = textureRenderable_->getMaterial()->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(depthTexture_.getTexture()->getName());
    tu->setTextureFiltering(Ogre::TFO_NONE);
    //ROS_INFO_STREAM("MATERIAL: Binding " <<  depthTexture_.getTexture()->getName()
    //                << " to " << textureRenderable_->getMaterial()->getName() << " texture unit: " << tu->getName());
    //display_->setStatus(rviz::StatusProperty::Ok,
    //   QString::fromStdString(depthTexture_.getTexture()->getName()), QString::fromStdString(textureRenderable_->getMaterial()->getName()));


    sceneNode_->attachObject(textureRenderable_);
    init();
}


TextureVisual::~TextureVisual() {
    delete textureRenderable_;
}


void TextureVisual::init(float scaledDepthVarTH, float absDepthVarTH,
              int minSupp, int depthSubsampleStep, bool isCameraVisible, bool deleteOriginalMsg) {
    setScaledDepthVarTH(scaledDepthVarTH);
    setAbsDepthVarTH(absDepthVarTH);
    setMinNearSupp(minSupp);
    setDepthSubsample(depthSubsampleStep);
    setCameraVisibility(isCameraVisible);
    setDeleteOriginalMsg(deleteOriginalMsg);
}


void TextureVisual::setFrom(const lsd_slam_msgs::keyframeMsgConstPtr msg) {
    KeyframeVisual::setFrom(msg);

    Ogre::GpuProgramParametersSharedPtr params;
    params = textureRenderable_->getMaterial()->getTechnique(0)->getPass(0)->getGeometryProgramParameters();
    params->setNamedConstant("depth_correction", 1.0f);

    rgbTexture_.addMessage(msg);
    depthTexture_.addMessage(msg);
}


void TextureVisual::update() {
    KeyframeVisual::update();
    rgbTexture_.update();
    depthTexture_.update();
}


void TextureVisual::setImageSize(int width, int height) {
    Ogre::GpuProgramParametersSharedPtr params;
    params =
            textureRenderable_->getMaterial()->getTechnique(0)->getPass(0)->getVertexProgramParameters();
    params->setNamedConstant("width", width);
    params->setNamedConstant("height", height);
    params =
            textureRenderable_->getMaterial()->getTechnique(0)->getPass(0)->getGeometryProgramParameters();
    params->setNamedConstant("width", width);
    params->setNamedConstant("height", height);

    textureRenderable_->setVertexCount(width * height / (step_ * step_));
}


void TextureVisual::setUnprojectionMatrix(const Ogre::Matrix4 & unproj) {
    Ogre::GpuProgramParametersSharedPtr params =
            textureRenderable_->getMaterial()->getTechnique(0)->getPass(0)->getGeometryProgramParameters();
    params->setNamedConstant("unproj", unproj);
}


void TextureVisual::setDepthSubsample(int subsample) {
    step_ = subsample;
    Ogre::GpuProgramParametersSharedPtr params;
    params =
            textureRenderable_->getMaterial()->getTechnique(0)->getPass(0)->getVertexProgramParameters();
    params->setNamedConstant("step", step_);
    params =
            textureRenderable_->getMaterial()->getTechnique(0)->getPass(0)->getGeometryProgramParameters();
    params->setNamedConstant("step", step_);

    textureRenderable_->setVertexCount(depthTexture_.getWidth()*depthTexture_.getHeight() / (step_ * step_));
}


void TextureVisual::setTriangleSideTH(float threshold) {
    Ogre::GpuProgramParametersSharedPtr params;
    params =
            textureRenderable_->getMaterial()->getTechnique(0)->getPass(0)->getGeometryProgramParameters();
    params->setNamedConstant("threshold2", threshold * threshold);
}


void TextureVisual::setScaledDepthVarTH(float threshold) {
    depthTexture_.setScaledDepthVarTH(threshold);
}


void TextureVisual::setAbsDepthVarTH(float threshold) {
    depthTexture_.setAbsDepthVarTH(threshold);
}


void TextureVisual::setMinNearSupp(int support) {
    depthTexture_.setMinNearSupp(support);
}

void TextureVisual::getWorldTransforms(Ogre::Matrix4* xform) const {
    *xform = sceneNode_->_getFullTransform();
}

void TextureVisual::setDeleteOriginalMsg(bool deleteOriginalMsg) {
    ROS_ERROR("setDeleteOriginalMsg() not implemented!"); //TODO: implement
}


//---------- TextureRenderable ----------//
TextureRenderable::TextureRenderable(TextureVisual * textureVisual) :
        textureVisual_(textureVisual) {
    mRenderOp.operationType = Ogre::RenderOperation::OT_POINT_LIST;
    mRenderOp.useIndexes = false;

    mRenderOp.vertexData = new Ogre::VertexData;
    mRenderOp.vertexData->vertexStart = 0;
    mRenderOp.vertexData->vertexCount = 0;

}

TextureRenderable::~TextureRenderable() {
    delete mRenderOp.vertexData;
}

void TextureRenderable::setVertexCount(int vc) {
    mRenderOp.vertexData->vertexCount = vc;
}

Ogre::Real TextureRenderable::getBoundingRadius(void) const {
    return Ogre::Math::Sqrt(
            std::max(mBox.getMaximum().squaredLength(),
                    mBox.getMinimum().squaredLength()));
}

Ogre::Real TextureRenderable::getSquaredViewDepth(
        const Ogre::Camera* cam) const {
    Ogre::Vector3 vMin, vMax, vMid, vDist;
    vMin = mBox.getMinimum();
    vMax = mBox.getMaximum();
    vMid = ((vMax - vMin) * 0.5) + vMin;
    vDist = cam->getDerivedPosition() - vMid;

    return vDist.squaredLength();
}

void TextureRenderable::getWorldTransforms(Ogre::Matrix4* xform) const {
    textureVisual_->getWorldTransforms(xform);
}



} // end namespace lsd_slam_rviz_plugins
