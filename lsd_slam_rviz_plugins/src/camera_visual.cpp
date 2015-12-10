
#include <OGRE/OgreSceneNode.h>

#include <ros/ros.h>
#include "camera_visual.h"


namespace lsd_slam_rviz_plugins {


CameraVisual::CameraVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode) :
        dirty_(false), width_(0.0f), height_(0.0f), fx_(0.0f), fy_(0.0f), cx_(0.0f), cy_(0.0f) {

    sceneManager_ = sceneManager;
    if ( !parentNode )
    {
      parentNode = sceneManager_->getRootSceneNode();
    }
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "Camera" << count++;
    name_ = ss.str();
    sceneNode_ = parentNode; //parentNode->createChildSceneNode(name);

    cameraDummy_ = sceneManager_->createManualObject(name_);

    if(!Ogre::MaterialManager::getSingleton().resourceExists("General/cameraMaterial")) {
        Ogre::MaterialPtr camera_mat = Ogre::MaterialManager::getSingleton().create("cameraMaterial","General");
        camera_mat->setReceiveShadows(false);
        camera_mat->getTechnique(0)->setLightingEnabled(true);
        camera_mat->getTechnique(0)->getPass(0)->setDiffuse(0,0,1,0);
        camera_mat->getTechnique(0)->getPass(0)->setAmbient(0,0,1);
        camera_mat->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,0);
    }

    cameraDummy_->estimateVertexCount(16);
    cameraDummy_->setDynamic(true);
    cameraDummy_->begin("cameraMaterial", Ogre::RenderOperation::OT_LINE_LIST);
    cameraDummy_->position(0, 0, 0);
    // lines filled in setFrom(...)
    cameraDummy_->end();
    sceneNode_->attachObject(cameraDummy_);
}

CameraVisual::~CameraVisual() {
    //sceneManager_->destroySceneNode(sceneNode_); // has the same node as pointcloud

//     ROS_INFO_STREAM("Deleting CameraVisual in SceneNode " << sceneNode_->getName()  << " with objects: ");
//     Ogre::SceneNode::ObjectIterator it = sceneNode_->getAttachedObjectIterator();
//     while (it.hasMoreElements()) {
//        Ogre::MovableObject* node = it.getNext();
//        Ogre::String name = node->getName();
//        ROS_INFO_STREAM("--> " << name);
//     }

    if (sceneNode_ && cameraDummy_) {
	sceneNode_->detachObject(cameraDummy_);
	sceneNode_->needUpdate();
	// TODO: Destroy cameraDummy_
	//sceneManager_->destroyManualObject(cameraDummy_->getName());
	//cameraDummy_ = nullptr;
    }

}

void CameraVisual::setValues(float width, float height, float fx, float fy, float cx, float cy) {
    width_ = width; height_ = height;
    fx_ = fx; fy_ = fy; cx_ = cx; cy_= cy;
    dirty_ = true;
}

void CameraVisual::update() {
    if (!dirty_) { return; }
    dirty_ = false;

    cameraDummy_->beginUpdate(0);
    float s =  0.05f;
    //frustrum
    cameraDummy_->position( 0, 0, 0 );
    cameraDummy_->position( s*(0-cx_)/fx_, s*(0-cy_)/fy_, s );
    cameraDummy_->position( 0, 0, 0 );
    cameraDummy_->position( s*(0-cx_)/fx_, s*(height_-1-cy_)/fy_, s );
    cameraDummy_->position( 0, 0, 0 );
    cameraDummy_->position( s*(width_-1-cx_)/fx_, s*(height_-1-cy_)/fy_, s );
    cameraDummy_->position( 0, 0, 0 );
    cameraDummy_->position( s*(width_-1-cx_)/fx_, s*(0-cy_)/fy_, s );
    // image plane
    cameraDummy_->position( s*(width_-1-cx_)/fx_, s*(0-cy_)/fy_, s );
    cameraDummy_->position( s*(width_-1-cx_)/fx_, s*(height_-1-cy_)/fy_, s );
    cameraDummy_->position( s*(width_-1-cx_)/fx_, s*(height_-1-cy_)/fy_, s );
    cameraDummy_->position( s*(0-cx_)/fx_, s*(height_-1-cy_)/fy_, s );
    cameraDummy_->position( s*(0-cx_)/fx_, s*(height_-1-cy_)/fy_, s );
    cameraDummy_->position( s*(0-cx_)/fx_, s*(0-cy_)/fy_, s );
    cameraDummy_->position( s*(0-cx_)/fx_, s*(0-cy_)/fy_, s );
    cameraDummy_->position( s*(width_-1-cx_)/fx_, s*(0-cy_)/fy_, s );
    cameraDummy_->end();
}

void CameraVisual::setVisible(bool isVisible) {
    cameraDummy_->setVisible(isVisible);
}


} // end namespace lsd_slam_rviz_plugins
