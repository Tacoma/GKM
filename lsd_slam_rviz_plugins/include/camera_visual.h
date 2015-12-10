#ifndef CAMERA_VISUAL_H
#define CAMERA_VISUAL_H

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>

namespace lsd_slam_rviz_plugins {

class CameraVisual {

public:
    CameraVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode);

    ~CameraVisual();

    void setValues(float width, float height, float fx, float fy, float cx, float cy);
    void setVisible(bool isVisible);
    void update();

private:
    std::string name_;
    bool dirty_;

    float width_, height_;
    float fx_, fy_, cx_, cy_;

    Ogre::ManualObject *cameraDummy_;
    Ogre::SceneManager *sceneManager_;
    Ogre::SceneNode *sceneNode_;

};


} // end namespace lsd_slam_rviz_plugins

#endif // CAMERA_VISUAL_H
