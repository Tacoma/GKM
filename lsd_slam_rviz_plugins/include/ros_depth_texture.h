#ifndef ROS_DEPTH_TEXTURE_H
#define ROS_DEPTH_TEXTURE_H

#include <ros/ros.h>

#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OgreTexture.h>

#include "lsd_slam_msgs/keyframeMsg.h"

#include <rviz/display.h>


namespace lsd_slam_rviz_plugins {

//--- structures ---//

struct InputPointDense
{
    float idepth;
    float idepth_var;
    unsigned char color[4];
};



class ROSDepthTexture {
public:
    ROSDepthTexture(bool color, rviz::Display *display);
    ~ROSDepthTexture();

    void init(float scaledDepthVarTH, float absDepthVarTH, int minNearSupp);
    void addMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg);
    bool update();
    void clear();

    void setScaledDepthVarTH(float threshold);
    void setAbsDepthVarTH(float threshold);
    void setMinNearSupp(int minNearSupp);

    const Ogre::TexturePtr& getTexture() {
        return texture_;
    }
    //const lsd_slam_msgs::keyframeMsgconstPtr& getImage();

    uint32_t getWidth() {
        return width_;
    }
    uint32_t getHeight() {
        return height_;
    }

private:
    //debug
    rviz::Display *display_;

    lsd_slam_msgs::keyframeMsgConstPtr current_image_;
    boost::mutex mutex_;
    bool new_image_;

    Ogre::TexturePtr texture_;
    Ogre::Image empty_image_;

    uint32_t width_;
    uint32_t height_;
    bool color_;

    float scaledTH_;
    float absTH_;
    int minNearSupp_;

    friend class KeyframeDisplay;

};

} // end namespace lsd_slam_rviz_plugins

#endif //ROS_DEPTH_TEXTURE_H
