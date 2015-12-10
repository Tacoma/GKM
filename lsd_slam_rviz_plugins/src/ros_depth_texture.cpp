#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "ros_depth_texture.h"

namespace lsd_slam_rviz_plugins {

ROSDepthTexture::ROSDepthTexture(bool color, rviz::Display *display) :
        new_image_(false), width_(0), height_(0), scaledTH_(0.001), absTH_(0.1), minNearSupp_(7) {
    color_ = color;
    display_ = display;

    empty_image_.load("no_image.png",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);


    static uint32_t colorCount = 0;
    static uint32_t depthCount = 0;
    std::stringstream ss;
    if (color) ss << "colorTex" << colorCount++;
    else ss << "depthTex" << depthCount++;


    texture_ = Ogre::TextureManager::getSingleton().loadImage(ss.str(),
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            empty_image_, Ogre::TEX_TYPE_2D, 0);

    //display_->setStatus(rviz::StatusProperty::Ok, QString::fromStdString(texture_->getName()), "created");
}

void ROSDepthTexture::init(float scaledDepthVarTH, float absDepthVarTH, int minNearSupp) {
    scaledTH_ = scaledDepthVarTH;
    absTH_ = absDepthVarTH;
    minNearSupp_ = minNearSupp;
    new_image_ = true;
}

ROSDepthTexture::~ROSDepthTexture() {
    //ROS_INFO_STREAM("Reset message with id " << current_image_->id << " and pointer count " << current_image_.use_count());
    current_image_.reset();
    
    //display_->deleteStatus(QString::fromStdString(texture_->getName()));
    
    //TODO: destroy SceneNode
    // TODO: remove textures and material? (see ogre_tools::Shape::~Shape())
    texture_->unload();
    Ogre::ResourcePtr resource(texture_);
    Ogre::TextureManager::getSingleton().remove(resource);
    // Ogre::TextureManager::getSingleton().remove(...);
    // Ogre::MaterialManager::getSingleton().remove(...);
}

void ROSDepthTexture::clear() {
    boost::mutex::scoped_lock lock(mutex_);

    texture_->unload();
    texture_->loadImage(empty_image_);

    new_image_ = false;
    current_image_.reset();
}

// save new data location and wait for update
void ROSDepthTexture::addMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg) {
    boost::mutex::scoped_lock lock(mutex_);
    current_image_ = msg;
    new_image_ = true;
}

void ROSDepthTexture::setScaledDepthVarTH(float threshold) {
    new_image_ = true;
    scaledTH_ = threshold;
}

void ROSDepthTexture::setAbsDepthVarTH(float threshold) {
    new_image_ = true;
    absTH_ = threshold;
}

void ROSDepthTexture::setMinNearSupp(int minNearSupp) {
    new_image_ = true;
    minNearSupp_ = minNearSupp;
}

// load image into texture_
bool ROSDepthTexture::update() {
    lsd_slam_msgs::keyframeMsgConstPtr image;
    bool new_image = false;
    {
        boost::mutex::scoped_lock lock(mutex_);

        image = current_image_;
        new_image = new_image_;
        new_image_ = false;
    }

    if (!image || !new_image) {
        return false;
    }
    //ROS_INFO_STREAM("update() called from " << boost::this_thread::get_id());

    width_  = image->width;
    height_ = image->height;

    //split rgb and depth info
    InputPointDense* pointcloud = (InputPointDense*) image->pointcloud.data();
    uint8_t* imagePtr;
    size_t element_size = 0;
    int c = 1; // number of channels, format has to be changed accordingly
    unsigned char *color_buffer = nullptr;
    float* depth_buffer = nullptr;
    size_t numValidPixels = 0;

    if (color_) {
        color_buffer = new unsigned char[c*width_*height_];
        imagePtr = (uint8_t*) color_buffer;
        element_size = c*sizeof(unsigned char);

        for (int i = 0; i < width_*height_; i++) {
            for (int j = 0; j < c; j++) {
                color_buffer[c*i+j] = pointcloud[i].color[j];
            }
        }
    } else {
        depth_buffer = new float[width_*height_];
        imagePtr = (uint8_t*) depth_buffer;
        element_size = sizeof(float);

        for (int i = 0; i < width_*height_; i++) {

            depth_buffer[i] = 0.0f;

            // non-valid point
            if (pointcloud[i].idepth <= 0.0f) continue;

            float depth = 1.0f / pointcloud[i].idepth;
            float depth4 = depth*depth;
            depth4 *= depth4;
            //TODO: absDepthVarTH

            // variance check
            if(pointcloud[i].idepth_var * depth4 > scaledTH_) {
                continue;
            }

            // neighborhood check
            if(minNearSupp_ > 1)
            {
                int numNeighbors = 0;
                for(int dx=-1;dx<2;dx++)
                    for(int dy=-1;dy<2;dy++)
                    {
                        int idx = i+dx+dy*width_;
                        if(pointcloud[idx].idepth > 0.0f)
                        {
                            float diff = pointcloud[idx].idepth - 1.0f / depth;
                            if(diff*diff < 2*pointcloud[i].idepth_var)
                                numNeighbors++;
                        }
                    }
                if(numNeighbors < minNearSupp_)
                    continue;
            }


            depth_buffer[i] = depth;
            numValidPixels++;
        }
    }

    // Ogre
    Ogre::PixelFormat format;
    if (color_) {
        //format = Ogre::PF_BYTE_RGBA; // 4 Byte rgba
        format = Ogre::PF_L8;   // 1 Byte intensity
    } else {
        format = Ogre::PF_FLOAT32_R;
    }
    //ROS_INFO_STREAM("Texture " << texture_->getName() <<" has new data of type " << format);

    Ogre::Image ogre_image;
    Ogre::DataStreamPtr pixel_stream;
    pixel_stream.bind(new Ogre::MemoryDataStream(imagePtr, width_*height_*element_size));

    try {
        ogre_image.loadRawData(pixel_stream, width_, height_, 1, format, 1, 0);
    } catch (Ogre::Exception& e) {
        // TODO: signal error better
        ROS_ERROR("Error loading image: %s", e.what());
        return false;
    }

    texture_->unload();
    texture_->loadImage(ogre_image);

    if (!color_) {
    //    ROS_INFO_STREAM("TEXTURE: Usage = " << (float)numValidPixels/(width_*height_) << "%");
    }

    if (color_buffer != nullptr) {
        delete[] color_buffer; }
    if (depth_buffer != nullptr) {
        delete[] depth_buffer; }

    return true;
}



} // end namespace lsd_slam_rviz_plugins

