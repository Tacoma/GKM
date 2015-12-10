
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreRoot.h>


#include "buffer_visual.h"
#include "ros_depth_texture.h" //InputPointDense


namespace lsd_slam_rviz_plugins {

BufferVisual::BufferVisual(Ogre::SceneManager *sceneManager, Ogre::SceneNode *parentNode, rviz::Display *display)
    : KeyframeVisual(sceneManager, parentNode, display),
    currentMsg_(nullptr) {

    static uint32_t count = 0;
    std::stringstream ss;
    ss << "KeyFrameMesh" << count;
    std::string meshName = ss.str();
    ss.str("");
    ss.clear();
    ss << "KeyFrameEntity" << count++;
    std::string entityName = ss.str();

    // Create the mesh via the MeshManager
    //TODO: make static mesh for all visuals with submeshes for each pc (are individual BB possible?)
    mesh_ = Ogre::MeshManager::getSingleton().createManual(meshName, "KeyframeVisual");

    // Create one submesh
    Ogre::SubMesh* submesh = mesh_->createSubMesh();
    submesh->useSharedVertices = true;
    submesh->operationType = Ogre::RenderOperation::OT_POINT_LIST;

    // Create bounding box (set in update)
    Ogre::AxisAlignedBox aabb;
    aabb.setNull();
    mesh_->_setBounds(aabb);

    // Create empty vertex data (set in update)
    mesh_->sharedVertexData = new Ogre::VertexData();
    mesh_->sharedVertexData->vertexCount = 0;

    // Create declaration of vertex and color data
    Ogre::VertexDeclaration* decl = mesh_->sharedVertexData->vertexDeclaration;
    decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    decl->addElement(1, 0, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);


    mesh_->load();

    // Add mesh to scene
    entity_ = sceneManager_->createEntity(entityName, meshName);
    entity_->setMaterialName("Test/Pointcloud");
    sceneNode_->attachObject(entity_);
}


BufferVisual::~BufferVisual() {

//     ROS_INFO_STREAM("Deleting BufferVisual in SceneNode " << sceneNode_->getName()  << " with objects: ");
//     Ogre::SceneNode::ObjectIterator it = sceneNode_->getAttachedObjectIterator();
//     while (it.hasMoreElements()) {
//         Ogre::MovableObject* node = it.getNext();
//         Ogre::String name = node->getName();
//         ROS_INFO_STREAM("--> " << name);
//     }

    // scene node is deleted in base class
    if (sceneNode_) {
	sceneNode_->detachObject(entity_);
	sceneNode_->needUpdate();
    }
    sceneManager_->destroyEntity(entity_);
}


void BufferVisual::init(float scaledDepthVarTH, float absDepthVarTH,
                        int minSupp, int depthSubsampleStep, bool isCameraVisible, bool deleteOriginalMsg) {
    setScaledDepthVarTH(scaledDepthVarTH);
    setAbsDepthVarTH(absDepthVarTH);
    setMinNearSupp(minSupp);
    setDepthSubsample(depthSubsampleStep);
    setCameraVisibility(isCameraVisible);
    setDeleteOriginalMsg(deleteOriginalMsg);
}


void BufferVisual::setFrom(const lsd_slam_msgs::keyframeMsgConstPtr msg) {
    KeyframeVisual::setFrom(msg);
    currentMsg_ = msg;
    newMsg_ = true;
}


void BufferVisual::update() {
    KeyframeVisual::update();
    if (!currentMsg_ || !newMsg_) {
        return;
    }
    newMsg_ = false;

    InputPointDense *pointcloud = (InputPointDense*) currentMsg_->pointcloud.data();

    // create temporary vertex array
    float *vertices = new float[width_*height_*3 / depthSubsample_];
    u_int32_t validPoints = 0;
    // create temporary color array
    Ogre::RenderSystem* rs = Ogre::Root::getSingleton().getRenderSystem();
    Ogre::RGBA *colors = new Ogre::RGBA[width_*height_ / depthSubsample_];

    Ogre::Vector3 min, max;
    min.x = min.y = min.z = std::numeric_limits<float>::max();
    max.x = max.y = max.z = std::numeric_limits<float>::min();


    // fill buffer
    for (u_int32_t y = 0; y < height_; y+= depthSubsample_) {
        for (u_int32_t x = 0; x < width_; x+=depthSubsample_) {

            u_int32_t i = x+y*width_;
            // Non-valid point
            if (pointcloud[i].idepth <= 0.0f) continue;

            float depth = 1.0f / pointcloud[i].idepth;
            float depth4 = depth*depth;
            depth4 *= depth4;
            //TODO: absDepthVarTH

            // Variance check
            if(pointcloud[i].idepth_var * depth4 > scaledTH_) {
                continue;
            }

            // Neighborhood check
            if(minNearSupp_ > 1)
            {
                int numNeighbors = 0;
                for(int dx=-1; dx<2; dx++)
                    for(int dy=-1; dy<2; dy++)
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

            // Unproject and store
            Ogre::Vector4 point = unprojection_ * Ogre::Vector4(x*depth, y*depth, depth, 1.0f);
            vertices[3*validPoints] = point.x;
            vertices[3*validPoints+1] = point.y;
            vertices[3*validPoints+2] = point.z;
            // Add color information
            //colors[validPoints] = Ogre::ColourValue(0.0f, 1.0f, 1.0f).getAsRGBA();
            rs->convertColourValue(Ogre::ColourValue(pointcloud[i].color[0],
                                   pointcloud[i].color[1], pointcloud[i].color[2]), &colors[validPoints]);

            // Create aabb
            min.x = (min.x > point.x) ? point.x : min.x;
            min.y = (min.y > point.y) ? point.y : min.y;
            min.z = (min.z > point.z) ? point.z : min.z;
            max.x = (max.x < point.x) ? point.x : max.x;
            max.y = (max.y < point.y) ? point.y : max.y;
            max.z = (max.z < point.z) ? point.z : max.z;

            validPoints++;

        } // x
    } // y

    // update mesh with new verices
    updateVertexPositionsAndColors( validPoints, vertices, colors);
    Ogre::AxisAlignedBox aabb;
    aabb.setMinimum(min);
    aabb.setMaximum(max);
    mesh_->_setBounds(aabb);

    delete[] vertices;
    delete[] colors;
    if (deleteOriginalMsg_) {
	currentMsg_ = nullptr;
    }
}


void BufferVisual::updateVertexPositionsAndColors(int size, float *points, Ogre::RGBA* colors ) {
    mesh_->sharedVertexData->vertexCount = size;
    Ogre::VertexDeclaration* decl = mesh_->sharedVertexData->vertexDeclaration;

    // Create vertex buffer
    vbuf_ = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
                decl->getVertexSize(0), mesh_->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC);

    // Upload the vertex data to the card
    vbuf_->writeData(0, vbuf_->getSizeInBytes(), points, true);

    // Set binding so buffer 0 is bound to our vertex buffer
    Ogre::VertexBufferBinding* bind = mesh_->sharedVertexData->vertexBufferBinding;
    bind->setBinding(0, vbuf_);

    // Create Color buffer
    cbuf_ = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
                decl->getVertexSize(1), mesh_->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC);

    // Upload the color data to the card
    cbuf_->writeData(0, cbuf_->getSizeInBytes(), colors, true);

    // Set binding so buffer 1 is bound to our color buffer
    bind->setBinding(1, cbuf_);
}


void BufferVisual::setImageSize(int width, int height) {
    width_ = width;
    height_ = height;
}


void BufferVisual::setUnprojectionMatrix(const Ogre::Matrix4 & unproj) {
    unprojection_ = unproj;
    newMsg_ = true;
}


void BufferVisual::setDepthSubsample(int subsample) {
    depthSubsample_ = subsample;
    newMsg_ = true;
}


void BufferVisual::setTriangleSideTH(float threshold) { //TODO
}


void BufferVisual::setScaledDepthVarTH(float threshold) {
    scaledTH_ = threshold;
    newMsg_ = true;
}


void BufferVisual::setAbsDepthVarTH(float threshold) {
    absTH_ = true;
    newMsg_ = true;
}


void BufferVisual::setMinNearSupp(int support) {
    minNearSupp_ = support;
    newMsg_ = true;
}

void BufferVisual::setDeleteOriginalMsg(bool deleteOriginalMsg) {
    deleteOriginalMsg_ =- deleteOriginalMsg;
    if (deleteOriginalMsg_) {
	currentMsg_ = nullptr;
    }
}


} // end namespace lsd_slam_rviz_plugins
