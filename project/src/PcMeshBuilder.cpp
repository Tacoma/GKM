#include "PcMeshBuilder.h"

// pcl
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <iostream>

const Eigen::Vector3f debugColors[] = { Eigen::Vector3f(128,0,0),
                                        Eigen::Vector3f(0,128,0),
                                        Eigen::Vector3f(0,0,128),
                                        Eigen::Vector3f(128,128,0),
                                        Eigen::Vector3f(128,0,128),
                                        Eigen::Vector3f(0,128,128),
                                        Eigen::Vector3f(128,128,128)};


PcMeshBuilder::PcMeshBuilder() :
    showOnlyPlanarPointclouds_(true),
    showOnlyCurrent_(true),
    showOnlyColorCurrent_(false) {

    // subscriber and publisher
    sub_keyframes_ = nh_.subscribe(nh_.resolveName("euroc2/lsd_slam/keyframes"), 10, &PcMeshBuilder::processMessage, this);
    sub_liveframes_ = nh_.subscribe(nh_.resolveName("euroc2/lsd_slam/liveframes"), 10, &PcMeshBuilder::processMessage, this);
    pub_pc_ = nh_.advertise<MyPointcloud>("meshPc", 10);

    // init
    pointcloud_union_planar_ = boost::make_shared<MyPointcloud>();
    pointcloud_union_non_planar_ = boost::make_shared<MyPointcloud>();

    std::cout << "PcMeshBuilder started..." << std::endl;
}

PcMeshBuilder::~PcMeshBuilder() {
}

void PcMeshBuilder::reset() {
    pointcloud_union_planar_ = boost::make_shared<MyPointcloud>();
    pointcloud_union_non_planar_ = boost::make_shared<MyPointcloud>();
    pointcloud_planar_vector_.clear();

    last_frame_id_ = 0;
}

void PcMeshBuilder::processMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg) {
    if (msg->isKeyframe) {
        MyPointcloud::Ptr cloud = processPointcloud(msg);
        cloud = findPlanes(cloud);

        if(cloud) {
            // add to vector and accumulated pointcloud
            pointcloud_planar_vector_.push_back(cloud);
            publishPointclouds();

            if(showOnlyColorCurrent_ && pointcloud_planar_vector_.size() > 0) {
                colorPointcloud(*pointcloud_planar_vector_[pointcloud_planar_vector_.size()-1], Eigen::Vector3f(128,128,128));
            }
        }
    } else {
        // check for reset
        if (last_frame_id_ > msg->id) {
            ROS_INFO_STREAM("detected backward-jump in id (" << last_frame_id_ << " to " << msg->id << "), resetting!");
            reset();
        }
        last_frame_id_ = msg->id;
    }
}

MyPointcloud::Ptr PcMeshBuilder::processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg) {
    Sophus::Sim3f camToWorld;
    memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));

    float fx = msg->fx;
    float fy = msg->fy;
    float cx = msg->cx;
    float cy = msg->cy;

    float fxi = 1.0/fx;
    float fyi = 1.0/fy;
    float cxi = -cx / fx;
    float cyi = -cy / fy;

    int width  = msg->width;
    int height = msg->height;
//    unsigned int id = msg->id;
//    double time = msg->time;

    InputPointDense* originalInput_ = 0;

    if(msg->pointcloud.size() != width*height*sizeof(InputPointDense)) {
        if(msg->pointcloud.size() != 0)
            ROS_WARN_STREAM("PC with points, but number of points not right!");
    } else {
        originalInput_ = new InputPointDense[width*height];
        memcpy(originalInput_, msg->pointcloud.data(), width*height*sizeof(InputPointDense));
    }

    if(originalInput_ == 0) {
        ROS_ERROR_STREAM("originalInput_ was null");
//        return boost::shared_ptr<MyPointcloud>(); // return nullptr;
    }

    // TODO parameterize
    const float scaledDepthVarTH = pow(10.0f,-3.0f);
    const float absDepthVarTH = pow(10.0f, 1.0f);
    const int minNearSupport = 7;
    const int sparsifyFactor = 1;

    float worldScale = camToWorld.scale();

    MyPointcloud::Ptr pc = boost::make_shared<MyPointcloud>();
    pc->resize(width*height);
    int numPoints = 0;
    for(int y=1; y<height-1; y++) {
        for(int x=1; x<width-1; x++) {
            if(originalInput_[x+y*width].idepth <= 0)
                continue;

            if(sparsifyFactor > 1 && rand()%sparsifyFactor != 0)
                continue;

            float depth = 1 / originalInput_[x+y*width].idepth;
            float depth4 = depth*depth;
            depth4*= depth4;

            if(originalInput_[x+y*width].idepth_var * depth4 > scaledDepthVarTH)
                continue;

            if(originalInput_[x+y*width].idepth_var * depth4 * worldScale*worldScale > absDepthVarTH)
                continue;

            if(minNearSupport > 1) {
                int nearSupport = 0;
                for(int dx=-1; dx<2; dx++) {
                    for(int dy=-1; dy<2; dy++) {
                        int idx = x+dx+(y+dy)*width;
                        if(originalInput_[idx].idepth > 0) {
                            float diff = originalInput_[idx].idepth - 1.0f / depth;
                            if(diff*diff < 2*originalInput_[x+y*width].idepth_var)
                                nearSupport++;
                        }
                    }
                }

                if(nearSupport < minNearSupport)
                    continue;
            }

            MyPoint point;
            point.x = (x*fxi + cxi) * depth;
            point.y = (y*fyi + cyi) * depth;
            point.z = depth;
            point.r = originalInput_[x+y*width].color[2];
            point.g = originalInput_[x+y*width].color[1];
            point.b = originalInput_[x+y*width].color[0];
            pc->points[numPoints] = point;

            numPoints++;
        }
    }
    // refit pointcloud and search for planes
    pc->resize(numPoints);
    pcl::transformPointCloud(*pc,*pc,camToWorld.matrix());

    delete[] originalInput_;

    return pc;
}

MyPointcloud::Ptr PcMeshBuilder::findPlanes(const MyPointcloud::Ptr cloud_in, unsigned int num_planes) {
    MyPointcloud::Ptr union_cloud = boost::make_shared<MyPointcloud>();

    MyPointcloud::Ptr cropped_cloud = boost::make_shared<MyPointcloud>(*cloud_in);
    MyPointcloud::Ptr cloud_f = boost::make_shared<MyPointcloud>();
    MyPointcloud::Ptr cloud_filtered = boost::make_shared<MyPointcloud>();
    MyPointcloud::Ptr cloud_plane = boost::make_shared<MyPointcloud>();

    // create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<MyPoint> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01); // TODO parameterize

    // extract the planar inliers from the input cloud
    pcl::ExtractIndices<MyPoint> extract;
    for(int i=0; i<num_planes; i++) {
        // segment the largest planar component from the cropped cloud
        seg.setInputCloud(cropped_cloud);
        seg.segment(*inliers, *coefficients);
        if(inliers->indices.size () == 0) {
            ROS_WARN_STREAM ("Could not estimate a planar model for the given pointcloud data");
            break;
        }

        // extract the inliers
        extract.setInputCloud(cropped_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        // get the points associated with the planar surface
        extract.filter(*cloud_plane);

        // recolor the extracted pointcloud for debug visualization
        colorPointcloud(*cloud_plane, debugColors[i%(sizeof(debugColors)/sizeof(Eigen::Vector3f))]);
        // add newly created pointcloud in the union pointcloud
        *union_cloud += *cloud_plane;

        // create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
        cropped_cloud.swap(cloud_filtered);

        i++;
    }
    // add pointcloud keyframe to the accumulated pointclouds depending on planar property
    *pointcloud_union_planar_ += *union_cloud;
    *pointcloud_union_non_planar_ += *cloud_filtered;

    return union_cloud;
}

inline void PcMeshBuilder::colorPointcloud(MyPointcloud& cloud_in, Eigen::Vector3f color) {
    for(int i=0; i<cloud_in.points.size(); i++) {
        cloud_in.points[i].r = color.x();
        cloud_in.points[i].g = color.y();
        cloud_in.points[i].b = color.z();
    }
}

void PcMeshBuilder::publishPointclouds() {
    MyPointcloud::Ptr union_cloud = boost::make_shared<MyPointcloud>();

    if(!showOnlyColorCurrent_) {
        if(showOnlyCurrent_ && pointcloud_planar_vector_.size() > 0) {
            *union_cloud += *pointcloud_planar_vector_[pointcloud_planar_vector_.size()-1];
        } else {
            *union_cloud += *pointcloud_union_planar_;
        }
    } else {
        for(int i=0; i<pointcloud_planar_vector_.size(); i++) {
            *union_cloud += *pointcloud_planar_vector_[i];
        }
    }

    if(!showOnlyPlanarPointclouds_) {
        *union_cloud += *pointcloud_union_non_planar_;
    }

    sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*union_cloud, *msg);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "world";

    pub_pc_.publish(msg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "project");
    PcMeshBuilder pcBuilder;
    ros::spin();
    return 0;
}
