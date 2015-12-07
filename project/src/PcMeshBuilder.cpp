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

PcMeshBuilder::PcMeshBuilder() :
    originalInput_(0)
{
    sub_pc_ = nh_.subscribe(nh_.resolveName("lsd_slam/keyframes"), 10, &PcMeshBuilder::setFrom, this);
    pub_pc_ = nh_.advertise<MyPointCloud>("meshPc", 10);

    std::cout << "PcMeshBuilder started..." << std::endl;
}

PcMeshBuilder::~PcMeshBuilder()
{
    delete[] originalInput_;
}

void PcMeshBuilder::setFrom(project::keyframeMsgConstPtr msg)
{
    if(!msg->isKeyframe)
        return;

    memcpy(camToWorld_.data(), msg->camToWorld.data(), 7*sizeof(float));

    fx_ = msg->fx;
    fy_ = msg->fy;
    cx_ = msg->cx;
    cy_ = msg->cy;

    fxi_ = 1.0/fx_;
    fyi_ = 1.0/fy_;
    cxi_ = -cx_ / fx_;
    cyi_ = -cy_ / fy_;

    width_  = msg->width;
    height_ = msg->height;
    id_     = msg->id;
    time_   = msg->time;

    if(originalInput_ != 0) {
        delete[] originalInput_;
        originalInput_=0;
    }

    if(msg->pointcloud.size() != width_*height_*sizeof(InputPointDense)) {
        if(msg->pointcloud.size() != 0)
            ROS_WARN_STREAM("PC with points, but number of points not right!");
    } else {
        originalInput_ = new InputPointDense[width_*height_];
        memcpy(originalInput_, msg->pointcloud.data(), width_*height_*sizeof(InputPointDense));
    }

    if(originalInput_ != 0)
        refreshPC();
}

void PcMeshBuilder::findPC(MyPointCloud cloud_in) {

    MyPointCloud::Ptr cropped_cloud(new MyPointCloud(cloud_in));
    MyPointCloud::Ptr cloud_f (new MyPointCloud);
    MyPointCloud::Ptr cloud_filtered (new MyPointCloud);
    MyPointCloud::Ptr cloud_plane (new MyPointCloud ());

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<MyPoint> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (200);
    seg.setDistanceThreshold (0.004);

    // Segment the largest planar component from the cropped cloud
    seg.setInputCloud (cropped_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
        //break;
    }


    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<MyPoint> extract;
    extract.setInputCloud (cropped_cloud);
    extract.setIndices(inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." );

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_plane,msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";

    pub_pc_.publish(msg);

}

void PcMeshBuilder::refreshPC()
{
    const float scaledDepthVarTH = 1.0f;
    const float absDepthVarTH = 1.0f;
    const int minNearSupport = 0;
    const int sparsifyFactor = 1;

    float worldScale = camToWorld_.scale();

    MyPointCloud pc;

    for(int y=1; y<height_-1; y++) {
        for(int x=1; x<width_-1; x++) {
            if(originalInput_[x+y*width_].idepth <= 0)
                continue;

            if(sparsifyFactor > 1 && rand()%sparsifyFactor != 0)
                continue;

            float depth = 1 / originalInput_[x+y*width_].idepth;
            float depth4 = depth*depth;
            depth4*= depth4;

            if(originalInput_[x+y*width_].idepth_var * depth4 > scaledDepthVarTH)
                continue;

            if(originalInput_[x+y*width_].idepth_var * depth4 * worldScale*worldScale > absDepthVarTH)
                continue;

            if(minNearSupport > 1) {
                int nearSupport = 0;
                for(int dx=-1; dx<2; dx++) {
                    for(int dy=-1; dy<2; dy++) {
                        int idx = x+dx+(y+dy)*width_;
                        if(originalInput_[idx].idepth > 0) {
                            float diff = originalInput_[idx].idepth - 1.0f / depth;
                            if(diff*diff < 2*originalInput_[x+y*width_].idepth_var)
                                nearSupport++;
                        }
                    }
                }

                if(nearSupport < minNearSupport)
                    continue;
            }

            MyPoint point;
            point.x = (x*fxi_ + cxi_) * depth;
            point.y = (y*fyi_ + cyi_) * depth;
            point.z = depth;
            point.r = originalInput_[x+y*width_].color[2];
            point.g = originalInput_[x+y*width_].color[1];
            point.b = originalInput_[x+y*width_].color[0];
            pc.points.push_back(point);
        }
    }

    //// planar segmentation test ////
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//    // create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    // optional
////    seg.setOptimizeCoefficients(true);
//    // mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (0.01);

//    seg.setInputCloud(pc);
//    seg.segment(*inliers, *coefficients);

//    if (inliers->indices.size () == 0)
//        ROS_ERROR("Could not estimate a planar model for the given dataset.");

    // send pc as msg
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pc,msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";

    findPC(pc);
    //pub_pc_.publish(msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "project");

    PcMeshBuilder pcBuilder;

    ros::spin();
    return 0;
}
