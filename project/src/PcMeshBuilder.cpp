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

const Eigen::Vector3f debugColors[] = { Eigen::Vector3f(128,0,0), Eigen::Vector3f(0,128,0), Eigen::Vector3f(0,0,128),
                                        Eigen::Vector3f(128,128,0), Eigen::Vector3f(128,0,128), Eigen::Vector3f(0,128,128), Eigen::Vector3f(128,128,128)};


PcMeshBuilder::PcMeshBuilder() :
    originalInput_(0)
{
    sub_pc_ = nh_.subscribe(nh_.resolveName("lsd_slam/keyframes"), 10, &PcMeshBuilder::setFrom, this);
    pub_pc_ = nh_.advertise<MyPointCloud>("meshPc", 10);
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker_normals", 0);

    initMarker();

    std::cout << "PcMeshBuilder started..." << std::endl;
}

PcMeshBuilder::~PcMeshBuilder()
{
    delete[] originalInput_;
}

void PcMeshBuilder::initMarker()
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time();
    marker.header.frame_id = "world";
    marker.ns = "namespace_normals";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;    // TBD: LINE_LIST may be yield better performance
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
}

void PcMeshBuilder::createNormal(geometry_msgs::Pose pose)
{
    marker_.header.stamp = ros::Time::now();
    marker_.pose = pose;
}

void PcMeshBuilder::createNormal(Eigen::Matrix3f rotation, Eigen::Vector3f translation)
{
    marker_.header.stamp = ros::Time::now();
//    marker_.pose = pose;
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

inline void PcMeshBuilder::colorPC(MyPointCloud& cloud_in, Eigen::Vector3f color)
{
    for(int i=0; i<cloud_in.points.size(); i++) {
        cloud_in.points[i].r = color.x();
        cloud_in.points[i].g = color.y();
        cloud_in.points[i].b = color.z();
    }
}

void PcMeshBuilder::findPlanes(const MyPointCloud& cloud_in)
{
    MyPointCloud::Ptr union_cloud(new MyPointCloud);

    MyPointCloud::Ptr cropped_cloud(new MyPointCloud(cloud_in));
    MyPointCloud::Ptr cloud_f(new MyPointCloud);
    MyPointCloud::Ptr cloud_filtered(new MyPointCloud);
    MyPointCloud::Ptr cloud_plane(new MyPointCloud());

    // create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<MyPoint> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);

    // extract the planar inliers from the input cloud
    pcl::ExtractIndices<MyPoint> extract;
    int i=0;
    int nr_points = (int)cropped_cloud->points.size();
    while (cropped_cloud->points.size() > 0.3*nr_points) {
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
        colorPC(*cloud_plane, debugColors[i%(sizeof(debugColors)/sizeof(Eigen::Vector3f))]);
        // add newly created pointcloud in the union pointcloud
        *union_cloud += *cloud_plane;

        // create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
        cropped_cloud.swap(cloud_filtered);

        i++;
    }
    // add non planar point clouds to the union pointcloud with a white color
    *union_cloud += *cloud_filtered;

    // send pc2 msg to rviz
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*union_cloud,msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";

    pub_pc_.publish(msg);
}

void PcMeshBuilder::refreshPC()
{
    const float scaledDepthVarTH = pow(10.0f,-3.0f);
    const float absDepthVarTH = pow(10.0f, 1.0f);
    const int minNearSupport = 7;
    const int sparsifyFactor = 1;

    float worldScale = camToWorld_.scale();

    MyPointCloud pc;
    pc.resize(width_*height_);
    int numPoints = 0;
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
            pc.points[numPoints] = point;

            numPoints++;
        }
    }
    // refit pointcloud and search for planes
    pc.resize(numPoints);
    //pcl::transformPointCloud(pc,pc,camToWorld_.matrix());
    findPlanes(pc);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "project");

    PcMeshBuilder pcBuilder;

    ros::spin();
    return 0;
}
