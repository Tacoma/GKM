// pcl
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
//#include <jsk_recognition_msgs/PolygonArray.h>

#include <iostream>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "PcMeshBuilderV2.h"


const Eigen::Vector3f debugColors[] = { Eigen::Vector3f(255,  0,  0),
                                        Eigen::Vector3f(  0,255,  0),
                                        Eigen::Vector3f(  0,  0,255),
                                        Eigen::Vector3f(255,255,  0),
                                        Eigen::Vector3f(  0,255,255),
                                        Eigen::Vector3f(255,  0,255),
                                        Eigen::Vector3f(128,  0,  0),
                                        Eigen::Vector3f(128,128,  0),
                                        Eigen::Vector3f(  0,128,  0),
                                        Eigen::Vector3f(128,  0,128),
                                        Eigen::Vector3f(  0,128,128),
                                        Eigen::Vector3f(  0,  0,128)
                                      };


PcMeshBuilder::PcMeshBuilder()
{

    // subscriber and publisher
    sub_keyframes_ = nh_.subscribe(nh_.resolveName("euroc2/lsd_slam/keyframes"), 10, &PcMeshBuilder::processMessageStickToSurface, this);
    sub_liveframes_ = nh_.subscribe(nh_.resolveName("euroc2/lsd_slam/liveframes"), 10, &PcMeshBuilder::processMessageStickToSurface, this);
    pub_pc_ = nh_.advertise< pcl::PointCloud<MyPoint> >("meshPc", 10);
    //pub_markers_ = nh_.advertise< jsk_recognition_msgs::PolygonArray>("Hull", 10);
    pub_tf_ = nh_.advertise< geometry_msgs::TransformStamped>("plane", 10);

    // init
    pointcloud_planar_ = boost::make_shared<MyPointcloud>();
    pointcloud_non_planar_ = boost::make_shared<MyPointcloud>();
    pointcloud_debug_ = boost::make_shared<MyPointcloud>();

    // Setting up Dynamic Reconfiguration

    dynamic_reconfigure::Server<project::projectConfig>::CallbackType f;

    f = boost::bind(&PcMeshBuilder::configCallback, this, _1, _2);
    server_.setCallback(f);

    std::cout << "PcMeshBuilder started..." << std::endl;

    planeExists_ = false;
}

PcMeshBuilder::~PcMeshBuilder() {
    reset();
}

void PcMeshBuilder::reset() {

    std::cout << "resetting PcMeshBuilder..." << std::endl;

    pointcloud_planar_ = boost::make_shared<MyPointcloud>();
    pointcloud_non_planar_ = boost::make_shared<MyPointcloud>();
    planes_.clear();
    coefficients_.clear();
    planeExists_ = false;

    last_frame_id_ = 0;
}

void PcMeshBuilder::processMessageStickToSurface(const lsd_slam_msgs::keyframeMsgConstPtr msg) {

    if(stickToSurface_ == true) {

        if (msg->isKeyframe) {
            last_msg_ = msg;
            MyPointcloud::Ptr cloud = boost::make_shared<MyPointcloud>();
            Sophus::Sim3f pose;

            processPointcloud(msg, cloud, pose);

            //Find the largest plane only if no plane exists
            if(!planeExists_) {

                std::cout << "Finding largest plane..." << std::endl;
                coefficients_.clear();
                findPlanes(cloud, pose, 1);
            }

            // Refine the largest plane with new inliers
            std::cout << "refining Plane coefficients..." << std::endl;
            refinePlane(cloud);
	    
            //publish plane
            publishPlane(pose);

            // add to vector and accumulated pointcloud
            publishPointclouds();

        } /*else {
            // check for reset
            if (last_frame_id_ > msg->id) {
                ROS_INFO_STREAM("detected backward-jump in id (" << last_frame_id_ << " to " << msg->id << "), resetting!");
                reset();
            }
            last_frame_id_ = msg->id;
        }*/
    }
}

void PcMeshBuilder::processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg, MyPointcloud::Ptr cloud, Sophus::Sim3f &pose) {
    memcpy(pose.data(), msg->camToWorld.data(), 7*sizeof(float));

    float fxi = 1.0/msg->fx;
    float fyi = 1.0/msg->fy;
    float cxi = -msg->cx / msg->fx;
    float cyi = -msg->cy / msg->fy;
    int width  = msg->width;
    int height = msg->height;

    cloud->id = msg->id;
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
        return;
    }

    Eigen::Vector2i min, max;
    getProcessWindow(min, max, width, height);

    float worldScale = pose.scale();

    cloud->resize(width*height);
    int numPoints = 0;
    for(int y=min.y(); y<max.y(); y++) {
        for(int x=min.x(); x<max.x(); x++) {

            if(originalInput_[x+y*width].idepth <= 0) {
                continue;
            }
            if(sparsifyFactor_ > 1 && rand()%sparsifyFactor_ != 0) {
                continue;
            }

            float depth = 1 / originalInput_[x+y*width].idepth;
            float depth4 = depth*depth;
            depth4*= depth4;

            if(originalInput_[x+y*width].idepth_var * depth4 > scaledDepthVarTH_) {
                continue;
            }
            if(originalInput_[x+y*width].idepth_var * depth4 * worldScale*worldScale > absDepthVarTH_) {
                continue;
            }

            if(minNearSupport_ > 1) {
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
                if(nearSupport < minNearSupport_) {
                    continue;
                }
            }

            MyPoint point;
            point.x = (x*fxi + cxi) * depth;
            point.y = (y*fyi + cyi) * depth;
            point.z = depth;
            point.r = originalInput_[x+y*width].color[2];
            point.g = originalInput_[x+y*width].color[1];
            point.b = originalInput_[x+y*width].color[0];
            cloud->points[numPoints] = point;

            numPoints++;
        } // x
    } // y
    // refit pointcloud and search for planes
    cloud->resize(numPoints);
    //pcl::transformPointCloud(*cloud,*cloud,pose.matrix());

    delete[] originalInput_;
}


void PcMeshBuilder::refinePlane(MyPointcloud::Ptr cloud) {

    // Init
    pointcloud_debug_ = boost::make_shared<MyPointcloud>();
    MyPointcloud::Ptr cloud_filtered = boost::make_shared<MyPointcloud>();
    pcl::ExtractIndices<MyPoint> extract;

    pcl::SampleConsensusModelPlane<MyPoint>::Ptr model = boost::make_shared<pcl::SampleConsensusModelPlane<MyPoint> >(cloud);
    std::vector< int > temp;
    model->setIndices(temp); // Assures that indices are reset so setInputCloud() will create new ones
    model->setInputCloud(cloud);

    boost::shared_ptr<std::vector<int> > inliers = boost::make_shared<std::vector<int> >();

    Plane::Ptr plane;

    Eigen::VectorXf coefficients, coefficients_refined;
    coefficients = coefficients_[0];

    // Find inliers to the best plane, and  refit the plane with new points only, if enough points were found.
    model->selectWithinDistance(coefficients, distanceThreshold_,*inliers);

    if(inliers->size() >= minPointsForEstimation_) {
        model->optimizeModelCoefficients(*inliers, coefficients, coefficients_refined);

        // Get the updated inlier points and save the updated Coefficients
        if(coefficients_refined.size() == 4){
            model->selectWithinDistance(coefficients_refined, distanceThreshold_,*inliers);
            coefficients_[0] = coefficients_refined;
        }
    }

    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);

    *pointcloud_debug_ += *cloud_filtered;

}



/**
 * looks for num_planes planes in cloud_in and returns all points in these planes,
 * colored to the corresponding plane.
 */
void PcMeshBuilder::findPlanes(MyPointcloud::Ptr cloud, const Sophus::Sim3f &pose, unsigned int num_planes) {
    MyPointcloud::Ptr cloud_union_planar = boost::make_shared<MyPointcloud>();

    MyPointcloud::Ptr cloud_cropped = cloud;
    int size = cloud->size();
    MyPointcloud::Ptr cloud_filtered = boost::make_shared<MyPointcloud>();
    MyPointcloud::Ptr cloud_plane = boost::make_shared<MyPointcloud>();

    // create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<MyPoint> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<MyPoint> extract;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(distanceThreshold_);

    // extract the planar inliers from the input cloud
    for(int i=0; i<num_planes && cloud_cropped->size() > std::max(noisePercentage_*size, (float)minPointsForEstimation_) ; i++) {
        // segment the largest planar component from the cropped cloud
        seg.setInputCloud(cloud_cropped);
        seg.segment(*inliers, *coefficients);
        if(inliers->indices.size() == 0) {
            ROS_WARN_STREAM ("Could not estimate a planar model for the given pointcloud data");
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_cropped);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        // recolor the extracted pointcloud for debug visualization
        Eigen::Vector3f color = debugColors[nextColor_++%(sizeof(debugColors)/sizeof(Eigen::Vector3f))];
        colorPointcloud(cloud_plane, color);
        // add newly created pointcloud in the union pointcloud
        *cloud_union_planar += *cloud_plane;

        // Extract the outliers
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
        cloud_cropped.swap(cloud_filtered);

        // Create plane and add points
        Plane::Ptr plane = boost::make_shared<Plane>(coefficients->values);
        plane->addPointcoud(cloud_plane);
        plane->color_ = color;
        planes_.push_back(plane);
        coefficients_.push_back(plane->getCoefficients());

        planeExists_ = true;
        i++;

    }

    // add pointcloud keyframe to the accumulated pointclouds depending on planar property
    *pointcloud_planar_ = *cloud_union_planar;
    *pointcloud_non_planar_ = *cloud_cropped;

    cloud = cloud_union_planar;
}

inline void PcMeshBuilder::colorPointcloud(MyPointcloud::Ptr cloud_in, Eigen::Vector3f color) {
    for(int i=0; i<cloud_in->points.size(); i++) {
        cloud_in->points[i].r = color.x();
        cloud_in->points[i].g = color.y();
        cloud_in->points[i].b = color.z();
    }
}

inline void PcMeshBuilder::getProcessWindow(Eigen::Vector2i &min, Eigen::Vector2i &max, int width, int height) {
    int radius = windowSize_/2;
    min.x() = width/2  - radius; // TODO parameterize
    max.x() = width/2  + radius;
    min.y() = height/2 - radius + windowPosY_;
    max.y() = height/2 + radius + windowPosY_;
    // Check values (clamp and swap)
    // Offset of 1 for minNearSupport
    min.x() = clamp(min.x(), 1, width-1);
    max.x() = clamp(max.x(), 1, width-1);
    min.y() = clamp(min.y(), 1, height-1);
    max.y() = clamp(max.y(), 1, height-1);
}

inline int PcMeshBuilder::clamp(int x, int min, int max) {
    x = (x < min) ? min : ((x > max) ? max : x);
}

void PcMeshBuilder::publishPointclouds() {

    sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*pointcloud_debug_, *msg);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "world";

    pub_pc_.publish(msg);
}

void PcMeshBuilder::publishPolygons() {
//     jsk_recognition_msgs::PolygonArray markers;
//     markers.header.frame_id = "world";
//     markers.header.stamp = ros::Time::now();
// 
//     geometry_msgs::PolygonStamped polyStamped;
//     polyStamped.header.stamp = ros::Time::now();
//     polyStamped.header.frame_id = "world";
// 
//     for (int i=0; i<planes_.size(); i++) {
//         MyPointcloud::ConstPtr hull = planes_[i]->getHull();
//         for (int i=0; i < hull->size(); i++) {
//             geometry_msgs::Point32 point;
//             point.x = hull->points[i].x;
//             point.y = hull->points[i].y;
//             point.z = hull->points[i].z;
//             polyStamped.polygon.points.push_back(point);
//         }
//         markers.polygons.push_back(polyStamped);
//     }
//     pub_markers_.publish(markers);
}

void PcMeshBuilder::publishPlane(const Sophus::Sim3f &pose) {

    // calculate Plane position
    Plane plane(coefficients_[0][0], coefficients_[0][1], coefficients_[0][2], coefficients_[0][3]);
    Eigen::Vector3f cam_t(0,0,0);// = pose.translation();
    Eigen::Vector3f plane_point, plane_normal;
    plane.getNormalForm(plane_point, plane_normal);
    Eigen::Vector3f intersection = plane.rayIntersection(cam_t, plane_normal);
    Eigen::Quaternionf plane_rot = plane.getRotation();

    // publish
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "world";
    tf.transform.translation.x = intersection.x();
    tf.transform.translation.y = intersection.y();
    tf.transform.translation.z = intersection.z();
    tf.transform.rotation.x = plane_rot.x();
    tf.transform.rotation.y = plane_rot.y();
    tf.transform.rotation.z = plane_rot.z();
    tf.transform.rotation.w = plane_rot.w();
    pub_tf_.publish(tf);
}


void PcMeshBuilder::configCallback(project::projectConfig &config, uint32_t level) {

    std::cout << "Configurating..." << std::endl;

    scaledDepthVarTH_ = pow(10.0f , config.scaledDepthVarTH );
    absDepthVarTH_ = pow(10.0f, config.absDepthVarTH);
    minNearSupport_ = config.minNearSupport;
    sparsifyFactor_ = config.sparsifyFactor;
    distanceThreshold_ = config.distanceThreshold;
    windowSize_ = config.windowSize;
    windowPosY_ = config.windowPosY;
    minPointsForEstimation_ = config.minPointsForEstimation;
    noisePercentage_ = 1-config.planarPercentage;
    maxPlanesPerCloud_ = config.maxPlanesPerCloud;
    stickToSurface_ = config.stickToSurface;
    planeExists_ = false; // Resetting the plane.
    if (last_msg_) {
        processMessageStickToSurface(last_msg_);
    }


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "project");
    PcMeshBuilder pcBuilder;
    ros::spin();
    return 0;
}




