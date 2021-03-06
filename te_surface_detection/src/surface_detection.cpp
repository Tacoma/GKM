// pcl
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PolygonStamped.h>
#include <surface_detection_msgs/Surface.h>
#include <visualization_msgs/MarkerArray.h>
//#include <jsk_recognition_msgs/PolygonArray.h>

#include <iostream>
#include <fstream>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "surface_detection.h"

#include <math.h>

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

// TODO:
// - change subsribed from pc too depth image again

SurfaceDetection::SurfaceDetection() :
    scaledDepthVarTH_(-1),
    absDepthVarTH_(-1),
    minNearSupport_(0),
    sparsifyFactor_(0),
    distanceThreshold_(0.003),
    windowSize_(640),
    windowPosY_(0),
    minPointsForEstimation_(20),
    noisePercentage_(0),
    maxPlanesPerCloud_(1),
    surfaceType_(1)
{
    nh_ = ros::NodeHandle("");
    private_nh_ = ros::NodeHandle("~");

    // subscriber and publisher
    subLiveframes_ = nh_.subscribe< pcl::PointCloud<MyPoint> >("/euroc_hex/kinect/kinect/depth/points", 10, &SurfaceDetection::processMessage,this);
    subStickToSurface_ = nh_.subscribe<std_msgs::Bool>("controller/stickToSurface", 10, &SurfaceDetection::setSearchPlane, this);
    subSurfaceType_ = nh_.subscribe<std_msgs::Int32>("controller/surfaceType", 10, &SurfaceDetection::setSurfaceType, this);
    pubPc_ = private_nh_.advertise< pcl::PointCloud<MyPoint> >("meshPc", 10);
    pubTf_ = private_nh_.advertise<surface_detection_msgs::Surface>("surface", 10);
    pubCylinder_ = private_nh_.advertise<visualization_msgs::Marker>("cylinder", 10);

    //ros param
    private_nh_.param("mavTFName", mavTFName_, std::string("world"));
    private_nh_.param("mavTFCameraName", mavTFCameraName_, std::string("world"));

    // init
#ifdef VISUALIZE
    pcVisSurfaces_ = boost::make_shared<MyPointcloud>();
    pcVisOutliers_ = boost::make_shared<MyPointcloud>();
#endif
    pcVisDebug_ = boost::make_shared<MyPointcloud>();
    searchSurface_ = false;
    surfaceExists_ = false;

    opticalToSensor_ = Eigen::Matrix4f::Identity();
    // (row, column)
    opticalToSensor_ (0,0) = 0; opticalToSensor_ (0,1) = 0; opticalToSensor_ (0,2) = 1;
    opticalToSensor_ (1,0) =-1; opticalToSensor_ (1,1) = 0; opticalToSensor_ (1,2) = 0;
    opticalToSensor_ (2,0) = 0; opticalToSensor_ (2,1) =-1; opticalToSensor_ (2,2) = 0;

    status = -1;

    sensorToMav_ = Eigen::Affine3f::Identity();
    sensorToMav_.translation() << 0.0, 0.0, 0.0;
    Eigen::Matrix3f m;
//    m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ())
//            * Eigen::AngleAxisf(0.5 * M_PI, Eigen::Vector3f::UnitY())
//            * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
    m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
    sensorToMav_.rotate(Eigen::Quaternionf(m));

    std::cout << "SurfaceDetection started..." << std::endl;
}


SurfaceDetection::~SurfaceDetection()
{
    reset();
}


void SurfaceDetection::reset()
{
#ifdef VISUALIZE
    pcVisSurfaces_ = boost::make_shared<MyPointcloud>();
    pcVisOutliers_ = boost::make_shared<MyPointcloud>();
#endif

    plane_.reset();
    cylinder_.reset();
    searchSurface_ = false;
    surfaceExists_ = false;
    status = -1;
}


void SurfaceDetection::setSearchPlane(const std_msgs::Bool::ConstPtr& msg)
{
    if (searchSurface_ == msg->data) {
        return;
    }
    reset();
    searchSurface_ = msg->data;
//    if (lastMsg_) {
//        processMessage(lastMsg_);
//    }
}

void SurfaceDetection::setSurfaceType(const std_msgs::Int32::ConstPtr& msg)
{
    if (surfaceType_ == msg->data) {
        return;
    }
    reset();
    surfaceType_ = msg->data;
//    if (lastMsg_) {
//        processMessage(lastMsg_);
//    }
}

void SurfaceDetection::update()
{
    updateCamToWorld();
    updateMavToWorld();
    currentPose_ = mavToWorld_.matrix();
    Eigen::Matrix4f lastToCurrent = currentPose_.matrix().inverse() * lastPose_.matrix();
    
    // Transform surface into new frame
    if (cylinder_) {
        cylinder_->transform(lastToCurrent);
    }
    if (plane_) {
        plane_->transform(lastToCurrent);
    }

    // Rememember pose
    lastPose_ = currentPose_;
}

void SurfaceDetection::processMessage(const MyPointcloud::ConstPtr& msg)
{
    update();
    if(searchSurface_) {
        pcVisDebug_ = boost::make_shared<MyPointcloud>();
        pcLastCloud_ = boost::make_shared<MyPointcloud>();

        // subsample pointcloud and filter out NAN and INF values
        for(int j=0; j<480; j+=8) {
            for(int i=0; i<640; i+=8) {
                //pcLastCloud_->push_back(msg->at(j*640+i));
                MyPoint point = msg->at(j*640+i);
                if(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
                    pcLastCloud_->push_back(point);
            }
        }
        pcl::transformPointCloud(*pcLastCloud_, *pcLastCloud_, sensorToMav_ * opticalToSensor_);

        // Find the largest plane only if no plane exists
        if(!surfaceExists_) {
            // Search
            if (surfaceType_ == 1) {
                findPlanes(pcLastCloud_, 1);
            } else if (surfaceType_ == 2) {
                findCylinder(pcLastCloud_, 1);
            }
        }
        // We don't want to call refine Plane for the same message
        else {
            // Transform and refine the plane with new inliers, searching in whole pointcloud
            if (surfaceType_ == 1) {
                refinePlane(pcLastCloud_);
            } else if (surfaceType_ == 2) {
                refineCylinder(pcLastCloud_);
            }
        }
        publishPointclouds();
    }

    //publish plane
    publishCylinder();
    publishPlane();
}


/**
 * looks for numSurfaces planes in cloud_in and returns all points in these planes,
 * colored to the corresponding plane.
 */
void SurfaceDetection::findPlanes(boost::shared_ptr<MyPointcloud> cloud, unsigned int numSurfaces)
{
    std::cout << "Searching " << numSurfaces << " plane(s) in " << cloud->size() << " points.";
    // Init
    boost::shared_ptr<MyPointcloud> pcCropped = cloud;
    int size = cloud->size();
    // Allocation
    boost::shared_ptr<MyPointcloud> pcFiltered = boost::make_shared<MyPointcloud>();
#ifdef VISUALIZE
    boost::shared_ptr<MyPointcloud> pcVisSurfaces = boost::make_shared<MyPointcloud>();
#endif


    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<MyPoint> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<MyPoint> extract;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10);
    seg.setDistanceThreshold(distanceThreshold_);

    // Extract the planar inliers from the input cloud
    for(int i=0; i<numSurfaces && pcCropped->size() > std::max(noisePercentage_*size, (float)minPointsForEstimation_) ; i++) {

        // Segment the largest planar component from the cropped cloud
        seg.setInputCloud(pcCropped);
        seg.segment(*inliers, *coefficients);
        if(inliers->indices.size() == 0 || coefficients->values.size() != 4) {
            ROS_WARN_STREAM ("Could not estimate a planar model for the given pointcloud data");
            break;
        }


        // Extract the inliers/outliers
        extract.setInputCloud(pcCropped);
        extract.setIndices(inliers);
#ifdef VISUALIZE
        extract.filter(*pcFiltered);
        // Recolor the extracted pointcloud for debug visualization
        Eigen::Vector3f color = debugColors[nextColor_++%(sizeof(debugColors)/sizeof(Eigen::Vector3f))];
        colorPointcloud(pcFiltered, color);
        // add newly created pointcloud in the union pointcloud
        *pcVisSurfaces += *pcFiltered;
#endif
        extract.setNegative(true);
        extract.filter(*pcFiltered);
        pcCropped.swap(pcFiltered);

        // Create plane and add points
        plane_.reset(new Plane(coefficients->values));
        surfaceExists_ = true;
        i++;
        std::cout << "| " << i;
        switch (i) {
        case  1:
            std::cout << "st";
            break;
        case  2:
            std::cout << "nd";
            break;
        case  3:
            std::cout << "rd";
            break;
        default:
            std::cout << "th";
            break;
        }
        std::cout << " plane found";
    }
    std::cout << std::endl;

#ifdef VISUALIZE
    // add pointcloud keyframe to the accumulated pointclouds depending on planar property
    pcl::transformPointCloud(*pcVisSurfaces, *pcVisSurfaces, mavToWorld_);
    pcl::transformPointCloud(*pcCropped, *pcCropped, mavToWorld_);
    *pcVisSurfaces_ = *pcVisSurfaces;
    *pcVisOutliers_ = *pcCropped;
#endif
}


void SurfaceDetection::refinePlane(boost::shared_ptr<MyPointcloud> cloud)
{
    // Init
    boost::shared_ptr<MyPointcloud> pcFiltered = boost::make_shared<MyPointcloud>();

    // Create the planar model and set all the parameters
    pcl::SampleConsensusModelPlane<MyPoint>::Ptr model = boost::make_shared<pcl::SampleConsensusModelPlane<MyPoint> >(cloud);
    pcl::ExtractIndices<MyPoint> extract;
    boost::shared_ptr<std::vector<int> > inliers = boost::make_shared<std::vector<int> >();
    model->setInputCloud(cloud);


    Eigen::VectorXf coefficients, coefficients_refined;
   
    //Eigen::Matrix4f transform = currentPose_.matrix().inverse() * lastPose_.matrix(); done in update
    // Transform plane into new frame and get coefficients
    //plane_->transform(transform);
    
    coefficients = plane_->getCoefficients();

    // Find inliers of the plane, and
    model->selectWithinDistance(coefficients, distanceThreshold_,*inliers);

    // Refit the plane with new points only, if enough points were found.
    if(inliers->size() >= minPointsForEstimation_) {
        model->optimizeModelCoefficients(*inliers, coefficients, coefficients_refined);

        // Get the updated inlier points and save the updated Coefficients
        if(coefficients_refined.size() == 4) {
            model->selectWithinDistance(coefficients_refined, distanceThreshold_,*inliers); //TODO: rm debug code
            plane_->setCoefficients(coefficients_refined);
            if (status != 0) {
                std::cout << "Refining Plane..." << "Ok.";
                status = 0;
                std::cout << std::endl;
            }
        } else {
            if (status != 1) {
                std::cout  << "Refining Plane..." << " Could not find a refinement!" << std::endl;
                status = 1;
            }
            return;
        }
    } else {
        if (status != 2) {
            std::cout  << "Refining Plane..." << " (" << inliers->size() << "/"
                       << cloud->size() << ") Not enough points!" << std::endl;
            status = 2;
        }
        return;
    }

    // Extract the inliers	//TODO: rm debug code
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*pcFiltered);

    // transform pointcloud from optical to world
//    pcl::transformPointCloud(*pcFiltered, *pcFiltered,  mavToWorld_ * sensorToMav_ * opticalToSensor_);
    pcl::transformPointCloud(*pcFiltered, *pcFiltered,  mavToWorld_);
    *pcVisDebug_ += *pcFiltered;
}


/**
 * looks for numSurfaces planes in cloud_in and returns all points in these planes,
 * colored to the corresponding plane.
 */
void SurfaceDetection::findCylinder(boost::shared_ptr<MyPointcloud> cloud, unsigned int numSurfaces)
{
    std::cout << "Searching " << numSurfaces << " cylinder in " << cloud->size() << " points.";
    // Init
    boost::shared_ptr<MyPointcloud> pcCropped = cloud;
    int size = cloud->size();
    // Allocation
    boost::shared_ptr<MyPointcloud> pcFiltered = boost::make_shared<MyPointcloud>();
#ifdef VISUALIZE
    boost::shared_ptr<MyPointcloud> pcVisSurfaces = boost::make_shared<MyPointcloud>();
#endif
    boost::shared_ptr<MyNormalcloud> normalsFiltered  = boost::make_shared<MyNormalcloud>();
    boost::shared_ptr<MyNormalcloud> normalsCropped = boost::make_shared<MyNormalcloud>();

    // Create normals
    pcl::NormalEstimation<MyPoint, MyNormal> ne;
    pcl::search::KdTree<MyPoint>::Ptr tree (new pcl::search::KdTree<MyPoint> ());

    ne.setSearchMethod (tree);
    ne.setInputCloud (pcCropped);
    ne.setKSearch (50);
    ne.compute (*normalsCropped);


    // Create the segmentation object for the cylinder model and set all the parameters
    pcl::SACSegmentationFromNormals<MyPoint, MyNormal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<MyPoint> extract;
    pcl::ExtractIndices<MyNormal> extract_normals;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (100); //10000
    seg.setDistanceThreshold (0.05); //0.003
    seg.setRadiusLimits (0, 0.5);

    // Extract the planar inliers from the input cloud
    for(int i=0; i<numSurfaces && pcCropped->size() > std::max(noisePercentage_*size, (float)minPointsForEstimation_) ; i++) {

        // Segment the largest planar component from the cropped cloud
        seg.setInputCloud (pcCropped);
        seg.setInputNormals (normalsCropped);
        seg.segment(*inliers, *coefficients);
        if(inliers->indices.size() == 0 || coefficients->values.size() != 7) {
            ROS_WARN_STREAM ("Could not estimate a model for the given pointcloud data (" << coefficients->values.size() << " coefficients)");
            break;
        }
        // Extract the inliers/outliers
        extract.setInputCloud(pcCropped);
        extract.setIndices(inliers);
#ifdef VISUALIZE
        extract.filter(*pcFiltered);
        // Recolor the extracted pointcloud for debug visualization
        Eigen::Vector3f color = debugColors[nextColor_++%(sizeof(debugColors)/sizeof(Eigen::Vector3f))];
        colorPointcloud(pcFiltered, color);
        // Add newly created pointcloud in the union pointcloud
        *pcVisSurfaces += *pcFiltered;
#endif
        extract.setNegative(true);
        extract.filter(*pcFiltered);
        pcCropped.swap(pcFiltered);

        // Extraxt normal outliers
        extract_normals.setInputCloud(normalsCropped);
        extract_normals.setIndices(inliers);
        extract_normals.setNegative(true);
        extract_normals.filter(*normalsFiltered);
        normalsCropped.swap(normalsFiltered);

        // Create cylinder and add points
        cylinder_.reset(new Cylinder(coefficients->values));
        // DEBUG
        std::cout << "Coefficients: ";
        for(int c=0; c<7; c++)
            std::cout << coefficients->values[c] << ", ";
        std::cout << std::endl;

        surfaceExists_ = true;
        i++;
        std::cout << "| " << i;
        switch (i) {
        case  1:
            std::cout << "st";
            break;
        case  2:
            std::cout << "nd";
            break;
        case  3:
            std::cout << "rd";
            break;
        default:
            std::cout << "th";
            break;
        }
        std::cout << " cylinder found";
    }
    std::cout << std::endl;

#ifdef VISUALIZE
    // Add pointcloud keyframe to the accumulated pointclouds depending on planar property
    pcl::transformPointCloud(*pcVisSurfaces, *pcVisSurfaces, mavToWorld_);
    pcl::transformPointCloud(*pcCropped, *pcCropped, mavToWorld_);
    *pcVisSurfaces_ = *pcVisSurfaces;
    *pcVisOutliers_ = *pcCropped;
#endif
}


void SurfaceDetection::refineCylinder(boost::shared_ptr<MyPointcloud> cloud)
{
    // Init
    boost::shared_ptr<MyPointcloud> pcFiltered = boost::make_shared<MyPointcloud>();

    // Create the cylinder model and set all the parameters
    pcl::SampleConsensusModelCylinder<MyPoint, MyNormal>::Ptr model = boost::make_shared<pcl::SampleConsensusModelCylinder<MyPoint, MyNormal> >(cloud);
    model->setInputCloud(cloud);

    // Create normals
    pcl::NormalEstimation<MyPoint, MyNormal> ne;
    pcl::search::KdTree<MyPoint>::Ptr tree (new pcl::search::KdTree<MyPoint> ());
    MyNormalcloud::Ptr normals = boost::make_shared<MyNormalcloud>();
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (50);
    ne.compute (*normals);
    model->setInputNormals(normals);

    // Init debug visualization
    pcl::ExtractIndices<MyPoint> extract;
    boost::shared_ptr<std::vector<int> > inliers = boost::make_shared<std::vector<int> >();

    // Get coefficients
    Eigen::VectorXf coefficients, coefficients_refined;
    coefficients = cylinder_->getCoefficients();

    // Find inliers of the cylinder, and...
    model->selectWithinDistance(coefficients, distanceThreshold_, *inliers);

    // Extract the inliers	//TODO: rm debug code
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*pcFiltered);

    // transform pointcloud from optical to world
//    pcl::transformPointCloud(*pcFiltered, *pcFiltered,  mavToWorld_ * sensorToMav_ * opticalToSensor_);
    pcl::transformPointCloud(*pcFiltered, *pcFiltered,  mavToWorld_);
    *pcVisDebug_ += *pcFiltered;

    // ...refit the cylinder with new points only, if enough points were found.
    if(inliers->size() >= minPointsForEstimation_) {
        model->optimizeModelCoefficients(*inliers, coefficients, coefficients_refined);

        // Get the updated inlier points and save the updated Coefficients
        if(coefficients_refined.size() == 7) {
            model->selectWithinDistance(coefficients_refined, distanceThreshold_, *inliers); //TODO: rm debug code
            cylinder_->setCoefficients(coefficients_refined);
            if (status != 0) {
                std::cout << "Refining Cylinder..." << " Ok.";
                status = 0;
                std::cout << std::endl;
            }
        } else {
            if (status != 1) {
                std::cout  << "Refining Cylinder..." << " Could not find a refinement!" << std::endl;
                status = 1;
            }
            return;
        }
    } else {
        if (status != 2) {
            std::cout  << "Refining Cylinder..." << " (" << inliers->size() << "/"
                       << cloud->size() << ") Not enough points!" << std::endl;
            status = 2;
        }
        return;
    }
}


void SurfaceDetection::updateCamToWorld()
{
    try {
        ros::Time now = ros::Time::now();
        tf::StampedTransform tf;
        subTf_.waitForTransform("/world", mavTFName_, now, ros::Duration(0.5));
        subTf_.lookupTransform("/world", mavTFName_, now, tf);
        Eigen::Affine3d temp;
        tf::transformTFToEigen(tf, temp);
        camToWorld_ = temp.cast<float>();
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
}

void SurfaceDetection::updateMavToWorld()
{
    // get ground_truth pose
    try {
        ros::Time now = ros::Time::now();
        tf::StampedTransform tf;
        subTf_.waitForTransform("/world", mavTFName_, now, ros::Duration(0.5));
        subTf_.lookupTransform("/world", mavTFName_, now, tf);
        Eigen::Affine3d temp;
        tf::transformTFToEigen(tf, temp);
        mavToWorld_ = temp.cast<float>();
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------


inline void SurfaceDetection::colorPointcloud(boost::shared_ptr<MyPointcloud> cloud_in, Eigen::Vector3f color)
{
    for(int i=0; i<cloud_in->points.size(); i++) {
        cloud_in->points[i].r = color.x();
        cloud_in->points[i].g = color.y();
        cloud_in->points[i].b = color.z();
    }
}

inline void SurfaceDetection::clampProcessWindow(Eigen::Vector2i &min_inout, Eigen::Vector2i &max_inout, int width, int height)
{
    // Check values (clamp and swap)
    // Offset of 1 for minNearSupport
    min_inout.x() = clamp(min_inout.x(), 1, width-1);
    max_inout.x() = clamp(max_inout.x(), 1, width-1);
    min_inout.y() = clamp(min_inout.y(), 1, height-1);
    max_inout.y() = clamp(max_inout.y(), 1, height-1);
}


inline int SurfaceDetection::clamp(int x, int min, int max)
{
    return (x < min) ? min : ((x > max) ? max : x);
}


void SurfaceDetection::publishPointclouds()
{
    boost::shared_ptr<MyPointcloud> pcUnion = boost::make_shared<MyPointcloud>();

    *pcUnion += *pcVisDebug_;

#ifdef VISUALIZE
    *pcUnion += *pcVisSurfaces_;
    *pcUnion += *pcVisOutliers_;
#endif

    // Publish debug
    sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*pcUnion, *msg);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "/world";
    pubPc_.publish(msg);
}


void SurfaceDetection::publishPlane()
{
    if (!plane_) { return; }
    // calculate plane position
    Eigen::Vector3f cam_t(0,0,0);
    Eigen::Vector3f plane_point, plane_normal;
    plane_->calculateNormalForm(plane_point, plane_normal);
    Eigen::Vector3f intersection = plane_->rayIntersection(cam_t, plane_normal);
    Eigen::Quaternionf plane_rot = plane_->getRotation();

    // publish
    surface_detection_msgs::Surface tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = mavTFName_;
    tf.transform.translation.x = intersection.x();
    tf.transform.translation.y = intersection.y();
    tf.transform.translation.z = intersection.z();
    tf.transform.rotation.x = plane_rot.x();
    tf.transform.rotation.y = plane_rot.y();
    tf.transform.rotation.z = plane_rot.z();
    tf.transform.rotation.w = plane_rot.w();
    tf.radius = 0.1f; // not used for planes
    pubTf_.publish(tf);
    
    //visualization
    plane_rot = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1,0,0), plane_normal);

    visualization_msgs::Marker marker;
    marker.header.frame_id = mavTFName_;
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = intersection.x();
    marker.pose.position.y = intersection.y();
    marker.pose.position.z = intersection.z();
    marker.pose.orientation.x = plane_rot.x();
    marker.pose.orientation.y = plane_rot.y();
    marker.pose.orientation.z = plane_rot.z();
    marker.pose.orientation.w = plane_rot.w();
    marker.scale.x = distanceThreshold_;
    marker.scale.y = 2;
    marker.scale.z = 2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;
    pubCylinder_.publish( marker );
}

void SurfaceDetection::publishCylinder()
{
    if (!cylinder_) { return; }
    // calculate cylinder position
    Eigen::VectorXf coefficients = cylinder_->getCoefficients(); // cylinder_->calculateNormalForm
//    Cylinder::transformCylinder(sensorToMav_ * opticalToSensor_, coefficients);

    Eigen::Vector3f cylinder_point = Eigen::Vector3f(coefficients[0], coefficients[1], coefficients[2]);
    Eigen::Vector3f cylinder_direction = Eigen::Vector3f(coefficients[3], coefficients[4], coefficients[5]);
    float cylinder_radius = coefficients[6];
    Eigen::Quaternionf cylinder_rot = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0,0,1), cylinder_direction).normalized();

    // recalculate cylinder pos
    Eigen::Vector3f camPlanePoint(0.0f,0.0f,0.0f);  // cam_t
    Eigen::Vector3f camPlaneNormal(1.0f,0.0f,0.0f);
    Plane::transformPlane(sensorToMav_ * opticalToSensor_, camPlanePoint, camPlaneNormal);

    std::vector<float> planeCoefficients;
    planeCoefficients.push_back(camPlaneNormal[0]);
    planeCoefficients.push_back(camPlaneNormal[1]);
    planeCoefficients.push_back(camPlaneNormal[2]);
    planeCoefficients.push_back(-camPlaneNormal.dot(camPlanePoint));
    Plane proj_plane(planeCoefficients);
    Eigen::Vector3f intersection = proj_plane.rayIntersection(cylinder_point, cylinder_direction);

    Eigen::Vector3f camPlaneNormal2(0,0,1);
    std::vector<float> planeCoefficients2;
    planeCoefficients2.push_back(camPlaneNormal2[0]);
    planeCoefficients2.push_back(camPlaneNormal2[1]);
    planeCoefficients2.push_back(camPlaneNormal2[2]);
    planeCoefficients2.push_back(-camPlaneNormal2.dot(camPlanePoint));
    Plane proj_plane2(planeCoefficients2);
    Eigen::Vector3f intersection2 = proj_plane2.rayIntersection(cylinder_point, cylinder_direction);

    if((intersection-camPlanePoint).squaredNorm() > (intersection2-camPlanePoint).squaredNorm()) {
        intersection = intersection2;
    }

    // publish
    surface_detection_msgs::Surface tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = mavTFName_;
    tf.transform.translation.x = intersection.x();
    tf.transform.translation.y = intersection.y();
    tf.transform.translation.z = intersection.z();
    tf.transform.rotation.x = cylinder_rot.x();
    tf.transform.rotation.y = cylinder_rot.y();
    tf.transform.rotation.z = cylinder_rot.z();
    tf.transform.rotation.w = cylinder_rot.w();
    tf.radius = cylinder_radius;
    pubTf_.publish(tf);

    // visualize
    visualization_msgs::Marker marker;
    marker.header.frame_id = mavTFName_;
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = intersection.x();
    marker.pose.position.y = intersection.y();
    marker.pose.position.z = intersection.z();
    marker.pose.orientation.x = cylinder_rot.x();
    marker.pose.orientation.y = cylinder_rot.y();
    marker.pose.orientation.z = cylinder_rot.z();
    marker.pose.orientation.w = cylinder_rot.w();
    // scale times 2 because RVIZ marker expects the diameter instead of the radius
    marker.scale.x = 2.0f * cylinder_radius;
    marker.scale.y = 2.0f * cylinder_radius;
    marker.scale.z = 2.0f * cylinder_radius;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8;
    marker.lifetime = ros::Duration();
    pubCylinder_.publish(marker);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "te_surface_detection");
    SurfaceDetection surfaceDetection;
    ros::spin();
    return 0;
}




