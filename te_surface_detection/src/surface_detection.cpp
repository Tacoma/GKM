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

#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
//#include <jsk_recognition_msgs/PolygonArray.h>

#include <iostream>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "surface_detection.h"


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



SurfaceDetection::SurfaceDetection() :
    scaledDepthVarTH_(-1),
    absDepthVarTH_(-1),
    minNearSupport_(0),
    sparsifyFactor_(0),
    distanceThreshold_(0.003),
    windowSize_(640),
    windowPosY_(0),
    minPointsForEstimation_(3),
    noisePercentage_(0),
    maxPlanesPerCloud_(1)
{
    nh_ = ros::NodeHandle("");
    private_nh_ = ros::NodeHandle("~");

    // subscriber and publisher
    sub_keyframes_ = nh_.subscribe(nh_.resolveName("lsd_slam/keyframes"), 10, &SurfaceDetection::processMessage, this);
    sub_liveframes_ = nh_.subscribe(nh_.resolveName("lsd_slam/liveframes"), 10, &SurfaceDetection::processMessage, this);
    sub_stickToSurface_ = nh_.subscribe<std_msgs::Bool>("controller/stickToSurface", 10, &SurfaceDetection::setSearchPlane, this);
    pub_pc_ = private_nh_.advertise< pcl::PointCloud<MyPoint> >("meshPc", 10);
    //pub_markers_ = private_nh_.advertise< jsk_recognition_msgs::PolygonArray>("Hull", 10);
    pub_tf_ = private_nh_.advertise<geometry_msgs::TransformStamped>("plane", 10);

    //ros param
    private_nh_.param("mavTFName", mavTFName_, std::string("world"));

    // init
#ifdef VISUALIZE
    pcVisSurfaces_ = boost::make_shared<MyPointcloud>();
    pcVisOutliers_ = boost::make_shared<MyPointcloud>();
#endif
    pcVisDebug_ = boost::make_shared<MyPointcloud>();
    searchPlane_ = false;
    planeExists_ = false;
    opticalToSensor_ = Eigen::Matrix4f::Identity();
    // (row, column)
    opticalToSensor_ (0,0) = 0;
    opticalToSensor_ (0,1) = 0;
    opticalToSensor_ (0,2) = 1;
    opticalToSensor_ (1,0) =-1;
    opticalToSensor_ (1,1) = 0;
    opticalToSensor_ (1,2) = 0;
    opticalToSensor_ (2,0) = 0;
    opticalToSensor_ (2,1) =-1;
    opticalToSensor_ (2,2) = 0;
    status = -1;

    sensorToMav_ = Eigen::Affine3f::Identity();
    sensorToMav_.translation() << 0.133,0,0.0605-0.117;
    sensorToMav_.rotate(Eigen::Quaternionf(0.98481,0,0.17365,0));

    // Setting up Dynamic Reconfiguration
    dynamic_reconfigure::Server<te_surface_detection::generalConfig>::CallbackType f;
    f = boost::bind(&SurfaceDetection::configCallback, this, _1, _2);
    server_.setCallback(f);

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
    searchPlane_ = false;
    planeExists_ = false;
    status = -1;
}


void SurfaceDetection::setSearchPlane(const std_msgs::Bool::ConstPtr& msg)
{
    if (searchPlane_ == msg->data) {
        return;
    }
    reset();
    searchPlane_ = msg->data;
    if (lastMsg_) {
		processMessage(lastMsg_);
    }
}


void SurfaceDetection::processMessage(const lsd_slam_msgs::keyframeMsgConstPtr msg)
{
    if (msg->isKeyframe) {
        lastMsg_ = msg;
        pcVisDebug_ = boost::make_shared<MyPointcloud>();

        if(searchPlane_) {
            MyPointcloud::Ptr cloud = boost::make_shared<MyPointcloud>();

            //Find the largest plane only if no plane exists
            if(!planeExists_) {
                Eigen::Vector2i min, max;
                // Search in small window
                getProcessWindow(min, max, windowSize_, windowPosY_, msg->width, msg->height);
                processPointcloud(msg, cloud, min, max);
                findPlanes(cloud, 1);
            }

            // We don't want to call refine Plane for the same message
            else {
                // Transform and refine the plane with new inliers, searching in whole pointcloud
                processPointcloud(msg, cloud);
                refinePlane(cloud);

                //publish plane
                publishPlane();
            }

            // rememember pose for plane transformation
            lastPose_ = currentPose_;
	    
	    publishPointclouds();
        }
        
    // !isKeyframe
    } else {
        // check for reset
        if (lastFrameId_ > msg->id) {
            ROS_INFO_STREAM("detected backward-jump in id (" << lastFrameId_ << " to " << msg->id << "), resetting!");
            lastFrameId_ = 0;
            reset();

        }
        lastFrameId_ = msg->id;
    }

}


void SurfaceDetection::processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg, MyPointcloud::Ptr cloud)
{
    processPointcloud(msg, cloud, Eigen::Vector2i(0,0), Eigen::Vector2i(msg->width, msg->height));
}

void SurfaceDetection::processPointcloud(const lsd_slam_msgs::keyframeMsgConstPtr msg,
                                      MyPointcloud::Ptr cloud,
                                      Eigen::Vector2i min,
                                      Eigen::Vector2i max)
{
    memcpy(currentPose_.data(), msg->camToWorld.data(), 7*sizeof(float));

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

    clampProcessWindow(min, max, width, height);
    float worldScale = currentPose_.scale();

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
    //pcl::transformPointCloud(*cloud,*cloud,currentPose_.matrix());

    delete[] originalInput_;
}



void SurfaceDetection::refinePlane(MyPointcloud::Ptr cloud)
{
    // Init
    MyPointcloud::Ptr pcFiltered = boost::make_shared<MyPointcloud>();

    // Create the planar model and set all the parameters
    pcl::SampleConsensusModelPlane<MyPoint>::Ptr model = boost::make_shared<pcl::SampleConsensusModelPlane<MyPoint> >(cloud);
    pcl::ExtractIndices<MyPoint> extract;
    boost::shared_ptr<std::vector<int> > inliers = boost::make_shared<std::vector<int> >();
    model->setInputCloud(cloud);


    Eigen::VectorXf coefficients, coefficients_refined;
    Eigen::Matrix4f transform = currentPose_.matrix().inverse() * lastPose_.matrix();

    // Transform plane into new frame and get coefficients
    plane_->transform(transform);
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

    *pcVisDebug_ += *pcFiltered;
}



/**
 * looks for numSurfaces planes in cloud_in and returns all points in these planes,
 * colored to the corresponding plane.
 */
void SurfaceDetection::findPlanes(MyPointcloud::Ptr cloud, unsigned int numSurfaces)
{
    std::cout << "Searching " << numSurfaces << " plane(s) in " << cloud->size() << " points.";
    // Init
    MyPointcloud::Ptr pcCropped = cloud;
    int size = cloud->size();
    // Allocation
    MyPointcloud::Ptr pcFiltered = boost::make_shared<MyPointcloud>();
#ifdef VISUALIZE
    MyPointcloud::Ptr pcVisSurfaces = boost::make_shared<MyPointcloud>();
#endif


    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<MyPoint> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<MyPoint> extract;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
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
        plane_.reset(new SimplePlane(coefficients->values));
        planeExists_ = true;
        i++;
        std::cout << "| " << i;
        switch (i) {
        case  1:  std::cout << "st"; break;
        case  2:  std::cout << "nd"; break;
        case  3:  std::cout << "rd"; break;
        default:  std::cout << "th"; break;
        }
        std::cout << " plane found";
    }
    std::cout << std::endl;

#ifdef VISUALIZE
    // add pointcloud keyframe to the accumulated pointclouds depending on planar property
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {  listener.lookupTransform(mavTFName_, "/world", ros::Time::now(), transform);  }
    catch (tf::TransformException ex){  ROS_ERROR("%s",ex.what());  }
    Eigen::Affine3d mavToWorld;
	tf::transformTFToEigen(transform, mavToWorld);
    //pcl::transformPointCloud(*pcVisSurfaces, *pcVisSurfaces, mavToWorld * sensorToMav_ * opticalToSensor_);
    //pcl::transformPointCloud(*pcCropped, *pcCropped, mavToWorld* sensorToMav_ * opticalToSensor_);
    *pcVisSurfaces_ = *pcVisSurfaces;
    *pcVisOutliers_ = *pcCropped;
#endif
}
// -------------------------------------------------------------------------------------------------------------------------------------------------------


/**
 * looks for numSurfaces planes in cloud_in and returns all points in these planes,
 * colored to the corresponding plane.
 */
void SurfaceDetection::findCylinder(MyPointcloud::Ptr cloud, unsigned int numSurfaces)
{
    std::cout << "Searching " << numSurfaces << " cylinder in " << cloud->size() << " points.";
    // Init
    MyPointcloud::Ptr pcCropped = cloud;
    int size = cloud->size();
    // Allocation
    MyPointcloud::Ptr pcFiltered = boost::make_shared<MyPointcloud>();
#ifdef VISUALIZE
    MyPointcloud::Ptr pcVisSurfaces = boost::make_shared<MyPointcloud>();
#endif
	MyNormalcloud::Ptr normalsFiltered  = boost::make_shared<MyNormalcloud>();
	MyNormalcloud::Ptr normalsCropped = boost::make_shared<MyNormalcloud>();

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
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.003);
    seg.setRadiusLimits (0, 1);

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
        // add newly created pointcloud in the union pointcloud
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

        // Create plane and add points
        //plane_.reset(new SimplePlane(coefficients->values));
        //planeExists_ = true;
        i++;
        std::cout << "| " << i;
        switch (i) {
        case  1:  std::cout << "st"; break;
        case  2:  std::cout << "nd"; break;
        case  3:  std::cout << "rd"; break;
        default:  std::cout << "th"; break;
        }
        std::cout << " cylinder found";
    }
    std::cout << std::endl;

#ifdef VISUALIZE
    // add pointcloud keyframe to the accumulated pointclouds depending on planar property
    //pcl::transformPointCloud(*pcVisSurfaces, *pcVisSurfaces, ... * sensorToMav_ * opticalToSensor_);
    //pcl::transformPointCloud(*pcCropped, *pcCropped, ... * sensorToMav_ * opticalToSensor_);
    *pcVisSurfaces_ = *pcVisSurfaces;
    *pcVisOutliers_ = *pcCropped;
#endif
}




inline void SurfaceDetection::colorPointcloud(MyPointcloud::Ptr cloud_in, Eigen::Vector3f color)
{
    for(int i=0; i<cloud_in->points.size(); i++) {
        cloud_in->points[i].r = color.x();
        cloud_in->points[i].g = color.y();
        cloud_in->points[i].b = color.z();
    }
}


inline void SurfaceDetection::getProcessWindow(Eigen::Vector2i &min_out, Eigen::Vector2i &max_out,
        float windowSize, float windowOffset,
        int width, int height)
{
    int radius = windowSize_/2;
    min_out.x() = width/2  - radius;
    max_out.x() = width/2  + radius;
    min_out.y() = height/2 - radius + windowPosY_;
    max_out.y() = height/2 + radius + windowPosY_;
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
    MyPointcloud::Ptr pcUnion = boost::make_shared<MyPointcloud>();


    *pcUnion += *pcVisDebug_;
    // transform pointcloud from optical to MAV (no optical tf published)
    //pcl::transformPointCloud(*pcUnion, *pcUnion, ... * sensorToMav_ * opticalToSensor_);
#ifdef VISUALIZE
    *pcUnion += *pcVisSurfaces_;
    *pcUnion += *pcVisOutliers_;
#endif

    // Publish debug
    sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*pcUnion, *msg);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = mavTFName_;
    pub_pc_.publish(msg);
}

void SurfaceDetection::publishPolygons()
{
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

void SurfaceDetection::publishPlane()
{
    // calculate Plane position
    Eigen::Vector3f cam_t(0,0,0);
    Eigen::Vector3f plane_point, plane_normal;
    plane_->calculateNormalForm(plane_point, plane_normal);
    Eigen::Vector3f intersection = plane_->rayIntersection(cam_t, plane_normal);
    Eigen::Quaternionf plane_rot = plane_->getRotation();



    // publish
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = mavTFName_;
    tf.transform.translation.x = intersection.x();
    tf.transform.translation.y = intersection.y();
    tf.transform.translation.z = intersection.z();
    tf.transform.rotation.x = plane_rot.x();
    tf.transform.rotation.y = plane_rot.y();
    tf.transform.rotation.z = plane_rot.z();
    tf.transform.rotation.w = plane_rot.w();
    pub_tf_.publish(tf);
}


void SurfaceDetection::configCallback(te_surface_detection::generalConfig &config, uint32_t level)
{

    std::cout << "Configurating." << std::endl;

    scaledDepthVarTH_ = pow(10.0f , config.scaledDepthVarTH );
    absDepthVarTH_ = pow(10.0f, config.absDepthVarTH);
    minNearSupport_ = config.minNearSupport;
    sparsifyFactor_ = config.sparsifyFactor;
    distanceThreshold_ = config.distanceThreshold;
    windowSize_ = config.windowSize;
    windowPosY_ = config.windowPosY;
    // debug parameters
    //minPointsForEstimation_ = config.minPointsForEstimation;
    //noisePercentage_ = 1-config.planarPercentage;
    //maxPlanesPerCloud_ = config.maxPlanesPerCloud;

    // apply changes to last message
    if (lastMsg_) {
        processMessage(lastMsg_);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "te_surface_detection");
    SurfaceDetection surfaceDetection;
    ros::spin();
    return 0;
}




