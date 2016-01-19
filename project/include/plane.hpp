#ifndef PLANE_H
#define PLANE_H

//PointCloud#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>


#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include "sophus/sim3.hpp"




typedef Eigen::Hyperplane<float, 3> EigenPlane;

typedef pcl::PointXYZRGB MyPoint;

class MyPointcloud : public pcl::PointCloud<MyPoint>
{
public:
    typedef boost::shared_ptr<MyPointcloud> Ptr;
    int id;
};

typedef unsigned char uchar;
//typedef pcl::PointCloud<MyPoint> MyPointcloud;

// ------------------------- SimplePlane -------------------------

class SimplePlane {

public:
    typedef boost::shared_ptr<SimplePlane> Ptr;

    SimplePlane(std::vector<float> coefficients) {
        if (coefficients.size() != 4) {
            ROS_ERROR("Plane with wrong number of coefficients created");
        }
        a_ = coefficients[0];
        b_ = coefficients[1];
        c_ = coefficients[2];
        d_ = coefficients[3];
    }

    Eigen::VectorXf getCoefficients() {
        Eigen::VectorXf coeff(4);
        coeff << a_ , b_ , c_ , d_;
        return coeff;
    }

    void setCoefficients(const Eigen::VectorXf coefficients) {
        if (coefficients.size() != 4) {
            return;
        }
        a_ = coefficients(0);
        b_ = coefficients(1);
        c_ = coefficients(2);
        d_ = coefficients(3);
    }

    Eigen::Quaternionf getRotation() {
        Eigen::Quaternionf rotation;
    Eigen::Vector3f normal;
        Eigen::Vector3f point;
        calculateNormalForm(point, normal);
        return rotation.FromTwoVectors(Eigen::Vector3f(0,0,1), normal);
    }

    //TODO: test
    void transform(const Eigen::Matrix4f &transform) {
        Eigen::Vector3f normal;
        Eigen::Vector3f point;
        calculateNormalForm(point, normal);
        Eigen::Vector4f normal_hom(normal[0], normal[1], normal[2], 0);
        Eigen::Vector4f point_hom(point[0], point[1], point[2], 1);
        normal_hom = transform * normal_hom;
        point_hom = transform * point_hom;
        for (int i = 0; i < 3; i++) {
            normal[i] = normal_hom[i];
            point[i] = point_hom[i] / point_hom[3];
        }
        calculateParameterForm(point, normal);
    }

    void transformPlane(const Eigen::Matrix4f &transform, Eigen::Vector3f &point_inout, Eigen::Vector3f &normal_inout) {
        Eigen::Vector4f normal_hom(normal_inout[0], normal_inout[1], normal_inout[2], 0);
        Eigen::Vector4f point_hom(point_inout[0], point_inout[1], point_inout[2], 1);
        normal_hom = transform * normal_hom;
        point_hom = transform * point_hom;
        for (int i = 0; i < 3; i++) {
            normal_inout[i] = normal_hom[i];
            point_inout[i] = point_hom[i] / point_hom[3];
        }
    }
    
    // returns closest point to point_in on the plane
    Eigen::Vector3f rayIntersection(Eigen::Vector3f point_in, Eigen::Vector3f direction_in) {
        direction_in.normalize();
        Eigen::Vector3f planeNormal;
        Eigen::Vector3f planePoint;
        calculateNormalForm(planePoint, planeNormal);
        EigenPlane plane = EigenPlane(planeNormal, planePoint);
        Eigen::ParametrizedLine<float,3> pline = Eigen::ParametrizedLine<float,3>(point_in, direction_in);
        Eigen::Vector3f intersection = pline.intersectionPoint(plane);
        return intersection;
    }

    void calculateNormalForm(Eigen::Vector3f &point_out, Eigen::Vector3f &normal_out) {
        normal_out = Eigen::Vector3f(a_, b_, c_);
        float length = normal_out.norm();
        normal_out = normal_out / length;
        point_out = -d_/length * normal_out;

        // Checking the direction of the normal
        int scalerProduct = point_out.dot(normal_out);
        if(scalerProduct < 0){
            normal_out = -normal_out;
        }

    }


private:

    void calculateParameterForm(const Eigen::Vector3f &point, const Eigen::Vector3f &normal) {
        a_ = normal[0];
        b_ = normal[1];
        c_ = normal[2];

        d_ = - normal.dot(point);
    }


// Member variables ----------
private:
    float a_, b_, c_, d_;


};

// // ------------------------- Plane -------------------------
// // deprecated
// class Plane {
// 
// public:
//     typedef boost::shared_ptr<Plane> Ptr;
// 
//     Plane(float a, float b, float c, float d) : a_(a), b_(b), c_(c), d_(d) {
//         init();
//     }
// 
//     Plane(std::vector<float> coefficients) {
//         if (coefficients.size() != 4) {
//             ROS_ERROR("Plane with wrong number of coefficients created");
//         }
//         a_ = coefficients[0];
//         b_ = coefficients[1];
//         c_ = coefficients[2];
//         d_ = coefficients[3];
//         init();
//     }
// 
//     ~Plane() {}
// 
//     void init() {
//         pointcloud_ = boost::make_shared<MyPointcloud>();
//         hull_ = boost::make_shared<MyPointcloud>();
//         calculateNormalForm(point_, normal_);
//         plane_ = EigenPlane(normal_, point_);
//     }
// 
//     Eigen::Vector3f rayIntersection(Eigen::Vector3f point, Eigen::Vector3f direction) {
//         direction.normalize();
//         Eigen::ParametrizedLine<float,3> pline = Eigen::ParametrizedLine<float,3>(point, direction);
//         Eigen::Vector3f intersection = pline.intersectionPoint( plane_ );
//         return intersection;
//     }
// 
//     Eigen::Quaternionf getRotation() {
//         Eigen::Quaternionf rotation;
//         return rotation.FromTwoVectors(Eigen::Vector3f(-1,0,0), normal_);
//     }
// 
//     void getNormalForm(Eigen::Vector3f &point, Eigen::Vector3f &normal) {
//         normal = normal_;
//         point = point_;
//     }
//     float getA() {
//         return a_;
//     }
//     float getB() {
//         return b_;
//     }
//     float getC() {
//         return c_;
//     }
//     float getD() {
//         return d_;
//     }
//     Eigen::VectorXf getCoefficients() {
//         Eigen::VectorXf coeff(4);
//         coeff << a_ , b_ , c_ , d_;
//         return coeff;
//     }
// 
// 
//     void setPlane(const Eigen::VectorXf coefficients) {
//         if (coefficients.size() != 4) {
//             return;
//         }
//         a_ = coefficients(0);
//         b_ = coefficients(1);
//         c_ = coefficients(2);
//         d_ = coefficients(3);
//         calculateNormalForm(point_, normal_);
//         plane_ = EigenPlane(normal_, point_);
//     }
// 
//     void addPointcoud(MyPointcloud::Ptr pc) {
//         *pointcloud_ += *pc;
//         createHull();
//     }
// 
//     MyPointcloud::ConstPtr getPointcloud() {
//         MyPointcloud::ConstPtr cloud = pointcloud_;
//         return cloud;
//     }
// 
//     MyPointcloud::ConstPtr getHull() {
//         MyPointcloud::ConstPtr cloud = hull_;
//         return cloud;
//     }
// 
//     void refitPlane() {
//         applyEuclideanFilter();
//         pcl::SampleConsensusModelPlane<MyPoint>::Ptr model =
//             boost::make_shared<pcl::SampleConsensusModelPlane<MyPoint> >(pointcloud_);
// 
//         Eigen::VectorXf coefficients(4), coefficients_refined(4);
//         coefficients << a_, b_, c_, d_;
//         model->optimizeModelCoefficients(*(model->getIndices()), coefficients, coefficients_refined);
//         setPlane(coefficients_refined);
//     }
// 
// private:
// 
//     void calculateNormalForm(Eigen::Vector3f &point_out, Eigen::Vector3f &normal_out) {
//         normal_out = Eigen::Vector3f(a_, b_, c_);
//         float length = normal_out.norm();
//         normal_out = normal_out / length;
//         point_out = -d_/length * normal_out;
//     }
// 
//     void createHull() {
//         // Create a Concave Hull representation of the projected inliers
//         pcl::ConvexHull<MyPoint> hull;
//         hull.setInputCloud (pointcloud_);
//         hull.setDimension(2);
//         hull.reconstruct (*hull_);
// //         std::stringstream ss;
// //         ss << "Hull: " << hull_->size();
// //         for (int i = 0; i < hull_->size(); i++) {
// //             if (i%3==0) ss << "\n";
// //             ss << hull_->points[i] << ", ";
// //         }
// //         ROS_INFO_STREAM(ss.str());
//     }
// 
//     void applyEuclideanFilter() {
//         pcl::search::KdTree<MyPoint>::Ptr kdTree =
//             boost::make_shared<pcl::search::KdTree<MyPoint> >(new pcl::search::KdTree<MyPoint>);
//         kdTree->setInputCloud(pointcloud_);
//         // Euclidean clustering object. (flood fill)
//         pcl::EuclideanClusterExtraction<MyPoint> clustering;
//         // Set cluster tolerance to 2cm
//         clustering.setClusterTolerance(0.02);
//         // Set the minimum and maximum number of points that a cluster can have.
//         clustering.setMinClusterSize(0);
//         clustering.setMaxClusterSize(pointcloud_->size());
//         clustering.setSearchMethod(kdTree);
//         clustering.setInputCloud(pointcloud_);
//         std::vector<pcl::PointIndices> clusters;
//         clustering.extract(clusters);
//         pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>(clusters[0]);
// 
// 
//         //Extract largest cluster
//         MyPointcloud::Ptr cloud_filtered = boost::make_shared<MyPointcloud>();
//         pcl::ExtractIndices<MyPoint> extract;
//         extract.setInputCloud(pointcloud_);
//         extract.setIndices(inliers);
//         extract.setNegative(false);
//         extract.filter(*cloud_filtered);
//         pointcloud_.swap(cloud_filtered);
//     }
// 
// public:
//     Eigen::Vector3f color_;
// 
// private:
//     // General Form
//     float a_, b_, c_, d_;
//     // Normal Form
//     Eigen::Vector3f point_, normal_;
//     // Eigen
// 
//     EigenPlane plane_;
// 
//     // Points
//     MyPointcloud::Ptr pointcloud_;
//     MyPointcloud::Ptr hull_;
// };


#endif //PLANE_H
