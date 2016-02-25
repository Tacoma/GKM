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

    SimplePlane(std::vector<float> coefficients) 
    {
        if (coefficients.size() != 4) {
            ROS_ERROR("Plane with wrong number of coefficients created");
        }
        a_ = coefficients[0];
        b_ = coefficients[1];
        c_ = coefficients[2];
        d_ = coefficients[3];
    }

    Eigen::VectorXf getCoefficients() 
    {
        Eigen::VectorXf coeff(4);
        coeff << a_ , b_ , c_ , d_;
        return coeff;
    }

    void setCoefficients(const Eigen::VectorXf coefficients) 
    {
        if (coefficients.size() != 4) {
            return;
        }
        a_ = coefficients(0);
        b_ = coefficients(1);
        c_ = coefficients(2);
        d_ = coefficients(3);
    }

    Eigen::Quaternionf getRotation() 
    {
	Eigen::Vector3f normal;
        Eigen::Vector3f point;
        calculateNormalForm(point, normal);
	return Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1,0,0), normal);
    }

    void transform(const Eigen::Matrix4f &transform) 
    {
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

    void transformPlane(const Eigen::Matrix4f &transform, Eigen::Vector3f &point_inout, Eigen::Vector3f &normal_inout) 
    {
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
    Eigen::Vector3f rayIntersection(Eigen::Vector3f point_in, Eigen::Vector3f direction_in) 
    {
        direction_in.normalize();
        Eigen::Vector3f planeNormal;
        Eigen::Vector3f planePoint;
        calculateNormalForm(planePoint, planeNormal);
        EigenPlane plane = EigenPlane(planeNormal, planePoint);
        Eigen::ParametrizedLine<float,3> pline = Eigen::ParametrizedLine<float,3>(point_in, direction_in);
        Eigen::Vector3f intersection = pline.intersectionPoint(plane);
        return intersection;
    }

    void calculateNormalForm(Eigen::Vector3f &point_out, Eigen::Vector3f &normal_out) 
    {
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


#endif //PLANE_H
