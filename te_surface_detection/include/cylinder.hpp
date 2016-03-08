#ifndef CYLINDER_H
#define CYLINDER_H

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
#include "typedef.h"


// ------------------------- Cylinder -------------------------
class Cylinder {

public:
    typedef boost::shared_ptr<Cylinder> Ptr;

    Cylinder(std::vector<float> coefficients) 
    {
        if (coefficients.size() != 7) {
            ROS_ERROR("Plane with wrong number of coefficients created");
	    return;
        }
        point_ = Eigen::Vector4f(coefficients[0], coefficients[1], coefficients[2], 1);
	//dir_
	radius_ = coefficients[6];
    }

    Eigen::VectorXf getCoefficients() 
    {
        Eigen::VectorXf coeff(7);
        coeff << point_[0], point_[1], point_[2], dir_[0], dir_[1], dir_[2], radius_;
        return coeff;
    }

    void setCoefficients(const Eigen::VectorXf coefficients) 
    {
        if (coefficients.size() != 7) {
	    ROS_ERROR("Plane with wrong number of coefficients created");
            return;
        }
        point_ = Eigen::Vector4f(coefficients[0], coefficients[1], coefficients[2], 1);
	//dir_
	radius_ = coefficients[6];
    }

//     Eigen::Quaternionf getRotation() //TODO
//     {
// 	return Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1,0,0), dir_);
//     }

    void transform(const Eigen::Matrix4f &transform) 
    {
        point_ = transform * point_;
        dir_ = transform * dir_;
    }
    
    // returns closest point to point_in on the plane
    Eigen::Vector3f rayIntersection(Eigen::Vector3f point_in, Eigen::Vector3f direction_in) //TODO
    {
//         direction_in.normalize();
//         Eigen::Vector3f planeNormal;
//         Eigen::Vector3f planePoint;
//         calculateNormalForm(planePoint, planeNormal);
//         EigenPlane plane = EigenPlane(planeNormal, planePoint);
//         Eigen::ParametrizedLine<float,3> pline = Eigen::ParametrizedLine<float,3>(point_in, direction_in);
//         Eigen::Vector3f intersection = pline.intersectionPoint(plane);
//         return intersection;
    }


private:

    
// Member variables ----------
private:
    Eigen::Vector4f point_, dir_;
    float radius_;
    
    


};


#endif //CYLINDER_H
