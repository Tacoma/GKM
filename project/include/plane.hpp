#ifndef PLANE_H
#define PLANE_H


#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include "sophus/sim3.hpp"
#include "PcMeshBuilder.h"

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



class Plane {

public:
    typedef boost::shared_ptr<Plane> Ptr;

    Plane(float a, float b, float c, float d) : a_(a), b_(b), c_(c), d_(d) {
        init();
    }

    Plane(std::vector<float> coefficients) {
        if (coefficients.size() != 4) {
            ROS_ERROR("Plane with wrong number of coefficients created");
        }
        a_ = coefficients[0];
        b_ = coefficients[1];
        c_ = coefficients[2];
        d_ = coefficients[3];
        init();
    }
    ~Plane() {}

    void init() {
        calculateNormalForm(point_, normal_);
        plane = EigenPlane(normal_, point_);
        pointcloud_ = boost::make_shared<MyPointcloud>();
    }

    Eigen::Quaternionf getRotation() {
        Eigen::Quaternionf rotation;
        return rotation.FromTwoVectors(Eigen::Vector3f(0,0,1), normal_);
    }

    void getNormalForm(Eigen::Vector3f &point, Eigen::Vector3f &normal) {
        normal = normal_;
        point = point_;
    }


    Eigen::Vector3f rayIntersection(Eigen::Vector3f point, Eigen::Vector3f direction) {
        direction.normalize();
        Eigen::ParametrizedLine<float,3> pline = Eigen::ParametrizedLine<float,3>(point, direction);
        Eigen::Vector3f intersection = pline.intersectionPoint( plane );
        return intersection;
    }

    void addPc(MyPointcloud::Ptr cloud) {
        *pointcloud_ += *cloud;
    }

    float getA() {
        return a_;
    }
    float getB() {
        return b_;
    }
    float getC() {
        return c_;
    }
    float getD() {
        return d_;
    }

    void setPlane(std::vector<float> coefficients) {
        a_ = coefficients[0];
        b_ = coefficients[1];
        c_ = coefficients[2];
        d_ = coefficients[3];
        init();
    }


private:

    void calculateNormalForm(Eigen::Vector3f &point, Eigen::Vector3f &normal) {
        normal = Eigen::Vector3f(a_, b_, c_);
        float length = normal.norm();
        normal = normal / length;
        point = -d_/length*normal;

        //if (a_ != 0.0f && b_ != 0.0f && c_ != 0.0f) {
        //    //point = Eigen::Vector3f(-d_/(3*a_),-d_/(3*b_),-d_/(3*c_));
        //} else {
        //    point = Eigen::Vector3f(0,0,0);
        //}
    }


private:
    // General form
    float a_, b_, c_, d_;
    // Normal form
    Eigen::Vector3f point_, normal_;
    // Eigen
    EigenPlane plane;
    // Points
    MyPointcloud::Ptr pointcloud_;



};


#endif //PLANE_H
