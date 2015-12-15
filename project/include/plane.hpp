#ifndef PLANE_H
#define PLANE_H


#include <Eigen/Core>
//#include <boost/concept_check.hpp>
#include "sophus/sim3.hpp"

typedef Eigen::Hyperplane<float, 3> EigenPlane;



class Plane {

public:
    Plane(float a, float b, float c, float d) : a_(a), b_(b), c_(c), d_(d) {
        calculateNormalForm(point_,normal_);
        plane = EigenPlane(normal_,point_);
    }

    Plane(Eigen::Vector4f coord) : a_(coord[0]), b_(coord[1]), c_(coord[2]), d_(coord[3]) {
        calculateNormalForm(point_, normal_);
        plane = EigenPlane(normal_, point_);
    }
    ~Plane() {}


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


private:

    void calculateNormalForm(Eigen::Vector3f &point, Eigen::Vector3f &normal) {
        normal = Eigen::Vector3f(a_, b_, c_);
        float length = normal.norm();
        normal = normal / length;
//         // get middle parameter for the point (x = e1 to e3)
//         float a = abs(a);
//         float b = abs(b);
//         float c = abs(c);
//         if ( a > b && a < c ) {
//             p = Eigen::Vector3f(-d_/a_,0,0);
//         } else if (b > c) {
//             p = Eigen::Vector3f(0,0,-d_/c_);
//         } else {
//             p = Eigen::Vector3f(0,-d_/b_,0);
//         }
//         p = Eigen::Vector3f(d_/length,d_/length,d_/length);
        if (a_ != 0.0f && b_ != 0.0f && c_ != 0.0f) {
	    point = -d_/length*normal;
            //point = Eigen::Vector3f(-d_/(3*a_),-d_/(3*b_),-d_/(3*c_));
        } else {
            point = Eigen::Vector3f(0,0,0);
        }
    }


private:
    // General form
    float a_, b_, c_, d_;
    // Normal form
    Eigen::Vector3f point_, normal_;
    // Eigen
    EigenPlane plane;



};


#endif //PLANE_H
