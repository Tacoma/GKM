#ifndef PLANE_H
#define PLANE_H


#include <Eigen/Core>
#include <boost/concept_check.hpp>

typedef Eigen::Hyperplane<float, 3> EigenPlane;



class Plane {

public:
    Plane(float a, float b, float c, float d) : a_(a), b_(b), c_(c), d_(d) {
        Eigen::Vector3f p, n;
        normalForm(p,n);
        plane = EigenPlane(p,n);
    }

    Plane(Eigen::Vector4f coord) : a_(coord[0]), b_(coord[1]), c_(coord[2]), d_(coord[3]) {}
    ~Plane() {}

    void normalForm(Eigen::Vector3f &p, Eigen::Vector3f n) {
        n = Eigen::Vector3f(a_, b_, c_);
        if ( a_ > b_ ) {
            if ( a_ > c_ ) {
                p = Eigen::Vector3f(d_/a_,0,0);
            } else {
                p = Eigen::Vector3f(0,0,d_/c_);
            }
        } else if (b_ > c_) {
            p = Eigen::Vector3f(0,d_/b_,0);
        } else {
            p = Eigen::Vector3f(0,0,d_/c_);
        }
    }
    
    Eigen::Vector3f rayIntersection(Eigen::Vector3f p, Eigen::Vector3f d) {
	d = d.normalized();
	Eigen::ParametrizedLine<float,3> pline = Eigen::ParametrizedLine<float,3>::Through(p,p+d);
	double t = pline.intersection( plane ) ;
	Eigen::Vector3f intersection =  p + t*(d);
	return intersection;
    }


private:
    // coordinate form
    float a_, b_, c_, d_;
    // Eigen
    EigenPlane plane;

};


#endif //PLANE_H
