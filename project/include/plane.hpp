#ifndef PLANE_H
#define PLANE_H


#include <Eigen/Core>
#include <boost/concept_check.hpp>

typedef Eigen::Hyperplane<float, 3> EigenPlane;



class Plane {

public:
    Plane(float a, float b, float c, float d) : a_(a), b_(b), c_(c), d_(d) {
        Eigen::Vector3f p, n;
        getNormalForm(p,n);
        plane = EigenPlane(p,n);
    }

    Plane(Eigen::Vector4f coord) : a_(coord[0]), b_(coord[1]), c_(coord[2]), d_(coord[3]) {
	Eigen::Vector3f p, n;
        getNormalForm(p,n);
        plane = EigenPlane(n,p);
    }
    ~Plane() {}

    void getNormalForm(Eigen::Vector3f &p, Eigen::Vector3f &n) {
        n = Eigen::Vector3f(a_, b_, c_);
	// get middle parameter for the point (x = e1 to e3)
	float a = abs(a); float b = abs(b); float c = abs(c);
        if ( a > b && a < c ) {
	    p = Eigen::Vector3f(d_/a_,0,0);
        } else if (b > c) {
            p = Eigen::Vector3f(0,0,d_/c_);
        } else {
            p = Eigen::Vector3f(0,d_/b_,0);
        }
    }
    
    Eigen::Quaternionf getRotation() {
	Eigen::Vector3f p, n;
	getNormalForm(p,n);
	Eigen::Quaternionf rotation; 
	return rotation.FromTwoVectors(Eigen::Vector3f(0,0,1), n);
    }
	
    
    Eigen::Vector3f rayIntersection(Eigen::Vector3f p, Eigen::Vector3f d) {
	Eigen::ParametrizedLine<float,3> pline = Eigen::ParametrizedLine<float,3>::Through(p,p+d);
	double t = pline.intersection( plane );
	Eigen::Vector3f intersection =  p + t*d;
	return intersection;
    }


private:
    // coordinate form
    float a_, b_, c_, d_;
    // Eigen
    EigenPlane plane;

};


#endif //PLANE_H
