#ifndef TYPEDEF_H
#define TYPEDEF_H

typedef pcl::PointXYZRGB MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointcloud;
typedef pcl::Normal MyNormal;
typedef pcl::PointCloud<MyNormal> MyNormalcloud;

//class MyNormalcloud : public pcl::PointCloud<MyNormal>
//{
//public:
//    typedef boost::shared_ptr<MyNormalcloud> Ptr;
//};

typedef unsigned char uchar;

#endif //TYPEDEF_H
