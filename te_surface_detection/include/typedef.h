#ifndef TYPEDEF_H
#define TYPEDEF_H

typedef pcl::PointXYZRGB MyPoint;
typedef pcl::Normal MyNormal;

class MyPointcloud : public pcl::PointCloud<MyPoint>
{
public:
    typedef boost::shared_ptr<MyPointcloud> Ptr;
    int id;
};

class MyNormalcloud : public pcl::PointCloud<MyNormal>
{
public:
    typedef boost::shared_ptr<MyNormalcloud> Ptr;
};

typedef unsigned char uchar;

#endif //TYPEDEF_H