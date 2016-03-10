#include <iostream>

#include <boost/thread/thread.hpp>
// PCL
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
// PCL CONVERSION
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// typedefs
typedef pcl::PointXYZRGB MyPoint;
typedef pcl::Normal MyNormal;

class MyPointcloud : public pcl::PointCloud<MyPoint>
{
public:
    typedef boost::shared_ptr<MyPointcloud> Ptr;
};

class MyNormalcloud : public pcl::PointCloud<MyNormal>
{
public:
    typedef boost::shared_ptr<MyNormalcloud> Ptr;
};

// global variables
boost::shared_ptr<ros::NodeHandle> nh_;
boost::shared_ptr<ros::NodeHandle> private_nh_;
ros::Subscriber sub_pc_;

MyPointcloud::Ptr point_cloud_ptr_;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

////// VISUALIZATION //////
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // Open 3D viewer and add point cloud
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0.3, 0.3, 0.3);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
    // Open 3D viewer and add point cloud and normals
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    // Add shapes at cloud points
    viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
                                     cloud->points[cloud->size() - 1], "line");
    viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

    // Add shapes at other locations
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (1.0);
    coeffs.values.push_back (0.0);
    viewer->addPlane (coeffs, "plane");
    coeffs.values.clear ();
    coeffs.values.push_back (0.3);
    coeffs.values.push_back (0.3);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (1.0);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (5.0);
    viewer->addCone (coeffs, "cone");

    return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
    // Open 3D viewer and add point cloud and normals
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud1", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "cloud2", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");
    viewer->addCoordinateSystem (1.0);

    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

    return (viewer);
}


unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getKeySym () == "r" && event.keyDown ()) {
        std::cout << "r was pressed => removing all text" << std::endl;

        char str[512];
        for (unsigned int i = 0; i < text_id; ++i) {
            sprintf (str, "text#%03d", i);
            viewer->removeShape (str);
        }
        text_id = 0;
    }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease) {
        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

        char str[512];
        sprintf (str, "text#%03d", text_id ++);
        viewer->addText ("clicked here", event.getX (), event.getY (), str);
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);

    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

    return (viewer);
}

////// ROS FUNCTIONS //////

inline void colorPointcloud(MyPointcloud::Ptr cloud_in, uint8_t r, uint8_t g, uint8_t b) {
    for(int i=0; i<cloud_in->points.size(); i++) {
        cloud_in->points[i].r = r;
        cloud_in->points[i].g = g;
        cloud_in->points[i].b = b;
    }
}

void processPointcloudMsg(const boost::shared_ptr<const sensor_msgs::PointCloud2> msg) {
    std::cout << "Generating point cloud from pointcloud message.\n";

    // conversion from sensor_msgs::PointCloud2 --> MyPointcloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    point_cloud_ptr_ = boost::make_shared<MyPointcloud>();
    pcl::fromPCLPointCloud2(pcl_pc2,*point_cloud_ptr_);
    std::cout << "Number of points: " << point_cloud_ptr_->points.size() << std::endl;
    std::cout << point_cloud_ptr_->points[0].x << ", " << point_cloud_ptr_->points[0].y << ", " << point_cloud_ptr_->points[0].z << std::endl;

    // calculate surface normals
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (point_cloud_ptr_);
    // search method
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // search radius
    ne.setRadiusSearch (0.1);
    ne.compute(*cloud_normals);

    // reset viewer
//    viewer_ = rgbVis(point_cloud_ptr_);
    viewer_->removePointCloud("cloud");
    viewer_->removePointCloud("normals");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr_);
    viewer_->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr_, rgb, "cloud");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer_->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (point_cloud_ptr_, cloud_normals, 10, 0.05, "normals");
}


////// MAIN //////
int main (int argc, char** argv)
{
    // ROS
    ros::init(argc, argv, "pcl_visualizer");
    nh_.reset(new ros::NodeHandle(""));
    private_nh_.reset(new ros::NodeHandle("~"));
    sub_pc_ = nh_->subscribe<sensor_msgs::PointCloud2>("/euroc_hex/te_surface_detection/meshPc", 10, processPointcloudMsg);



    // create viewer
    point_cloud_ptr_ = boost::make_shared<MyPointcloud>();
    viewer_ = rgbVis(point_cloud_ptr_);


    // Main loop
    while (!viewer_->wasStopped()) {
        ros::spinOnce();
        viewer_->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
    }
}
