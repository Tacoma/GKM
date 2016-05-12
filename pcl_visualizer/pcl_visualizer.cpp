#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// additional pcl headers
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#define KEYSYM_ARROW_LEFT "Left"
#define KEYSYM_ARROW_RIGHT "Right"

// global variables
static int counter_ = 0;
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_(new pcl::PointCloud<pcl::Normal>);
static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_(new pcl::visualization::PCLVisualizer ("3D Viewer"));
static std::ifstream coefficientsFile_;
static std::vector<pcl::ModelCoefficients::Ptr> coefficients_;

// prototypes
void calculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_ptr, float radius=0.1f);
void loadPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_ptr, int idx);
void update();

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    if(event.keyDown()) {
        if(event.getKeySym() == KEYSYM_ARROW_LEFT)
            counter_ = (counter_-1) >= 0 ? (counter_-1) : 0;
        else if(event.getKeySym() == KEYSYM_ARROW_RIGHT)
            counter_++;
        else if(event.getKeySym() == "r")
            counter_ = 0;

        if(counter_ >= coefficients_.size()) \
            counter_ = coefficients_.size()-1;

        std::cout << "Index: " << counter_ << std::endl;
        loadPointCloud(point_cloud_ptr_, cloud_normals_, counter_);
        update();
    }
}

void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease) {
        // TODO
    }
}

void calculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_ptr, float radius)
{
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud_ptr);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);
    ne.compute(*normals_cloud_ptr);
}

void loadPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_ptr, int idx)
{
    std::stringstream filename;
    filename << "/usr/stud/mueller/test_pcd" << idx << ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename.str().c_str(), *cloud_ptr) == -1) {
        std::cerr << "Couldn't read file " << filename.str().c_str() << std::endl;
        return;
    }
    calculateNormals(cloud_ptr, normals_cloud_ptr);
}

void update()
{
    viewer_->updatePointCloud(point_cloud_ptr_, "cloud");
//    viewer_->removePointCloud("normals");
//    viewer_->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (point_cloud_ptr_, cloud_normals_, 10, 0.05, "normals");
    viewer_->removeShape("cylinder");
    viewer_->addCylinder(*coefficients_[counter_]);
}

int main(int argc, char** argv)
{
    // Init viewer
    viewer_->setBackgroundColor (45.0/255.0,45.0/255.0,45.0/255.0);
    viewer_->addCoordinateSystem (0.1);
    viewer_->initCameraParameters ();

    // Register callbacks
    viewer_->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer_);
    viewer_->registerMouseCallback (mouseEventOccurred, (void*)&viewer_);

    // Load files
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr initial_pc_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr initial_normals_(new pcl::PointCloud<pcl::Normal>);
    loadPointCloud(initial_pc_ptr_, initial_normals_, counter_);
    loadPointCloud(point_cloud_ptr_, cloud_normals_, counter_);
    coefficientsFile_.open("/usr/stud/mueller/cylinderCoefficients.txt");
    std::string line;
    if(coefficientsFile_.is_open()) {
        while(std::getline(coefficientsFile_, line)) {
            std::vector<std::string> strs;
            boost::split(strs, line, boost::is_any_of(" "));

            std::vector<float> c(strs.size());
            for(int i=0; i<strs.size(); i++) {
                c[i] = std::atof(strs[i].c_str());
            }

            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
            coeff->values = c;
            coefficients_.push_back(coeff);
        }
        coefficientsFile_.close();
    } else {
        std::cerr << "Could not open cylinder coefficient file." << std::endl;
        return 0;
    }

    // Add from file
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorH(initial_pc_ptr_);
//    viewer_->addPointCloud<pcl::PointXYZRGB> (initial_pc_ptr_, colorH, "initialCloud");
//    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "initialCloud");
//    viewer_->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (initial_pc_ptr_, initial_normals_, 10, 0.05, "initialNormals");
    viewer_->addCylinder(*coefficients_[0]);

    // Add pointclouds
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr_);
    viewer_->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr_, rgb, "cloud");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
//    viewer_->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (point_cloud_ptr_, cloud_normals_, 10, 0.05, "normals");

    // Main loop
    while(!viewer_->wasStopped()) {
        viewer_->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    // Clean up
    coefficients_.clear();

    return 0;
}
