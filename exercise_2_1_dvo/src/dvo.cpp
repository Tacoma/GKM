// This source code is intended for use in the teaching course "Vision-Based Navigation" at Technical University Munich only.
// Copyright 2015 Robert Maier, Joerg Stueckler, Technical University Munich

#include <dvo.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <iomanip>


// #ifndef WIN64
//     #define EIGEN_DONT_ALIGN_STATICALLY
// #endif
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sophus/se3.hpp>

#include "ros/ros.h"


#define STR1(x)  #x
#define STR(x)  STR1(x)


#define DEBUG_OUTPUT 1

void convertSE3ToTf(const Eigen::VectorXf &xi, Eigen::Matrix3f &rot, Eigen::Vector3f &t)
{
    // rotation
    Sophus::SE3f se3 = Sophus::SE3f::exp(xi);
    Eigen::Matrix4f mat = se3.matrix();
    rot = mat.topLeftCorner(3, 3);
    t = mat.topRightCorner(3, 1);
}


void convertTfToSE3(const Eigen::Matrix3f &rot, const Eigen::Vector3f &t, Eigen::VectorXf &xi)
{
    Sophus::SE3f se3(rot, t);
    xi = Sophus::SE3f::log(se3);
}


cv::Mat downsampleGray(const cv::Mat &gray)
{
    const float* ptrIn = (const float*)gray.data;
    int w = gray.cols;
    int h = gray.rows;
    int wDown = w/2;
    int hDown = h/2;

    cv::Mat grayDown = cv::Mat::zeros(hDown, wDown, gray.type());
    float* ptrOut = (float*)grayDown.data;
    for (size_t y = 0; y < hDown; ++y)
    {
        for (size_t x = 0; x < wDown; ++x)
        {
            float sum = 0.0f;
            sum += ptrIn[2*y * w + 2*x] * 0.25f;
            sum += ptrIn[2*y * w + 2*x+1] * 0.25f;
            sum += ptrIn[(2*y+1) * w + 2*x] * 0.25f;
            sum += ptrIn[(2*y+1) * w + 2*x+1] * 0.25f;
            ptrOut[y*wDown + x] = sum;
        }
    }

    return grayDown;
}


cv::Mat downsampleDepth(const cv::Mat &depth)
{
    const float* ptrIn = (const float*)depth.data;
    int w = depth.cols;
    int h = depth.rows;
    int wDown = w/2;
    int hDown = h/2;

    // downscaling by averaging the inverse depth
    cv::Mat depthDown = cv::Mat::zeros(hDown, wDown, depth.type());
    float* ptrOut = (float*)depthDown.data;
    for (size_t y = 0; y < hDown; ++y)
    {
        for (size_t x = 0; x < wDown; ++x)
        {
            float d0 = ptrIn[2*y * w + 2*x];
            float d1 = ptrIn[2*y * w + 2*x+1];
            float d2 = ptrIn[(2*y+1) * w + 2*x];
            float d3 = ptrIn[(2*y+1) * w + 2*x+1];

            int cnt = 0;
            float sum = 0.0f;
            if (d0 != 0.0f)
            {
                sum += 1.0f / d0;
                ++cnt;
            }
            if (d1 != 0.0f)
            {
                sum += 1.0f / d1;
                ++cnt;
            }
            if (d2 != 0.0f)
            {
                sum += 1.0f / d2;
                ++cnt;
            }
            if (d3 != 0.0f)
            {
                sum += 1.0f / d3;
                ++cnt;
            }

            if (cnt > 0)
            {
                float dInv = sum / float(cnt);
                if (dInv != 0.0f)
                    ptrOut[y*wDown + x] = 1.0f / dInv;
            }
        }
    }

    return depthDown;
}


bool depthToVertexMap(const Eigen::Matrix3f &K, const cv::Mat &depth, cv::Mat &vertexMap)
{
    int w = depth.cols;
    int h = depth.rows;
    float cx = K(0, 2);
    float cy = K(1, 2);
    float fxInv = 1.0f / K(0, 0);
    float fyInv = 1.0f / K(1, 1);
    
    vertexMap = cv::Mat::zeros(h, w, CV_32FC3);
    float* ptrVert = (float*)vertexMap.data;
    const float* ptrDepth = (const float*)depth.data;
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            float depthVal = ptrDepth[y*w + x];
            float x0 = (float(x) - cx) * fxInv;
            float y0 = (float(y) - cy) * fyInv;
            //depthVal = depthVal * std::sqrt(x0*x0 + y0*y0 + 1.0f);
            
            size_t off = (y*w + x) * 3;
            ptrVert[off] = x0 * depthVal;
            ptrVert[off+1] = y0 * depthVal;
            ptrVert[off+2] = depthVal;
        }
    }
    
    return true;
}


void transformVertexMap(const Eigen::Matrix3f &R, const Eigen::Vector3f &t, cv::Mat &vertexMap)
{
    int w = vertexMap.cols;
    int h = vertexMap.rows;
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            cv::Vec3f pt = vertexMap.at<cv::Vec3f>(y, x);
            if (pt.val[2] == 0.0 || std::isnan(pt.val[2]))
                continue;
            Eigen::Vector3f ptTf(pt.val[0], pt.val[1], pt.val[2]);
            ptTf = R * ptTf + t;
            vertexMap.at<cv::Vec3f>(y, x) = cv::Vec3f(ptTf[0], ptTf[1], ptTf[2]);
        }
    }
}


bool savePlyFile(const std::string &filename, const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector3f> &colors)
{
    if (pts.empty())
        return false;
    
    std::ofstream plyFile;
    plyFile.open(filename.c_str());
    if (!plyFile.is_open())
        return false;
    
    plyFile << "ply" << std::endl;
    plyFile << "format ascii 1.0" << std::endl;
    plyFile << "element vertex " << pts.size() << std::endl;
    plyFile << "property float x" << std::endl;
    plyFile << "property float y" << std::endl;
    plyFile << "property float z" << std::endl;
    plyFile << "property uchar red" << std::endl;
    plyFile << "property uchar green" << std::endl;
    plyFile << "property uchar blue" << std::endl;
    plyFile << "element face 0" << std::endl;
    plyFile << "property list uchar int vertex_indices" << std::endl;
    plyFile << "end_header" << std::endl;
    
    for (size_t i = 0; i < pts.size(); i++)
    {
        plyFile << pts[i][0] << " " << pts[i][1] << " " << pts[i][2];
        plyFile << " " << (int)colors[i][0] << " " << (int)colors[i][1] << " " << (int)colors[i][2];
        plyFile << std::endl;
    }
    plyFile.close();
    
    return true;
}


bool savePlyFile(const std::string &filename, const cv::Mat &color, const cv::Mat &vertexMap)
    {
    // convert frame to points vector and colors vector
    std::vector<Eigen::Vector3f> pts;
    std::vector<Eigen::Vector3f> colors;
    for (int y = 0; y < vertexMap.rows; ++y)
    {
        for (int x = 0; x < vertexMap.cols; ++x)
        {
            cv::Vec3f pt = vertexMap.at<cv::Vec3f>(y, x);
            if (pt.val[2] == 0.0 || std::isnan(pt.val[2]))
                continue;
            pts.push_back(Eigen::Vector3f(pt.val[0], pt.val[1], pt.val[2]));
            
            cv::Vec3b c = color.at<cv::Vec3b>(y, x);
            colors.push_back(Eigen::Vector3f(c.val[2], c.val[1], c.val[0]));
        }
    }
    
    return savePlyFile(filename, pts, colors);
}


bool saveTrajectory(const std::string &filename, const tf::StampedTransform transform, const ros::Time timestamp) {
    
    std::ofstream file;
    file.open(filename.c_str(), std::ios::out | std::ios::app | std::ios::binary);
    if (!file.is_open()) {
	ROS_ERROR_STREAM("Error opening " << filename << ".");
	return false;
    }

    tf::Quaternion R = transform.getRotation();
    tf::Vector3 t = transform.getOrigin();
    file << timestamp << " ";
    file << t.x() << " " << t.y() << " " << t.z() << " " << R.x() << " " << R.y() << " " << R.z() << " " << R.w();
    file << std::endl;

    file.close();    
    
    return true;
}


void computeGradient(const cv::Mat &gray, cv::Mat &gradient, int direction)
{
    int dirX = 1;
    int dirY = 0;
    if (direction == 1)
    {
        dirX = 0;
        dirY = 1;
    }

    // compute gradient manually using finite differences
    int w = gray.cols;
    int h = gray.rows;
    const float* ptrIn = (const float*)gray.data;
    gradient = cv::Mat::zeros(h, w, CV_32FC1);
    float* ptrOut = (float*)gradient.data;

    int yStart = dirY;
    int yEnd = h - dirY;
    int xStart = dirX;
    int xEnd = w - dirX;
    for (size_t y = yStart; y < yEnd; ++y)
    {
        for (size_t x = xStart; x < xEnd; ++x)
        {
            float v0;
            float v1;
            if (direction == 1)
            {
                // y-direction
                v0 = ptrIn[(y-1)*w + x];
                v1 = ptrIn[(y+1)*w + x];
            }
            else
            {
                // x-direction
                v0 = ptrIn[y*w + (x-1)];
                v1 = ptrIn[y*w + (x+1)];
            }
            ptrOut[y*w + x] = 0.5f * (v1 - v0);
        }
    }
}


float interpolate(const float* ptrImgIntensity, float x, float y, int w, int h)
{
    float valCur = std::numeric_limits<float>::quiet_NaN();

#if 0
    // direct lookup, no interpolation
    int x0 = static_cast<int>(std::floor(x + 0.5));
    int y0 = static_cast<int>(std::floor(y + 0.5));
    if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h)
        valCur = ptrImgIntensity[y0*w + x0];
#else
    //bilinear interpolation
    int x0 = static_cast<int>(std::floor(x));
    int y0 = static_cast<int>(std::floor(y));
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    float x1_weight = x - static_cast<float>(x0);
    float y1_weight = y - static_cast<float>(y0);
    float x0_weight = 1.0 - x1_weight;
    float y0_weight = 1.0 - y1_weight;

    if (x0 < 0 || x0 >= w)
        x0_weight = 0.0;
    if (x1 < 0 || x1 >= w)
        x1_weight = 0.0;
    if (y0 < 0 || y0 >= h)
        y0_weight = 0.0;
    if (y1 < 0 || y1 >= h)
        y1_weight = 0.0;
    float w00 = x0_weight * y0_weight;
    float w10 = x1_weight * y0_weight;
    float w01 = x0_weight * y1_weight;
    float w11 = x1_weight * y1_weight;

    float sumWeights = w00 + w10 + w01 + w11;
    float sum = 0.0;
    if (w00 > 0.0)
        sum += ptrImgIntensity[y0*w + x0] * w00;
    if (w01 > 0.0)
        sum += ptrImgIntensity[y1*w + x0] * w01;
    if (w10 > 0.0)
        sum += ptrImgIntensity[y0*w + x1] * w10;
    if (w11 > 0.0)
        sum += ptrImgIntensity[y1*w + x1] * w11;

    if (sumWeights > 0.0)
        valCur = sum / sumWeights;
#endif

    return valCur;
}


float calculateError(const Eigen::VectorXf &residuals)
{
    float error = 0.0;
    int n = residuals.size();
    int numValid = 0;
    for (int i = 0; i < n; ++i)
    {
        if (residuals[i] != 0.0)
        {
            error += residuals[i] * residuals[i];
            ++numValid;
        }
    }
    if (numValid > 0)
        error = error / static_cast<float>(numValid);
    return error;
}


void calculateErrorImage(const Eigen::VectorXf &residuals, int w, int h, cv::Mat &errorImage)
{
    cv::Mat imgResiduals = cv::Mat::zeros(h, w, CV_32FC1);
    float* ptrResiduals = (float*)imgResiduals.data;

    // fill residuals image
    for (size_t y = 0; y < h; ++y)
    {
        for (size_t x = 0; x < w; ++x)
        {
            size_t off = y*w + x;
            if (residuals[off] != 0.0)
                ptrResiduals[off] = residuals[off];
        }
    }

    imgResiduals.convertTo(errorImage, CV_8SC1, 127.0);
}


Eigen::VectorXf calculateError( const cv::Mat &grayRef, const cv::Mat &depthRef,
                                const cv::Mat &grayCur, const cv::Mat &depthCur,
                                const Eigen::VectorXf &xi, const Eigen::Matrix3f &K)
{
    Eigen::VectorXf residualsVec;

    // create residual image
    int w = grayRef.cols;
    int h = grayRef.rows;

    // camera intrinsics
    float fx = K(0, 0);
    float fy = K(1, 1);
    float cx = K(0, 2);
    float cy = K(1, 2);
    float fxInv = 1.0 / fx;
    float fyInv = 1.0 / fy;

    // convert SE3 to rotation matrix and translation vector
    Eigen::Matrix3f rotMat;
    Eigen::Vector3f t;
    convertSE3ToTf(xi, rotMat, t);

    const float* ptrGrayRef = (const float*)grayRef.data;
    const float* ptrDepthRef = (const float*)depthRef.data;
    const float* ptrGrayCur = (const float*)grayCur.data;
    const float* ptrDepthCur = (const float*)depthCur.data;

    residualsVec.resize(w*h);
    for (size_t y = 0; y < h; ++y)
    {
        for (size_t x = 0; x < w; ++x)
        {
            size_t off = y*w + x;
            float residual = 0.0;

            // project 2d point back into 3d using its depth
            float dRef = ptrDepthRef[y*w + x];
            if (dRef > 0.0)
            {
                float x0 = (static_cast<float>(x) - cx) * fxInv;
                float y0 = (static_cast<float>(y) - cy) * fyInv;
                //dRef = dRef * std::sqrt(x0*x0 + y0*y0 + 1.0);
                x0 = x0 * dRef;
                y0 = y0 * dRef;

                // transform reference 3d point into current frame
                // reference 3d point
                Eigen::Vector3f pt3Ref(x0, y0, dRef);
                Eigen::Vector3f pt3Cur = rotMat * pt3Ref + t;
                if (pt3Cur[2] > 0.0)
                {
                    // project 3d point to 2d
                    Eigen::Vector3f pt2CurH = K * pt3Cur;
                    float px = pt2CurH[0] / pt2CurH[2];
                    float py = pt2CurH[1] / pt2CurH[2];

                    // interpolate residual
                    float valCur = interpolate(ptrGrayCur, px, py, w, h);
                    if (!std::isnan(valCur))
                    {
                        float valRef = ptrGrayRef[off];
                        float valDiff = valRef - valCur;
                        residual = valDiff;
                    }
                }
            }
            residualsVec[off] = residual;
        }
    }

    return residualsVec;
}


void calculateMeanStdDev(const Eigen::VectorXf &residuals, float &mean, float &stdDev)
{
    mean = residuals.mean();
    
#if 1
    float variance = 0.0;
    for (int i = 0; i < residuals.size(); ++i)
        variance += (residuals[i] - mean) * (residuals[i] - mean);
    stdDev = std::sqrt(variance);
#else
    stdDev = 4.0;
#endif
}


void calculateCovariance(const Eigen::Matrix<float, -1, 6>& J, Eigen::Matrix<float,6,6>& covariance)
{
    Eigen::Matrix<float, 6, -1> Jt = J.transpose();
    covariance = (Jt*J).inverse();
}


void weighting(Eigen::VectorXf &residuals, Eigen::VectorXf &weights)
{
    int n = residuals.size();
    
#if 0
    // no weighting
    weights = Eigen::VectorXf::Ones(n);
#if 0
    // squared residuals
    for (int i = 0; i < n; ++i)
        residuals[i] = residuals[i] * residuals[i];
    return;
#endif
#endif
    
    // compute mean and standard deviation
    float mean, stdDev;
    calculateMeanStdDev(residuals, mean, stdDev);

    // compute robust Huber weights
    float k = 1.345 * stdDev;
    weights = Eigen::VectorXf(n);
    for (int i = 0; i < n; ++i)
    {
        float w;
        if (std::abs(residuals[i]) <= k)
            w = 1.0;
        else
            w = k / std::abs(residuals[i]);
        weights[i] = w;
    }
    //std::cout << W.block(0, 0, 10, 10) << std::endl;

#if 0
    // adjust residuals
    for (int i = 0; i < n; ++i)
    {
        if (std::abs(residuals[i]) <= k)
            residuals[i] = 0.5 * residuals[i] * residuals[i];
        else
            residuals[i] = k * std::abs(residuals[i]) - 0.5*k*k;
    }
#endif
}


// TODO: test
void deriveNumeric( const cv::Mat &grayRef, const cv::Mat &depthRef,
                    const cv::Mat &grayCur, const cv::Mat &depthCur,
                    const Eigen::VectorXf &xi, const Eigen::Matrix3f &K,
                    Eigen::VectorXf &residuals, Eigen::MatrixXf &J)
{
    // residuals are 1 x n
    // J is n x 6
    const float eps = 0.000001f;
    Eigen::VectorXf epsVec(6,1); // may need to set data zero
    Eigen::VectorXf xiPerm;
    Eigen::VectorXf residualsTmp;

    residuals = calculateError(grayRef, depthRef, grayCur, depthCur, xi, K);
    for(unsigned int j=0; j<6; j++)
    {
        epsVec << 0,0,0,0,0,0;
        epsVec(j) = eps;

        xiPerm = Sophus::SE3f::log( Sophus::SE3f::exp(epsVec) * Sophus::SE3f::exp(xi) );

        residualsTmp = calculateError(grayRef, depthRef, grayCur, depthCur, xiPerm, K);
        for(unsigned int i=0; i<residuals.size(); i++)
        {
            J(i,j) = (residualsTmp(i)-residuals(i)) / eps;
        }
    }
}


// TODO: compile and test
void deriveAnalytic(const cv::Mat &grayRef, const cv::Mat &depthRef,
                   const cv::Mat &grayCur, const cv::Mat &depthCur,
                   const cv::Mat &gradX, const cv::Mat &gradY,
                   const Eigen::VectorXf &xi, const Eigen::Matrix3f &K,
                   Eigen::VectorXf &residuals, Eigen::MatrixXf &J)
{

    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    convertSE3ToTf(xi, R, t);

    Eigen::MatrixXf RKInv = R * K.inverse();

    Eigen::Vector3f tempVec(3,1);
    Eigen::Vector3f p(3,1);
    Eigen::Vector3f pTrans(3,1);
    Eigen::Vector3f pTransProj(3,1);

#if DEBUG_OUTPUT
    ROS_ERROR_STREAM( "t: " << t(0) << ","<< t(1) << "," << t(2) << "\n");
    ROS_ERROR_STREAM( "R: " << R(0,0) << ","<< R(0,1) << "," << R(0,2) << "\n");
#endif

    double xp = 0.0, yp = 0.0, zp = 0.0;
    double dxInt = 0.0, dyInt = 0.0;

    int width = grayRef.cols;
    int height = grayRef.rows;
    
    for (int x = 0; x < width; x++) {
	for (int y = 0; y < height; y++) {
	    if (std::isnan(gradX.at<double>(y,x))) {
		ROS_ERROR_STREAM("gradX is NAN");
	    }
	    if (std::isnan(gradY.at<double>(y,x))) {
		ROS_ERROR_STREAM("gradY is NAN");
	    }
	}
    }


    double dRef = 0.0;
    double J1 =0.0, J2=0.0, J3=0.0, J4=0.0, J5=0.0, J6=0.0;


    J = Eigen::MatrixXf::Zero(width * height, 6);
    residuals = calculateError(grayRef, depthRef, grayCur, depthCur, xi, K);

    for (int x = 0; x < width ; x++)
    {
        for (int y=0; y < height ; y++)
        {

            dRef = depthRef.at<double>(y,x);

            if(!std::isnan(dRef) && dRef > 0.0 && dRef< 100.0)
            {
                p << x * dRef , y * dRef , dRef ;

//            #if DEBUG_OUTPUT
//                ROS_ERROR_STREAM( "P: " << p(0) << "\n");
//                ROS_ERROR_STREAM( "P: " << p(1) << "\n");
//                ROS_ERROR_STREAM( "P: " << p(2) << "\n");
//                ROS_ERROR_STREAM( "depth: " << dRef << "\n");
//            #endif




            // if point is valid (depth > 0), project and save result.

                // transform to image (unproject, rotate & translate)
                pTrans = RKInv * p + t;
                // warped 3d point, for calculation of Jacobian.

                xp = pTrans(0);
                yp = pTrans(1);
                zp = pTrans(2);


//                #if DEBUG_OUTPUT
//                    ROS_ERROR_STREAM( "xP: " << xp << "\n");
//                    ROS_ERROR_STREAM( "yP: " << yp << "\n");
//                    ROS_ERROR_STREAM( "zP: " << zp << "\n");
//                    ROS_ERROR_STREAM( "depth: " << dRef << "\n");
//                #endif


                if (!std::isnan(zp) && zp > 0.0 && zp < 100.0)
                {

                    dxInt = K(0,0) * gradX.at<double>(y,x);
                    dyInt = K(1,1) * gradY.at<double>(y,x);

                    J1 = dxInt / zp;
                    J2 = dyInt / zp;
                    J3 = -(dxInt*xp + dyInt*yp) / (zp*zp);
                    J4 = - (dxInt*xp*yp)/(zp*zp) - dyInt*(1+(yp/zp)*(yp/zp));
                    J5 = + (dyInt*xp*yp)/(zp*zp) + dxInt*(1+(xp/zp)*(xp/zp));
                    J6 = (- dxInt* yp + dyInt*xp)/ zp;

//#if DEBUG_OUTPUT
//    ROS_ERROR_STREAM("zp:" << zp << "J1: " << J1 << "J2"<< J2 << "J3" << J3 << "J4: " << J4 << "J5"<< J5 << "J6" << J6 << "\n");
//#endif
                    if (!std::isnan(J1) && !std::isnan(J2) && !std::isnan(J3) && !std::isnan(J4) && !std::isnan(J5) && !std::isnan(J6))
                    {
                        J(y*width + x , 0) = J1;
                        J(y*width + x , 1) = J2;
                        J(y*width + x , 2) = J3;
                        J(y*width + x , 3) = J4;
                        J(y*width + x , 4) = J5;
                        J(y*width + x , 5) = J6;
                    }
                }

            }
        }
    }

    J = -J;

//    deriveNumeric(grayRef, depthRef, grayCur, depthCur, xi, K, residuals, J);

}

// TODO: test
// expects float images (CV_32FC1), grayscale scaled to [0,1], metrical depth
// @parameter transform: transform from _ to _
// @parameter imgGrayRef, imgDepthRef: reference rgbd image
// @parameter imgGrayCur, imgDepthCur: current rgbd image
// @parameter cameraMatrix: camera intrinsics
void alignImages( Eigen::Matrix4f& transform, const cv::Mat& imgGrayRef, const cv::Mat& imgDepthRef,
                  const cv::Mat& imgGrayCur, const cv::Mat& imgDepthCur, const Eigen::Matrix3f& cameraMatrix )
{
  
    cv::Mat grayRef = imgGrayRef;
    cv::Mat grayCur = imgGrayCur;
    cv::Mat depthRef = imgDepthRef;
    cv::Mat depthCur = imgDepthCur;

    // downsampling
    int numPyramidLevels = 5;
    std::vector<Eigen::Matrix3f> kPyramid;
    kPyramid.push_back(cameraMatrix);
    std::vector<cv::Mat> grayRefPyramid;
    grayRefPyramid.push_back(grayRef);
    std::vector<cv::Mat> depthRefPyramid;
    depthRefPyramid.push_back(depthRef);
    std::vector<cv::Mat> grayCurPyramid;
    grayCurPyramid.push_back(grayCur);
    std::vector<cv::Mat> depthCurPyramid;
    depthCurPyramid.push_back(depthCur);
    for (int i = 1; i < numPyramidLevels; ++i)
    {
        // downsample camera matrix
        Eigen::Matrix3f kDown = kPyramid[i-1];
        kDown(0, 2) += 0.5; kDown(1, 2) += 0.5;
        kDown.topLeftCorner(2, 3) = kDown.topLeftCorner(2, 3) * 0.5;
        kDown(0, 2) -= 0.5; kDown(1, 2) -= 0.5;
        kPyramid.push_back(kDown);
        //std::cout << "Camera matrix (level " << i << "): " << kDown << std::endl;

        // downsample grayscale images
        cv::Mat grayRefDown = downsampleGray(grayRefPyramid[i-1]);
        grayRefPyramid.push_back(grayRefDown);
        cv::Mat grayCurDown = downsampleGray(grayCurPyramid[i-1]);
        grayCurPyramid.push_back(grayCurDown);

        // downsample depth images
        cv::Mat depthRefDown = downsampleDepth(depthRefPyramid[i-1]);
        depthRefPyramid.push_back(depthRefDown);
        cv::Mat depthCurDown = downsampleDepth(depthCurPyramid[i-1]);
        depthCurPyramid.push_back(depthCurDown);
    }

    Eigen::Matrix3f rot;
    Eigen::Vector3f t;
    typedef Eigen::Matrix<float, 6, 6> Mat6f;
    typedef Eigen::Matrix<float, 6, 1> Vec6f;
    Eigen::VectorXf xi = Eigen::VectorXf::Zero( 6 );
    Eigen::VectorXf lastXi = Eigen::VectorXf::Zero( 6 );
    
    //convertSE3ToTf(xi, rot, t);
    rot = transform.block<3,3>(0,0);
    t = transform.block<3,1>(0,3);
    convertTfToSE3( rot, t, xi );
    
    ROS_INFO_STREAM("Initial pose: ");
    ROS_INFO_STREAM("t = " << t.transpose() << std::endl);
    ROS_INFO_STREAM("R = " << rot << std::endl);
    

    bool useNumericDerivative = false;
    
    bool useGN = true;
    bool useGD = false;
    bool useLM = false;
    bool useWeights = true;
    int numIterations = 20;
    int maxLevel = numPyramidLevels-1;
    int minLevel = 1;
    
    Mat6f A;                      // 6 x 6
    Mat6f diagMatA = Mat6f::Identity();
    Vec6f delta;
    
    float tmr = (float)cv::getTickCount();
    // Iterate pyramids
    for (int level = maxLevel; level >= minLevel; --level)
    {
        float lambda = 0.1;
        
        grayRef = grayRefPyramid[level];
        depthRef = depthRefPyramid[level];
        grayCur = grayCurPyramid[level];
        depthCur = depthCurPyramid[level];
        Eigen::Matrix3f kLevel = kPyramid[level];
        //std::cout << "level " << level << " (size " << depthRef.cols << "x" << depthRef.rows << ")" << std::endl;

        // compute gradient images
        cv::Mat gradX;
        computeGradient(grayCur, gradX, 0);
        cv::Mat gradY;
        computeGradient(grayCur, gradY, 1);

        float errorLast = std::numeric_limits<float>::max();
        // Iterative computation of pose (rot, t)
        for (int itr = 0; itr < numIterations; ++itr)
        {
            // compute residuals and Jacobian
            Eigen::VectorXf residuals;                          // 1 x n
            Eigen::MatrixXf J(grayRef.rows * grayRef.cols, 6);  // n x 6 // maybe columns and rows have to be switched

            if( useNumericDerivative )
              deriveNumeric(grayRef, depthRef, grayCur, depthCur, xi, kLevel, residuals, J);
            else
              deriveAnalytic(grayRef, depthRef, grayCur, depthCur, gradX, gradY, xi, kLevel, residuals, J);

#if DEBUG_OUTPUT
            // compute and show error image
            cv::Mat errorImage;
            calculateErrorImage(residuals, grayRef.cols, grayRef.rows, errorImage);
            //cv::imshow("error", errorImage);
            //cv::waitKey(100);
#endif

            // calculate error
            float error = calculateError(residuals);

            Eigen::MatrixXf Jt = J.transpose();     // 6 x n
            Eigen::VectorXf weights;                // n x 1
            if (useWeights)
            {
                // compute robust weights
                weighting(residuals, weights);
                if (weights.size() != residuals.size())
                {
                    std::cout << "weight vector has wrong size!" << std::endl;
                    continue;
                }
                residuals = residuals.cwiseProduct(weights);
                
                // compute weighted Jacobian
                for (int i = 0; i < residuals.size(); ++i)
                    for (int j = 0; j < J.cols(); ++j)
                        J(i, j) = J(i, j) * weights[i];
            }
            
            // compute update
            Eigen::VectorXf b = Jt * residuals;
            if (useGD)
            {
                // BEN: Implement Gradient Descent (step size 0.001)
                delta = 0.001 * -b;
            }
            
            if (useGN)
            {
                // Gauss-Newton algorithm
                A = Jt * J;
                // solve using Cholesky LDLT decomposition
                delta = -(A.ldlt().solve(b));
            }
            
            if (useLM)
            {
                // BEN: Implement Levenberg-Marquardt algorithm
                A = Jt * J;
                Mat6f diag = A.diagonal().asDiagonal();
#if DEBUG_OUTPUT
                ROS_ERROR_STREAM( "If " << diag << " is is the diagonal of " << A << " delete this output");
#endif
                A = A + lambda * diag;
                delta = -(A.ldlt().solve(b));
            }

            // apply update
            // right-multiplicative increment on SE3
            lastXi = xi;
            xi = Sophus::SE3f::log( Sophus::SE3f::exp(xi) * Sophus::SE3f::exp(delta) );
#if DEBUG_OUTPUT
            ROS_INFO_STREAM( "delta = " << delta.transpose() << " size = " << delta.rows() << " x " << delta.cols());
            ROS_INFO_STREAM( "xi = " << xi.transpose());
#endif
            
            // compute error again
            error = (residuals.cwiseProduct(residuals)).mean();

            if (useLM)
            {
                if (error >= errorLast)
                {
                    lambda = lambda * 5.0;
                    xi = lastXi;

                    if (lambda > 5.0)
                        break;
                }
                else
                {
                    lambda = lambda / 1.5;
                }
            }
            
            if (useGN || useGD)
            {
                // break if no improvement (0.99 or 0.995)
                if (error / errorLast > 0.995)
                    break;
            }

            errorLast = error;
        }// iteration
    }// level
    tmr = ((float)cv::getTickCount() - tmr)/cv::getTickFrequency();
    ROS_INFO_STREAM( "runtime: " << tmr );

    convertSE3ToTf(xi, rot, t);
    
    transform.block<3,3>(0,0) = rot;
    transform.block<3,1>(0,3) = t;


    
#if DEBUG_OUTPUT
    //cv::waitKey(0);
#endif

}


