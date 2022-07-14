#ifndef _COMMON_H_
#define _COMMON_H_
#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

struct PointXYZIT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen的字段对齐
} EIGEN_ALIGN16;

// 将 PointXYZIRT 在pcl中注册，其包括的字段为 x,y,z,intensity,ring,time
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (float, time, time)
)

class parameter
{
public:
    std::vector<float> extLivoxToCam;
    std::vector<float> camIntrinsicMatrix;
    std::vector<float> imageSize;
    std::vector<float> distortionCoeff;

    Eigen::Matrix<float, 4, 4> M_extLivoxToCam;
    Eigen::Matrix<float, 3, 3> M_camIntrinsicMatrix;
public:
    parameter(){
    }
    void load(ros::NodeHandle& nh)
    {
        nh.param<vector<float>>("calibration/extLivoxToCam",  extLivoxToCam, vector<float>());
        nh.param<vector<float>>("calibration/camIntrinsicMatrix",  camIntrinsicMatrix, vector<float>());
        nh.param<vector<float>>("calibration/imageSize",  imageSize, vector<float>());
        nh.param<vector<float>>("calibration/distortionCoeff",  distortionCoeff, vector<float>());

        Eigen::Matrix<float, 4, 4> tmp = Eigen::Map<Eigen::Matrix<float, 4, 4>>(extLivoxToCam.data());
        M_extLivoxToCam = tmp.transpose();
        M_camIntrinsicMatrix = Eigen::Map<Eigen::Matrix<float, 3, 3>>(camIntrinsicMatrix.data());
    }
    ~parameter(){

    }
};

#endif