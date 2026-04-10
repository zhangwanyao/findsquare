#ifndef STD_HEADER_H
#define STD_HEADER_H

#define DEBUG_INFO
#define _USE_MATH_DEFINES

//#include "Kinect.h"

//opencv header
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
//#include <opencv2/flann/flann.hpp>

////pcl header 
//#include <pcl/point_types.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/point_cloud.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/point_representation.h>
//#include <pcl/pcl_base.h>
//#include <pcl/sample_consensus/boost.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/common/transforms.h>
////#include <pcl/io/openni_grabber.h>
////#include <pcl/visualization/cloud_viewer.h>
////eigen header
//#include <Eigen/Core>
//#include <Eigen/Geometry>
////boost header
//#include<boost/shared_ptr.hpp>

//standard header
#include <stdio.h>
#include <stdint.h>
#ifdef _WIN32
#include <tchar.h>
#endif
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#ifdef _WIN32
#include <Windows.h>
#endif
#include <iomanip>
#include <limits>
#include <omp.h>
#include <math.h>
#include <algorithm>
#include <queue>
#include <ctime>
#include <tuple>

using namespace std;
//using namespace pcl;

//self write header
//#include "InOutData.h"
//#include "ParamsSetting.h"
//#include "MathOperation.h"
//#include "MatrixOperation.h"
////#include "CloudSampling.h"
//#include "ErrorLog.h"
//#include "TypeError.h"

//#include "ErrorCheck.h"
//#include "FindKNN.h"

//#define DEVELOPER_MODE

#endif STD_HEADER_H