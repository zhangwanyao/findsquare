#pragma once
#include "iostream"
#include <vector>
#include <algorithm>
#include <random>
#include <cmath>
#include <opencv2/core.hpp>
#include "../OptimizeByRGBD/THandleImgAndEdge.h"

template<class Dtype = double>
inline cv::Point3_<Dtype>unrePt2rightCoord1(const cv::Point3_<Dtype>& unrePt)
{
	return { unrePt.x,unrePt.z,-unrePt.y };
}

template<class Dtype = double>
inline cv::Point3_<Dtype>rightCoord2unrePt1(const cv::Point3_<Dtype>& unrePt)
{
	return { unrePt.x,-unrePt.z,unrePt.y };
}

typedef struct scenePointInfo
{
	float x;
	float y;
	float z;
	unsigned char i;
}scenePointInfo;

bool ReadEncryFile(vector<scenePointInfo>* vecPtInfos, string strPath);

class BallDetection
{
public:
	using data_type = tuple<Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Vector3d, cv::Mat,
		shared_ptr<THandleImgAndEdge>>;

	auto Read(std::string &sense_path);

	auto alignment(std::string &sense_path);

	std::string WstringToString(const std::wstring wstr);

	static void ReadEncryptionData(std::string path, std::vector<cv::Point3f>& data, std::vector<int> &reflectance);

	std::vector<std::vector<cv::Point3d>> ROI_map2points(std::string &sense_path);

	void writeInfo_sphere_argument(std::vector<std::vector<cv::Point3d>> sphere_argument_vec_all, std::string address);

private:
	auto xyz2local(Eigen::Vector3d &xyz, data_type &camera_param)
	{
		auto xyz_local = get<1>(camera_param) * xyz + get<2>(camera_param);
		return xyz_local;
	}

	auto local2pixel(Eigen::Vector3d &xyz, data_type &camera_param)
	{
		auto xyz_local = get<0>(camera_param) * xyz;
		return xyz_local;
	}

	auto cv3f2eigen3f(const cv::Point3f &point3f);

	auto eigen3d2cv3d(const Eigen::Vector3d &vector);

	int cagaus(double a[], double b[], int n, double x[]);

	inline void estimate_sphere(const cv::Point3d& p1, const cv::Point3d& p2, const cv::Point3d& p3, const cv::Point3d& p4, cv::Point3d& center, double & radius2);

	inline void GetNRand(const int maxV, const int N, std::set<int>& idxs);

	auto sphere_leastFit(const std::vector<cv::Point3d> &points, double &center_x, double &center_y, double &center_z, double &radius);

	double FitCircleByRANSAC(const std::vector<cv::Point3d>& pointArray, cv::Point3d& center, double& radius, const int iterNum, const double e, const float ratio);

	auto points_filter(std::vector<cv::Point3d> &points_ROI, std::vector<int> &reflect_ROI);


};