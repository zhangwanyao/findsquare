#pragma once
#include "WallXZRefectImg.h"
#include "OneMeterCheck.h"

class COneMeterHelper
{
public:
	COneMeterHelper();
	~COneMeterHelper();

	bool CheckAllWallOneMeterAve(
		const std::vector<int>& plane_wall_idx,
		const std::vector<cv::Point3f> plane_normals,
		const std::vector<cv::Point3f>& plane_centers,
		const std::vector<std::vector<cv::Point3f>>& plane_xyz,
		const std::vector<std::vector<unsigned char>>& plane_reflect,
		float& mean_one_meter_z);

private:

	bool CheckAllWallOneMeterAveHelper(
		const std::vector<int>& plane_wall_idx,
		const std::vector<cv::Point3f> plane_normals,
		const std::vector<cv::Point3f>& plane_centers,
		const std::vector<std::vector<cv::Point3f>>& plane_xyz,
		const std::vector<std::vector<unsigned char>>& plane_reflect,
		float& mean_one_meter_z);

	bool CheckWallIdOneMeter(const int& wall_id,
		const cv::Point3f& plane_normal,
		const cv::Point3f& plane_center,
		const std::vector<cv::Point3f>& ori_plane,
		const std::vector<int>& planeReflectValues,
		float& one_meter_z);



private:
	CWallXZRefectImg *mWallXZRefectImg;
	COneMeter *mOneMeter;
};

