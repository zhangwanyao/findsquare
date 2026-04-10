#pragma once

#include <vector>
#include <opencv2/core/core.hpp> 
#include "PlaneXyRefectImg.h"

class CAxisLine
{
public:
	CAxisLine();
	~CAxisLine();

	bool CheckGroundAixsLine(
		const std::vector<int>& plane_ground_idx,
		const std::vector<cv::Point3f> plane_normals,
		const std::vector<cv::Point3f>& plane_centers,
		const std::vector<std::vector<cv::Point3f>>& plane_xyz,
		const std::vector<std::vector<unsigned char>>& plane_reflect,
		std::vector<std::pair<cv::Point3f, cv::Point3f>>& m_ground_axis_line);

private:
	bool CheckGroundAixsLineHelper(
		const std::vector<int>& plane_ground_idx,
		const std::vector<cv::Point3f> plane_normals,
		const std::vector<cv::Point3f>& plane_centers,
		const std::vector<std::vector<cv::Point3f>>& plane_xyz,
		const std::vector<std::vector<unsigned char>>& plane_reflect,
		std::vector<std::pair<cv::Point3f, cv::Point3f>>& m_ground_axis_line);


private:
	CPlaneXyImg *mXYRefectImg;
};
