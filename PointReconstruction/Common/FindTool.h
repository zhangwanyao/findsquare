#pragma once
#include <vector>
#include <opencv2/core/core.hpp> 
namespace FindTool
{
	void FindMinMaxHelper(const std::vector<cv::Point3f>& points, cv::Point3f& min_pt, cv::Point3f& max_pt);
}