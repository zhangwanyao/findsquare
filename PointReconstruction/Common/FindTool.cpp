#include "FindTool.h"

void FindTool::FindMinMaxHelper(const std::vector<cv::Point3f>& points, cv::Point3f& min_pt, cv::Point3f& max_pt)
{
	min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::infinity();
	max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::infinity();
	for (int i = 0; i<points.size(); i++)
	{
		min_pt.x = (min_pt.x > points[i].x) ? points[i].x : min_pt.x;
		max_pt.x = (max_pt.x < points[i].x) ? points[i].x : max_pt.x;
		min_pt.y = (min_pt.y > points[i].y) ? points[i].y : min_pt.y;
		max_pt.y = (max_pt.y < points[i].y) ? points[i].y : max_pt.y;
		min_pt.z = (min_pt.z > points[i].z) ? points[i].z : min_pt.z;
		max_pt.z = (max_pt.z < points[i].z) ? points[i].z : max_pt.z;
	}
}
