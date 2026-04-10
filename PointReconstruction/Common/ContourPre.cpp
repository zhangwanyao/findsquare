#include "ContourPre.h"
#include "mesh_tool.h"
#include "LookatOperation.h"
#include "FindTool.h"

void ContourPre::FilterOutMinMax(
	const cv::Point3f min_pt,
	const cv::Point3f max_pt,
	const std::vector<std::pair<float, cv::Point3f>>& defect_pair_lookat,
	std::vector<std::pair<float, cv::Point3f>>& defect_pair_lookat_filter)
{
	defect_pair_lookat_filter.resize(0);
	float min_x = min_pt.x;
	float min_y = min_pt.y;
	
	float max_x = max_pt.x;
	float max_y = max_pt.y;
	size_t point_num = defect_pair_lookat.size();
	
	for (int i=0;i<point_num;i++)
	{
		cv::Point3f pt = defect_pair_lookat[i].second;
		if (pt.x>=min_x && pt.x<=max_x && pt.y >= min_y && pt.y <= max_y)
		{
			defect_pair_lookat_filter.push_back(defect_pair_lookat[i]);
		}
	}

}

bool ContourPre::FilterDefectPairHelper(
	const std::string& mesh_dir,
	const int& wall_id,
	const cv::Point3f& inner_normal,
	const cv::Point3f& plane_center,
	const std::vector<std::pair<float, cv::Point3f>> defect_pair,
	std::vector<std::pair<float, cv::Point3f>>& defect_pair_filter)
{
	std::vector<cv::Point3f> obj_points;
	std::vector<cv::Vec3i> face;
	//std::string obj_path = "origin_meshes\\mesh" + std::to_string(wall_id) + ".obj";
	std::string obj_path = mesh_dir + "mesh" + std::to_string(wall_id) + ".obj";

	CMeshTool meshTool;
	//¶ÁÈ¡obj
	bool isGet = meshTool.GetObjData(obj_path, obj_points, face);
	if (!isGet)
	{
		std::cout << "FilterDefectPairHelper GetObjData false!" << std::endl;
		return false;
	}

	cv::Mat lookAt;
	CLookAt::GetLookAtMat(inner_normal, plane_center, 100, lookAt);

	std::vector<cv::Point3f> obj_points_lookat;
	CLookAt::RotationPoints(obj_points, obj_points_lookat, lookAt);

	std::vector<std::pair<float, cv::Point3f>> defect_pair_lookat;
	CLookAt::RotationPointsPair(defect_pair, defect_pair_lookat, lookAt);

	cv::Point3f min_pt;
	cv::Point3f max_pt;
	FindTool::FindMinMaxHelper(obj_points_lookat, min_pt, max_pt);

	std::vector<std::pair<float, cv::Point3f>> defect_pair_lookat_filter;
	FilterOutMinMax(min_pt, max_pt, defect_pair_lookat, defect_pair_lookat_filter);

	cv::Mat lookAt_inv;
	cv::invert(lookAt, lookAt_inv, CV_SVD);

	std::vector<std::pair<float, cv::Point3f>> defect_pair_lookat_filter_ori;
	CLookAt::RotationPointsPair(defect_pair_lookat_filter, defect_pair_lookat_filter_ori, lookAt_inv);

	defect_pair_filter = defect_pair_lookat_filter_ori;
	return true;
}

void ContourPre::FilterVecPointOutMinMax(
	const cv::Point3f min_pt,
	const cv::Point3f max_pt,
	const std::vector<cv::Point3f>& plane_point_lookat,
	const std::vector<unsigned char>& reflects,
	std::vector<cv::Point3f>& plane_point_lookat_filter,
	std::vector<unsigned char>& reflects_filter)
{
	plane_point_lookat_filter.resize(0);
	reflects_filter.resize(0);
	float min_x = min_pt.x;
	float min_y = min_pt.y;

	float max_x = max_pt.x;
	float max_y = max_pt.y;
	size_t point_num = plane_point_lookat.size();

	for (int i = 0; i<point_num; i++)
	{
		cv::Point3f pt = plane_point_lookat[i];
		if (pt.x >= min_x && pt.x <= max_x && pt.y >= min_y && pt.y <= max_y)
		{
			plane_point_lookat_filter.push_back(pt);
			reflects_filter.push_back(reflects[i]);
		}
	}
}

 bool GetVecPointCenter(const std::vector<cv::Point3f>& plane_point, cv::Point3f center_point)
{
	 size_t num = plane_point.size();
	 if (num==0)
	 {
		 return false;
	 }

	 cv::Point3f pt_ave(0,0,0);
	 for (int i=0;i<num;i++)
	 {
		 pt_ave += plane_point[i];
	 }
	 pt_ave / float(num);
	 center_point = pt_ave;
	 return true;
}

bool ContourPre::FilterPlanePointHelper(
	const std::string& mesh_dir,
	const int& wall_id,
	const cv::Point3f& inner_normal,
	const cv::Point3f& plane_center,
	const std::vector<cv::Point3f>& plane_point,
	const std::vector<unsigned char>& reflects,
	std::vector<cv::Point3f>& plane_point_filter,
	std::vector<unsigned char>& reflects_filter,
	cv::Point3f& plane_center_filter)
{
	std::vector<cv::Point3f> obj_points;
	std::vector<cv::Vec3i> face;
	//std::string obj_path = "origin_meshes\\mesh" + std::to_string(wall_id) + ".obj";
	std::string obj_path = mesh_dir + "mesh" + std::to_string(wall_id) + ".obj";

	CMeshTool meshTool;
	//¶ÁÈ¡obj
	bool isGet = meshTool.GetObjData(obj_path, obj_points, face);
	if (!isGet)
	{
		std::cout << "FilterPlanePointHelper GetObjData false!" << std::endl;
		return false;
	}

	bool isCenter = GetVecPointCenter(obj_points, plane_center_filter);
	if (!isCenter)
	{
		std::cout << "FilterPlanePointHelper isCenter false!" << std::endl;
		return false;
	}

	cv::Mat lookAt;
	CLookAt::GetLookAtMat(inner_normal, plane_center, 100, lookAt);

	std::vector<cv::Point3f> obj_points_lookat;
	CLookAt::RotationPoints(obj_points, obj_points_lookat, lookAt);

	cv::Point3f min_pt;
	cv::Point3f max_pt;
	FindTool::FindMinMaxHelper(obj_points_lookat, min_pt, max_pt);

	std::vector<cv::Point3f> plane_point_lookat;
	CLookAt::RotationPoints(plane_point, plane_point_lookat, lookAt);

	std::vector<cv::Point3f> plane_point_lookat_filter;
	std::vector<unsigned char> reflects_filte_tmp;
	FilterVecPointOutMinMax(min_pt,max_pt,plane_point_lookat,reflects,plane_point_lookat_filter, reflects_filte_tmp);


	cv::Mat lookAt_inv;
	cv::invert(lookAt, lookAt_inv, CV_SVD);

	std::vector<cv::Point3f> plane_point_lookat_filter_ori;
	CLookAt::RotationPoints(plane_point_lookat_filter, plane_point_lookat_filter_ori, lookAt_inv);

	plane_point_filter = plane_point_lookat_filter_ori;
	reflects_filter = reflects_filte_tmp;


	
	return true;
}

bool ContourPre::FilterPlanePointInfo(
	const std::string& mesh_dir,
	const int& wall_id,
	const cv::Point3f& inner_normal,
	const cv::Point3f& plane_center,
	const std::vector<cv::Point3f>& plane_point,
	const std::vector<unsigned char>& reflects,
	const std::vector<std::pair<float, cv::Point3f>> defect_pair,
	std::vector<cv::Point3f>& plane_point_filter,
	std::vector<unsigned char>& reflects_filter,
	std::vector<std::pair<float, cv::Point3f>>& defect_pair_filter,
	cv::Point3f& obj_center)

{
	std::vector<cv::Point3f> obj_points;
	std::vector<cv::Vec3i> face;
	//std::string obj_path = "origin_meshes\\mesh" + std::to_string(wall_id) + ".obj";
	std::string obj_path = mesh_dir + "mesh" + std::to_string(wall_id) + ".obj";

	CMeshTool meshTool;
	//¶ÁÈ¡obj
	bool isGet = meshTool.GetObjData(obj_path, obj_points, face);
	if (!isGet)
	{
		std::cout << "FilterPlanePointInfo GetObjData false!" << std::endl;
		return false;
	}

	bool isCenter = GetVecPointCenter(obj_points, obj_center);
	if (!isCenter)
	{
		std::cout << "FilterPlanePointInfo isCenter false!" << std::endl;
		return false;
	}

	cv::Mat lookAt;
	CLookAt::GetLookAtMat(inner_normal, plane_center, 100, lookAt);

	std::vector<cv::Point3f> obj_points_lookat;
	CLookAt::RotationPoints(obj_points, obj_points_lookat, lookAt);

	cv::Point3f min_pt;
	cv::Point3f max_pt;
	FindTool::FindMinMaxHelper(obj_points_lookat, min_pt, max_pt);

	std::vector<cv::Point3f> plane_point_lookat;
	CLookAt::RotationPoints(plane_point, plane_point_lookat, lookAt);

	std::vector<cv::Point3f> plane_point_lookat_filter;
	std::vector<unsigned char> reflects_filte_tmp;
	FilterVecPointOutMinMax(min_pt, max_pt, plane_point_lookat, reflects, plane_point_lookat_filter, reflects_filte_tmp);


	cv::Mat lookAt_inv;
	cv::invert(lookAt, lookAt_inv, CV_SVD);

	std::vector<cv::Point3f> plane_point_lookat_filter_ori;
	CLookAt::RotationPoints(plane_point_lookat_filter, plane_point_lookat_filter_ori, lookAt_inv);


	std::vector<std::pair<float, cv::Point3f>> defect_pair_lookat;
	CLookAt::RotationPointsPair(defect_pair, defect_pair_lookat, lookAt);
	std::vector<std::pair<float, cv::Point3f>> defect_pair_lookat_filter;
	FilterOutMinMax(min_pt, max_pt, defect_pair_lookat, defect_pair_lookat_filter);
	std::vector<std::pair<float, cv::Point3f>> defect_pair_lookat_filter_ori;
	CLookAt::RotationPointsPair(defect_pair_lookat_filter, defect_pair_lookat_filter_ori, lookAt_inv);

	plane_point_filter = plane_point_lookat_filter_ori;
	reflects_filter = reflects_filte_tmp;
	defect_pair_filter = defect_pair_lookat_filter_ori;
	return true;
}