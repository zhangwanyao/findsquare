#pragma once
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>


namespace ContourPre
{

	bool FilterDefectPairHelper(
		const std::string& mesh_dir,
		const int& wall_id,
		const cv::Point3f& inner_normal,
		const cv::Point3f& plane_center,
		const std::vector<std::pair<float, cv::Point3f>> defect_pair,
		std::vector<std::pair<float, cv::Point3f>>& defect_pair_filter);

	void FilterOutMinMax(
		const cv::Point3f min_pt,
		const cv::Point3f max_pt,
		const std::vector<std::pair<float, cv::Point3f>>& defect_pair_lookat,
		std::vector<std::pair<float, cv::Point3f>>& defect_pair_lookat_filter);


	void FilterVecPointOutMinMax(
		const cv::Point3f min_pt,
		const cv::Point3f max_pt,
		const std::vector<cv::Point3f>& plane_point_lookat,
		const std::vector<unsigned char>& reflects,
		std::vector<cv::Point3f>& plane_point_lookat_filter,
		std::vector<unsigned char>& reflects_filter);

	bool FilterPlanePointHelper(
		const std::string& mesh_dir,
		const int& wall_id,
		const cv::Point3f& inner_normal,
		const cv::Point3f& plane_center,
		const std::vector<cv::Point3f>& plane_point,
		const std::vector<unsigned char>& reflects,
		std::vector<cv::Point3f>& plane_point_filter,
		std::vector<unsigned char>& reflects_filter,
		cv::Point3f& plane_center_filter);


	bool FilterPlanePointInfo(
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
		cv::Point3f& obj_center);
	
}
