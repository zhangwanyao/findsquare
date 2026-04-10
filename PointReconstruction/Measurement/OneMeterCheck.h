#pragma once

#include <opencv2/core/core.hpp> 

class COneMeter
{
public:
	COneMeter();
	~COneMeter();


	bool StepOneCheckImgTenPixleCircles(const int& wall_id, const int& pixelHeigh,const cv::Mat& refect_img_ten, std::vector<cv::Vec3f>& vec_circle_big_ten);

	bool StepTwoRefineMoveCircleFindMostFit(const int& wall_id,
		const cv::Mat& refect_img_one,
		const std::vector<cv::Vec3f>& vec_circle_big_ten,
		const cv::Point3f& pt_min,
		const cv::Point3f& pt_max,
		float& one_meter_z);

private:


	bool CheckReflectImg10PixleCirclesWithStretch(const int& wall_id, const int& pixelHeigh, const cv::Mat& refect_img, std::vector<cv::Vec3f>& vec_circle);

	bool isCircleZFitHeight(const float& z_height);


	void GetRefineMoveDistance(float& radius,float& move_distance);

	bool MoveCircleFindMostFit(const int& wall_id,
		const cv::Mat& refect_img_one,
		const std::vector<cv::Vec3f>& vec_circle_big_ten,
		const cv::Point3f& pt_min,
		const cv::Point3f& pt_max,
		std::vector<cv::Vec3f>& refine_circle,
		std::vector<cv::Point3f>& refine_center_point,
		float& one_meter_z);


};


