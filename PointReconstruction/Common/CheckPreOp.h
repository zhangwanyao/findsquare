#pragma once
#include <opencv2/core/core.hpp> 

namespace CheckPreOp
{


	void FindImageFilterZeroMinMax(const cv::Mat &srcImage, float& min, float& max);

	bool ContrastStretchBetweenMinMax(const cv::Mat &srcImage, cv::Mat& srcImage_stretch);

	void ContrastStretch(cv::Mat &srcImage);

	cv::Mat GetReflectGradXY(const cv::Mat& reflectRasterImgAve);

	cv::Mat PrintCircles(const cv::Mat& gradXYImg, const std::vector<cv::Vec3f>& vec_circle);

	void ChangeVecCirce2PointCenter(const std::vector<cv::Vec3f>& vec_circle, std::vector<cv::Point>&vec_center);
	cv::Mat PrintCirclesForAxis(const cv::Mat& gradXYImg, const std::vector<cv::Vec3f>& vec_circle);


	uchar GetNeighbourhoodsAveValue(const cv::Mat& img_gray, const int& row, const int& col, const unsigned int& out_num);
	cv::Mat ReflectRasterImgAveInterpolation(const cv::Mat& reflectRasterImg);
	cv::Mat RefectPixelOneImgInterpolate(const cv::Mat& reflectRasterImgOne);


	void ChaneTenCircleToFitPixelOneCircle(const std::vector<cv::Vec3f>& vec_circle_big_ten, std::vector<cv::Vec3f>& vec_circle_small_one);


	float CmptCircleVariance(const cv::Mat& img, const float& center_x, const float& center_y, const float& radius, bool isFilterZero);
	bool FindMinCircleValue(const cv::Mat& img,
		const cv::Point2f& center_in,
		const float& radius_in,
		const float& d,
		cv::Point2f& center_out,
		bool isFilterZero);

	bool FindFitCircleCentersHelper(
		const cv::Mat& refect_img_one,
		std::vector<cv::Vec3f> circle_one_pixel,
		const float& radius,
		const float& move_distance,
		std::vector<cv::Vec3f>& circle_fit);
}

