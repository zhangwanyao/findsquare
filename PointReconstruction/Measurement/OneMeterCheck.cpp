#include "OneMeterCheck.h"
#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "../Common/CheckPreOp.h"
// if true set circle radius 5cm ,false default is 10cm
#define ONE_METER_CIRCLE_RADIUS_IS_FIVE false 
#define DEBUG_ONE_METER false

COneMeter::COneMeter()
{
}

COneMeter::~COneMeter()
{
}

bool COneMeter::StepOneCheckImgTenPixleCircles(const int& wall_id, const int& pixelHeigh, const cv::Mat& refect_img_ten, std::vector<cv::Vec3f>& vec_circle_big_ten)
{
	return CheckReflectImg10PixleCirclesWithStretch(wall_id, pixelHeigh, refect_img_ten, vec_circle_big_ten);
}

bool COneMeter::StepTwoRefineMoveCircleFindMostFit(const int& wall_id,
	const cv::Mat& refect_img_one,
	const std::vector<cv::Vec3f>& vec_circle_big_ten,
	const cv::Point3f& pt_min,
	const cv::Point3f& pt_max,
	float& one_meter_z)
{
	std::vector<cv::Vec3f> refine_circle;
	std::vector<cv::Point3f> refine_center_point;

	return MoveCircleFindMostFit(wall_id,
		refect_img_one,
		vec_circle_big_ten,
		pt_min,
		pt_max,
		refine_circle,
		refine_center_point,
		one_meter_z);
}


bool COneMeter::CheckReflectImg10PixleCirclesWithStretch(const int& wall_id, const int& pixelHeight,const cv::Mat& refect_img, std::vector<cv::Vec3f>& vec_circle)
{
	cv::Mat refect_img_ten_stretch;
	bool isStretch = CheckPreOp::ContrastStretchBetweenMinMax(refect_img, refect_img_ten_stretch);
	if(!isStretch)
	{ 
		std::cout << "COneMeter::CheckReflectImg10PixleCirclesWithStretch img_ten stretch false!" << std::endl;
		return false;
	}

	if (DEBUG_ONE_METER)
	{
		std::cout << "======wall_id:=========" << wall_id << std::endl;
		cv::imwrite("rst_file\\img_10_" + std::to_string(wall_id) + "_.jpg", refect_img);
	}

	int height = refect_img.rows;
	int width = refect_img.cols;
	if (height<160 || width<95)
	{
		std::cout << "COneMeter::CheckReflectImg10PixleCirclesWithStretch wall height or width too samll!" << std::endl;
		return false;
	}

	cv::Rect rect;
	rect.x = 10;
	rect.y = 10;//adjust up
	rect.height = height - 20;//adjust
	rect.width = width - 20;

	if ((rect.y + rect.height) >= height || (rect.x + rect.width) >= width)
	{
		std::cout << "COneMeter::CheckReflectImg10PixleCirclesWithStretch rect out img_size!" << std::endl;
		return false;
	}
	cv::Mat refect_img_gradxy = CheckPreOp::GetReflectGradXY(refect_img);
	cv::Mat img_rect = refect_img_gradxy(rect).clone();
	CheckPreOp::ContrastStretch(img_rect);//lashen
	if (DEBUG_ONE_METER)
	{
		cv::imwrite("rst_file\\gad_10_" + std::to_string(wall_id) + "_.jpg", refect_img_gradxy);
		cv::imwrite("rst_file\\rect_circle_10_" + std::to_string(wall_id) + "_1.jpg", img_rect);
	}
	std::vector<cv::Vec3f> cir;
	std::vector<cv::Vec3f> cir_big;
	cv::HoughCircles(img_rect, cir, CV_HOUGH_GRADIENT, 1, 20, 100, 17, 10, 12);//10_size ok

	if (cir.size() == 0)
	{
		//std::cout << " no check wall circle!" << std::endl;
		refect_img_gradxy.release();
		return false;
	}
	cv::Mat outMat = img_rect.clone();
	cv::Mat outMat_big = refect_img_gradxy.clone();
	float ave_y = 0;
	for (size_t i = 0; i < cir.size(); i++)
	{
		cir_big.push_back(cv::Vec3f(cir[i][0] + rect.x, cir[i][1] + rect.y, cir[i][2]));
		//cv::Point center(cvRound(cir[i][0]), cvRound(cir[i][1]));
		//ave_y += center.y;
		//int radius = cvRound(cir[i][2]);
		//std::cout << " ===" << i << " radius:" << radius << " center.y:" << center.y << std::endl;
	}
	float wall_true_height = refect_img.rows*pixelHeight;//*10

	if (DEBUG_ONE_METER)
	{
		std::cout << "==cir_big.size()==:" << cir_big.size() << std::endl;
		//cv::Mat outMat2 = img_rect.clone();
		cv::Mat outMat_big2 = refect_img.clone();
		for (size_t i = 0; i < cir_big.size(); i++)
		{
			cv::Point center(cvRound(cir_big[i][0]), cvRound(cir_big[i][1]));
			int radius = cvRound(cir_big[i][2]);
			std::cout << " =cir==" << i << " radius:" << radius << " center.y:" << center.y << std::endl;
		}
		cv::Mat outMat_big2_ldmk = CheckPreOp::PrintCircles(outMat_big2, cir_big);
		cv::imwrite("rst_file\\ldmk_circle_10_" + std::to_string(wall_id) + "_.jpg", outMat_big2_ldmk);
	}

	std::vector<cv::Vec3f> cir_big_filter;
	//filter (900mm-1300mm)outside distance
	for (size_t i = 0; i < cir_big.size(); i++)
	{
		float circle_y_val = wall_true_height - cir_big[i][1] * pixelHeight;
		if (DEBUG_ONE_METER)
		{
			std::cout << "===i:==== " << i << "wall_true_height:" << wall_true_height << " cir_big[i][1]:" << cir_big[i][1] << " circle_y_val:" << circle_y_val << std::endl;
		}

		if (isCircleZFitHeight(circle_y_val))
		{
			cir_big_filter.push_back(cir_big[i]);
		}
	}

	if (0 == cir_big_filter.size())
	{
		std::cout << "COneMeter::CheckReflectImg10PixleCirclesWithStretch filter wall circle!" << std::endl;
		return false;
	}

	vec_circle = cir_big_filter;
	return true;
}

bool COneMeter::isCircleZFitHeight(const float& z_height)
{
	if (z_height >= 900 && z_height <= 1350)
	{
		return true;
	}
	return false;
}

void COneMeter::GetRefineMoveDistance(float& radius, float& move_distance)
{
	//default set fixed circle radius is 100mm
	radius = 100;//1== pixelSize
	move_distance = 24;//1== pixelSize
	if (ONE_METER_CIRCLE_RADIUS_IS_FIVE)
	{
		//fixed circle radius is 50mm
		radius = 50;//1== pixelSize
		move_distance = 12;//1== pixelSize
	}
}


bool COneMeter::MoveCircleFindMostFit(const int& wall_id,
	const cv::Mat& refect_img_one, 
	const std::vector<cv::Vec3f>& vec_circle_big_ten,
	const cv::Point3f& pt_min, 
	const cv::Point3f& pt_max,
	std::vector<cv::Vec3f>& refine_circle,
	std::vector<cv::Point3f>& refine_center_point,
	float & one_meter_z)
{
	int num = vec_circle_big_ten.size();
	if (num == 0)
	{
		std::cout << "COneMeter MoveCicleFindMostFit vec_circle_big_ten circle num is zero!" << std::endl;
		return false;
	}

	cv::Mat refect_img_one_ave = CheckPreOp::ReflectRasterImgAveInterpolation(refect_img_one);

	std::vector<cv::Vec3f> vec_circle_fit;
	CheckPreOp::ChaneTenCircleToFitPixelOneCircle(vec_circle_big_ten, vec_circle_fit);

	float radius;
	float move_distance;
	GetRefineMoveDistance(radius,move_distance);
	std::vector<cv::Vec3f> vec_circle_fit_after;
	bool isFitOk=CheckPreOp::FindFitCircleCentersHelper(refect_img_one, vec_circle_fit, radius,move_distance, vec_circle_fit_after);

	//int find_err_count = 0;
	//for (int s = 0; s < vec_circle_fit.size(); s++)
	//{
	//	cv::Point2f center;
	//	center.x = vec_circle_fit[s][0];
	//	center.y = vec_circle_fit[s][1];
	//	cv::Point2f center_out;
	//	//bool isFind=FindMinCircleValue(refect_img_one_ave, center, radius, move_distance, center_out,false);
	//	bool isFind = CheckPreOp::FindMinCircleValue(refect_img_one, center, radius, move_distance, center_out, true);
	//	if (isFind)
	//	{
	//		vec_circle_fit_after.push_back(cv::Vec3f(center_out.x, center_out.y, radius));
	//	}
	//	else
	//	{
	//		find_err_count++;
	//	}
	//	//std::cout << "==center_out:" << center_out.x << " center_out.y:" << center_out.y << std::endl;

	//}

	if (!isFitOk)
	{
		std::cout << "in COneMeter::FindFitCircleCentersHelper false!" << std::endl;
		return false;
	}
	if (DEBUG_ONE_METER)
	{
		int pixelSize = 1;
		cv::Mat refect_img_one_ave_cir = CheckPreOp::PrintCircles(refect_img_one_ave, vec_circle_fit);
		cv::imwrite("rst_file\\img" + std::to_string(pixelSize) + "_xz_" + std::to_string(wall_id) + "_ave_cir_.jpg", refect_img_one_ave_cir);
		cv::Mat refect_img_one_ave_cir_opt = CheckPreOp::PrintCircles(refect_img_one_ave, vec_circle_fit_after);
		cv::imwrite("rst_file\\img" + std::to_string(pixelSize) + "_xz_" + std::to_string(wall_id) + "_ave_cir_opt_.jpg", refect_img_one_ave_cir_opt);
	}

	int imgHeight = refect_img_one.rows;
	int pixelSize = 1; //pixel_height and pixel_width is 1
	std::vector<cv::Point3f> vec_pt_after;
	float one_meter_z_ave = 0;
	for (int s = 0; s<vec_circle_fit_after.size(); s++)
	{
		cv::Point3f pt;
		pt.x = pt_min.x + vec_circle_fit_after[s][0] * pixelSize;//pixel_width 
		pt.z = pt_min.z + (imgHeight - vec_circle_fit_after[s][1])*pixelSize; //pixel_height
		pt.y = (pt_max.y+pt_min.y)/2;
		float radius = vec_circle_fit_after[s][2];
		one_meter_z_ave += pt.z;
		//std::cout << " ==origin_wall_id==after:" << origin_wall_id << " ==2==s:" << s << " pt.z:" << pt.z << " ave_min_line_z:" << ave_min_line_z << " (pt.z-ave_min_line_z): " << pt.z - ave_min_line_z << " ave_max_line_z:" << ave_max_line_z << " (ave_max_line_z-pt.z): " << ave_max_line_z - pt.z << " radius:" << radius << std::endl;
		vec_pt_after.push_back(pt);
	}
	one_meter_z_ave /= vec_circle_fit_after.size();

	float one_meter_distance = one_meter_z_ave - pt_min.z;
	if (DEBUG_ONE_METER)
	{
		std::cout << "==one_meter_z_ave==" << one_meter_z_ave << " ave_min_line_z:" << pt_min.z << " one_meter_distance:" << one_meter_distance << std::endl;
	}

	//bool isFitDis = (one_meter_distance >= 900 && one_meter_distance <= 1300);
	bool isFitDis = isCircleZFitHeight(one_meter_distance);
	if (!isFitDis) //0.9m outside 1.3m
	{
		std::cout << "in DetectObstacle::GetWallOneMeterCircles one meter distance outside 0.9--1.3 error!" << std::endl;
		return false;
	}

	refine_circle = vec_circle_fit_after;
	refine_center_point = vec_pt_after;
	one_meter_z = one_meter_z_ave;
	return true;

}