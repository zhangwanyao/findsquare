#include "CheckPreOp.h"
#include <iostream>
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>


void CheckPreOp::FindImageFilterZeroMinMax(const cv::Mat &srcImage, float& min, float& max)
{
	min = 256;
	max = -1;
	for (int row = 0; row<srcImage.rows; row++)
	{
		for (int col = 0; col<srcImage.cols; col++)
		{
			int val = srcImage.at<uchar>(row, col);
			if (val>0)
			{
				if (val<min)
				{
					min = val;
				}

				if (val>max)
				{
					max = val;
				}
			}
		}
	}
}

bool CheckPreOp::ContrastStretchBetweenMinMax(const cv::Mat &srcImage, cv::Mat& srcImage_stretch)
{
	srcImage_stretch = srcImage.clone();
	if (srcImage.empty()) {
		std::cout << "CCheckPreOp::ContrastStretchBetweenMinMax image empty" << std::endl;
		return false;
	}

	//double minVal, maxVal;
	//cv::minMaxLoc(srcImage, &minVal, &maxVal);
	//std::cout << "fcvFind min_a=" << pixMin << " max_b=" << pixMax << std::endl;

	float minVal, maxVal;
	FindImageFilterZeroMinMax(srcImage_stretch, minVal, maxVal);
	//std::cout << "minVal=" << minVal << " maxVal=" << maxVal << std::endl;

	//create lut table
	cv::Mat lut(1, 256, CV_8U);
	for (int i = 0; i < 256; i++) {
		if (i < minVal) lut.at<uchar>(i) = 0;
		else if (i > maxVal) lut.at<uchar>(i) = 255;
		else lut.at<uchar>(i) = static_cast<uchar>(255.0*(((float)i - minVal) / (maxVal - minVal)));
	}
	//apply lut
	cv::LUT(srcImage_stretch, lut, srcImage_stretch);
	return true;
}


void CheckPreOp::ContrastStretch(cv::Mat &srcImage)
{
	if (srcImage.empty()) {
		std::cout << "CCheckPreOp::ContrastStretch image empty" << std::endl;
		return;
	}

	//// 计算图像的最大最小值
	//double pixMin, pixMax;
	//cv::minMaxLoc(srcImage, &pixMin, &pixMax);
	//std::cout << "min_a=" << pixMin << " max_b=" << pixMax << std::endl;
	////create lut table
	//cv::Mat lut(1, 256, CV_8U);
	//for (int i = 0; i < 256; i++) {
	//	if (i < pixMin) lut.at<uchar>(i) = 0;
	//	else if (i > pixMax) lut.at<uchar>(i) = 255;
	//	else lut.at<uchar>(i) = static_cast<uchar>(255.0*(i - pixMin) / (pixMax - pixMin) + 0.5);
	//}
	uchar th = 125;
	//create lut table
	cv::Mat lut(1, 256, CV_8U);
	for (int i = 0; i < 256; i++)
	{
		if (i == 0)lut.at<uchar>(i) = 0;
		else if (i > th)lut.at<uchar>(i) = 255;
		else lut.at<uchar>(i) = static_cast<uchar>(255.0*(i - 0) / (th - 0) + 0.5);
	}
	//apply lut
	cv::LUT(srcImage, lut, srcImage);
}

cv::Mat CheckPreOp::GetReflectGradXY(const cv::Mat& reflectRasterImgAve)
{
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y, dst;
	cv::Sobel(reflectRasterImgAve, grad_x, CV_16S, 1, 0, 3, 1, 1, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(grad_x, abs_grad_x);
	//cv::imwrite("reflect_abs_grad_x" + to_string(wall_id) + "_2.jpg", abs_grad_x);

	cv::Sobel(reflectRasterImgAve, grad_y, CV_16S, 0, 1, 3, 1, 1, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(grad_y, abs_grad_y);
	//cv::imwrite("reflect_abs_grad_y" + to_string(wall_id) + "_2.jpg", abs_grad_y);

	cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst);
	//cv::imwrite("reflect_abs_grad_xy" + to_string(wall_id) + "_2.jpg", dst);

	dst.convertTo(dst, CV_8UC1);
	return dst;
}


cv::Mat CheckPreOp::PrintCircles(const cv::Mat& gradXYImg, const std::vector<cv::Vec3f>& vec_circle)
{
	float ave_big_y = 0;
	cv::Mat img = gradXYImg.clone();
	int height_big = img.rows;
	for (size_t i = 0; i < vec_circle.size(); i++) {

		cv::Point center(cvRound(vec_circle[i][0]), cvRound(vec_circle[i][1]));
		ave_big_y += center.y;
		//std::cout << "==i:" << i << " height_dif:" << height_big - center.y << " double*h_diff:" << (height_big - center.y) * 2 << std::endl;
		int radius = cvRound(vec_circle[i][2]);
		//绘制圆心
		cv::circle(img, center, 1, cv::Scalar(255), -1, 8, 0);
		//绘制圆轮廓
		cv::circle(img, center, radius, cv::Scalar(255), 1, 8, 0);
		std::string cicle_id = std::to_string(i);
		cv::putText(img, cicle_id.c_str(), cv::Point(center.x, center.y - radius), cv::FONT_HERSHEY_PLAIN, 2, 0, 1, 4);
	}
	if (vec_circle.size() != 0)
	{
		ave_big_y /= vec_circle.size();
		//std::cout << "======test=====ave_big_y:" << ave_big_y << std::endl;
		cv::line(img, cv::Point(0, ave_big_y), cv::Point(img.cols - 1, ave_big_y), cv::Scalar(255), 1, 8);
	}
	return img;
}

void CheckPreOp::ChangeVecCirce2PointCenter(const std::vector<cv::Vec3f>& vec_circle,std::vector<cv::Point>&vec_center)
{
	vec_center.resize(vec_circle.size());
	for (size_t i = 0; i < vec_circle.size(); i++) {

		cv::Point center(cvRound(vec_circle[i][0]), cvRound(vec_circle[i][1]));
		//int radius = cvRound(vec_circle[i][2]);
		vec_center[i] = center;
	}

}

cv::Mat CheckPreOp::PrintCirclesForAxis(const cv::Mat& gradXYImg, const std::vector<cv::Vec3f>& vec_circle)
{

	cv::Mat img = gradXYImg.clone();
	int height_big = img.rows;
	for (size_t i = 0; i < vec_circle.size(); i++) {

		cv::Point center(cvRound(vec_circle[i][0]), cvRound(vec_circle[i][1]));
		//std::cout << "==i:" << i << " height_dif:" << height_big - center.y << " double*h_diff:" << (height_big - center.y) * 2 << std::endl;
		int radius = cvRound(vec_circle[i][2]);
		//绘制圆心
		cv::circle(img, center, 1, cv::Scalar(255), -1, 8, 0);
		//绘制圆轮廓
		cv::circle(img, center, radius, cv::Scalar(255), 1, 8, 0);
		std::string cicle_id = std::to_string(i);
		cv::putText(img, cicle_id.c_str(), cv::Point(center.x, center.y - radius), cv::FONT_HERSHEY_PLAIN, 2, 0, 1, 4);
	}
	return img;
}


uchar CheckPreOp::GetNeighbourhoodsAveValue(const cv::Mat& img_gray, const int& row, const int& col, const unsigned int& out_num)
{
	float ave_val = 0;
	int count = 0;
	for (int i = row - out_num; i <= row + out_num; i++)
	{
		for (int j = col - out_num; j <= col + out_num; j++)
		{
			if ((i == row) & (j == col))
			{
				continue;
			}

			uchar val = img_gray.at<uchar>(i, j);
			if (val != 0)
			{
				ave_val += val;
				count++;
			}
		}
	}
	if (count != 0)
	{
		ave_val /= count;
	}

	//std::cout <<"("<<row<<","<<col<<")="<< " ave_val:" << int(ave_val) << std::endl;
	return ave_val;
}

cv::Mat CheckPreOp::ReflectRasterImgAveInterpolation(const cv::Mat& reflectRasterImg)
{
	cv::Mat reflectRasterImg_copy = reflectRasterImg.clone();

	unsigned int out_num = 1;
	int row_start = out_num;
	int row_end = reflectRasterImg_copy.rows - out_num;

	int col_start = out_num;
	int col_end = reflectRasterImg_copy.cols - out_num;

	for (int row = row_start; row < row_end; row++)
	{
		for (int col = col_start; col<col_end; col++)
		{
			if (reflectRasterImg_copy.at<uchar>(row, col) == 0)
			{
				reflectRasterImg_copy.at<uchar>(row, col) = GetNeighbourhoodsAveValue(reflectRasterImg_copy, row, col, out_num);
			}

		}
	}
	return reflectRasterImg_copy;
}


cv::Mat CheckPreOp::RefectPixelOneImgInterpolate(const cv::Mat& reflectRasterImgOne)
{
	return ReflectRasterImgAveInterpolation(reflectRasterImgOne);
}


void CheckPreOp::ChaneTenCircleToFitPixelOneCircle(const std::vector<cv::Vec3f>& vec_circle_big_ten, std::vector<cv::Vec3f>& vec_circle_small_one)
{
	float rate = 10;// 10/1;
	int num = vec_circle_big_ten.size();
	vec_circle_small_one.resize(num);
	for (int i = 0; i<num; i++)
	{
		float x = vec_circle_big_ten[i][0] * rate;
		float y = vec_circle_big_ten[i][1] * rate;
		float r = vec_circle_big_ten[i][2] * rate;
		//std::cout << "==heightPixelSmall="<< heightPixelSmall<<" i:" << i << " x:" << x << " y:" << y << " r:" << r << std::endl;
		vec_circle_small_one[i] = cv::Vec3f(x, y, r);
	}
}


float CheckPreOp::CmptCircleVariance(const cv::Mat& img, const float& center_x, const float& center_y, const float& radius, bool isFilterZero)
{
	std::vector<cv::Point> cir_points;
	float ave_vlaue = 0;
	int count = 0;
	int x_st = center_x - radius;
	int x_ed = center_x + radius;
	int y_st = center_y - radius;
	int y_ed = center_y + radius;
	std::vector<float> cir_value;
	for (int h = y_st; h <= y_ed; h++)
	{
		for (int w = x_st; w <= x_ed; w++)
		{
			float distance = std::sqrt((w - center_x)*(w - center_x) + (h - center_y)*(h - center_y));
			if (distance <= radius)
			{
				if (isFilterZero)
				{
					if (img.at<uchar>(h, w)>0)
					{
						ave_vlaue += img.at<uchar>(h, w);
						cir_points.push_back(cv::Point(h, w));
						cir_value.push_back(img.at<uchar>(h, w));
						count++;
					}

				}
				else
				{
					ave_vlaue += img.at<uchar>(h, w);
					cir_points.push_back(cv::Point(h, w));
					cir_value.push_back(img.at<uchar>(h, w));
					count++;
				}

			}
		}
	}

	if (count != 0)
	{
		ave_vlaue /= count;
	}

	float var = 0;
	for (int idx = 0; idx<count; idx++)
	{
		var += (cir_value[idx] - ave_vlaue)*(cir_value[idx] - ave_vlaue);
	}
	//var = sqrt(var)/count;
	var = var / count;
	return var;
}

bool CheckPreOp::FindMinCircleValue(const cv::Mat& img,
	const cv::Point2f& center_in,
	const float& radius_in,
	const float& d,
	cv::Point2f& center_out,
	bool isFilterZero)
{
	int img_width = img.cols;
	int img_height = img.rows;
	float center_x = center_in.x;
	float center_y = center_in.y;
	float radius = radius_in;
	if ((center_x + radius_in) >= img_width || (center_y + radius_in) >= img_height
		|| (center_x - radius_in) <0 || (center_y - radius_in) < 0)
	{
		std::cout << "=FindMinCircleValue out img size station1=" << std::endl;
		return false;
	}

	float min_ct_x = center_x;
	float min_ct_y = center_y;
	int x_st = center_x - d;
	int x_ed = center_x + d;
	int y_st = center_y - d;
	int y_ed = center_y + d;

	if (x_ed >= img_width || y_ed >= img_height || x_st <0 || y_st < 0)
	{
		std::cout << "=FindMinCircleValue out img size station2=" << std::endl;
		return false;
	}
	float min_variance = CmptCircleVariance(img, center_x, center_y, radius, isFilterZero);

	for (int h = y_st; h <= y_ed; h++)
	{
		for (int w = x_st; w <= x_ed; w++)
		{
			float var = CmptCircleVariance(img, w, h, radius, isFilterZero);
			if (var < min_variance)
			{
				min_variance = var;
				min_ct_x = w;
				min_ct_y = h;

			}
		}
	}
	center_out.x = min_ct_x;
	center_out.y = min_ct_y;
	return true;
}


bool CheckPreOp::FindFitCircleCentersHelper(
	const cv::Mat& refect_img_one,
	std::vector<cv::Vec3f> circle_one_pixel,
	const float& radius,
	const float& move_distance,
	std::vector<cv::Vec3f>& circle_fit)
{
	if (circle_one_pixel.size() == 0)
	{
		std::cout << "FindFitCircleCenterHelper circles input size is zero!" << std::endl;
		return false;
	}
	std::vector<cv::Vec3f> vec_circle_fit_after;
	int find_err_count = 0;
	for (int s = 0; s < circle_one_pixel.size(); s++)
	{
		cv::Point2f center;
		center.x = circle_one_pixel[s][0];
		center.y = circle_one_pixel[s][1];
		cv::Point2f center_out;
		//bool isFind=FindMinCircleValue(refect_img_one_ave, center, radius, move_distance, center_out,false);
		bool isFind = CheckPreOp::FindMinCircleValue(refect_img_one, center, radius, move_distance, center_out, true);
		if (isFind)
		{
			vec_circle_fit_after.push_back(cv::Vec3f(center_out.x, center_out.y, radius));
		}
		else
		{
			find_err_count++;
		}
		//std::cout << "==center_out:" << center_out.x << " center_out.y:" << center_out.y << std::endl;

	}

	if (find_err_count >= 1)
	{
		std::cout << "FindFitCircleCenterHelper circles is not equal error!" << std::endl;
		return false;
	}
	circle_fit = vec_circle_fit_after;
	return true;
}
